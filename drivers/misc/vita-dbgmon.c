// SPDX-License-Identifier: GPL-2.0
/*
 * PlayStation Vita DevKit: a read-only debug monitor on the debug SDIO link
 * to the CP board, so the PC can fetch the framebuffer (and other memory)
 * through the CP's mini-USB port while Linux runs.
 *
 * The controller at 0xE5800000 (SceSDbgSdio's) is the device end of an SDIO
 * card whose host is the CP board; the CP's sce_sdio.ko offers its four
 * channels to the relay daemon dfcpm, which carries DECI4p between them and
 * the PC's USB link. The card set-up and the transfers follow SceSDbgSdio,
 * with every wait bounded. The driver registers a debug monitor's DECI4p
 * protocol 0x00BE0000 with dfcpm (DCMP 2/1 on channel 0) and serves three
 * of its requests:
 *
 *   HELLO  -> {version, online CPUs, 0, uptime ms} + "vita-dbgmon ..."
 *   READ   {phys, len, width, flags} -> the bytes; System RAM, and the
 *          regions under /reserved-memory and the simple-framebuffers
 *          (CDRAM), nothing else
 *   FB     -> {phys, width | height << 16, stride, DRM fourcc} of what is
 *          scanned out: the display's DRM primary plane when a CRTC is
 *          active, otherwise the simple-framebuffer in the DT
 *
 * Every other request gets BMON_ENOSYS; nothing here writes to memory or
 * registers outside the SDIO controller. The controller takes no interrupt
 * here (SceSDbgSdio polls too); a thread polls the doorbells every 10 ms.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_data/simplefb.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/utsname.h>

#if IS_REACHABLE(CONFIG_DRM) && IS_REACHABLE(CONFIG_DRM_GEM_DMA_HELPER)
#define VDM_DRM 1
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_file.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_plane.h>
#include <drm/drm_rect.h>
#endif

/* ---- the debug SDIO controller ---- */

#define DSDIO_PRESENT		0x040
#define DSDIO_TO_CP(n)		(0x100 + (n) * 0x10)
#define DSDIO_TO_VITA(n)	(0x108 + (n) * 0x10)
#define DSDIO_VITA_WIN(n)	(0xc00 + (n) * 0x100)
#define DSDIO_CP_WIN(n)		(0x800 + (n) * 0x100)
#define DSDIO_CP_VERSION	0x800	/* sce_sdio.ko writes 0x103 when it probes */
#define DSDIO_CHANNELS		4
#define DSDIO_INLINE_MAX	0xe0
#define DSDIO_CHUNK		0x100
#define DSDIO_MAX_MESSAGE	0x11000
#define DSDIO_TIMEOUT_US	5000000

#define CMD_DESCRIPTOR		0x00002
#define CMD_CHUNK		0x00004
#define ACK_STREAM		0x20000
#define ACK_TAKEN		0x40000
#define FLAG_INLINE		2

/* ---- DECI4p ---- */

#define DECI_MAGIC		0x40
#define DECI_HEADER		0x10
#define NODE_PC			0x4000
#define NODE_CP			0x6000
#define NODE_VITA		0xa000
#define PROTOCOL_DCMP		0x100
#define MON_CHANNEL		0

/* ---- the monitor protocol ---- */

#define BMON_PROTOCOL		0x00be0000u
#define BMON_VERSION		1
#define BMON_DATA_MAX		0x10000u
#define BMON_REPLY		0x80000000u

#define BMON_HELLO		1
#define BMON_READ		2
#define BMON_FB			13

#define BMON_OK			0
#define BMON_EINVAL		-1
#define BMON_EFAULT		-2
#define BMON_ENOSYS		-5

struct bmon_request {
	__le32 op;
	__le32 tag;
	__le32 arg[6];
};

struct bmon_reply {
	__le32 op;
	__le32 tag;
	__le32 status;
	__le32 fault[4];
	__le32 result[4];
};

struct vdm {
	struct device *dev;
	void __iomem *base;
	struct task_struct *thread;
	u8 *rx, *tx;
	bool registered;
	bool reinit;
	bool probe_logged;
	unsigned long next_announce;
	struct resource allowed[16];
	unsigned int n_allowed;
};

static u32 vdm_readl(struct vdm *m, u32 off)
{
	return readl(m->base + off);
}

static void vdm_writel(struct vdm *m, u32 val, u32 off)
{
	writel(val, m->base + off);
}

/* The windows take byte accesses, as SceSDbgSdio makes them. */
static void vdm_copy_in(struct vdm *m, void *dst, u32 off, unsigned int len)
{
	u8 *d = dst;

	while (len--)
		*d++ = readb(m->base + off++);
}

static void vdm_copy_out(struct vdm *m, u32 off, const void *src, unsigned int len)
{
	const u8 *s = src;

	while (len--)
		writeb(*s++, m->base + off++);
}

static void vdm_descriptor_out(struct vdm *m, u32 off, u32 a, u32 b, u32 c, u32 d)
{
	const __le32 words[4] = { cpu_to_le32(a), cpu_to_le32(b), cpu_to_le32(c),
				  cpu_to_le32(d) };

	vdm_copy_out(m, off, words, sizeof(words));
}

/* Waits for bits in channel ch's doorbell to the Vita and clears them. */
static bool vdm_wait_bits(struct vdm *m, unsigned int ch, u32 bits)
{
	u32 val;

	if (readl_poll_timeout(m->base + DSDIO_TO_VITA(ch), val, val & bits, 10,
			       DSDIO_TIMEOUT_US))
		return false;
	vdm_writel(m, bits, DSDIO_TO_VITA(ch));
	return true;
}

static void vdm_card_init(struct vdm *m)
{
	/* common CIS: MANFID 0xFF55/0xAABB, FUNCE type 0 (block size 0x800, speed 0x32) */
	static const u8 common_cis[] = {
		0x20, 0x04, 0x55, 0xff, 0xbb, 0xaa, 0x22, 0x04, 0x00, 0x00, 0x08, 0x32, 0xff, 0x01,
	};
	/* function CIS: MANFID, FUNCE type 1 (unit 0, block size 0x800, OCR 0x00FC0000),
	 * FUNCID 0x0C, vendor tuple 0x80 version 0x101 */
	static const u8 function_cis[0x3c] = {
		[0x00] = 0x20, 0x04, 0x55, 0xff, 0xbb, 0xaa, 0x22, 0x2a, 0x01, 0x01,
		[0x15] = 0x08, [0x18] = 0xfc,
		[0x32] = 0x21, 0x02, 0x0c, 0x00, 0x80, 0x02, 0x01, 0x01, 0xff, 0x01,
	};
	unsigned int ch;

	/*
	 * Always a fresh card, even when the Vita OS left one present: taking it
	 * away and back makes the CP drop what it held for the Vita OS and probe
	 * again. Stale doorbell bits go too.
	 */
	vdm_writel(m, 0, DSDIO_PRESENT);
	msleep(100);
	vdm_copy_out(m, 0x400, common_cis, sizeof(common_cis));
	vdm_copy_out(m, 0x480, function_cis, sizeof(function_cis));
	vdm_writel(m, 0, DSDIO_CP_VERSION);
	for (ch = 0; ch < DSDIO_CHANNELS; ch++)
		vdm_writel(m, vdm_readl(m, DSDIO_TO_VITA(ch)), DSDIO_TO_VITA(ch));
	vdm_writel(m, 1, DSDIO_PRESENT);
	m->registered = false;
	m->probe_logged = false;
	m->next_announce = jiffies;
}

/*
 * Polls channel ch once. Returns the length of a message copied to m->rx, 0
 * when none is waiting, or a negative value when the CP stopped answering
 * mid-way (-1) or the message did not fit (-2).
 */
static int vdm_receive(struct vdm *m, unsigned int ch)
{
	u32 desc[4], len, done, chunk;
	u8 *out = m->rx;

	if (!(vdm_readl(m, DSDIO_TO_VITA(ch)) & 3))
		return 0;
	vdm_writel(m, 3, DSDIO_TO_VITA(ch));
	vdm_copy_in(m, desc, DSDIO_CP_WIN(ch), sizeof(desc));
	len = le32_to_cpu((__force __le32)desc[0]);
	if (le32_to_cpu((__force __le32)desc[1]) & FLAG_INLINE) {
		if (!vdm_wait_bits(m, ch, CMD_CHUNK))
			return -1;
		vdm_copy_in(m, out, DSDIO_CP_WIN(ch) + 0x20, min(len, (u32)DSDIO_INLINE_MAX));
		vdm_writel(m, ACK_TAKEN, DSDIO_TO_CP(ch));
		return len <= DSDIO_INLINE_MAX ? (int)len : -2;
	}
	/* A stream: accept it into our window, then take it DSDIO_CHUNK bytes at a time. */
	vdm_descriptor_out(m, DSDIO_CP_WIN(ch) + 0x10, DSDIO_CP_WIN(ch), len, 0, DSDIO_CHUNK);
	vdm_writel(m, ACK_STREAM, DSDIO_TO_CP(ch));
	for (done = 0; done < len; done += chunk) {
		chunk = min(len - done, (u32)DSDIO_CHUNK);
		if (!vdm_wait_bits(m, ch, CMD_CHUNK))
			return -1;
		if (done + chunk <= DSDIO_MAX_MESSAGE)
			vdm_copy_in(m, out + done, DSDIO_CP_WIN(ch), chunk);
		vdm_writel(m, ACK_TAKEN, DSDIO_TO_CP(ch));
	}
	return len <= DSDIO_MAX_MESSAGE ? (int)len : -2;
}

/* Sends one message from m->tx; 0 on success, negative when the CP did not take it. */
static int vdm_transmit(struct vdm *m, unsigned int ch, unsigned int len)
{
	const u8 *in = m->tx;
	u32 done, chunk;

	if (len > DSDIO_MAX_MESSAGE)
		return -3;
	if (len <= DSDIO_INLINE_MAX) {
		vdm_copy_out(m, DSDIO_VITA_WIN(ch) + 0x20, in, len);
		vdm_descriptor_out(m, DSDIO_VITA_WIN(ch), DSDIO_VITA_WIN(ch) + 0x20, len,
				   FLAG_INLINE, DSDIO_CHUNK);
		vdm_writel(m, CMD_DESCRIPTOR, DSDIO_TO_CP(ch));
		return vdm_wait_bits(m, ch, ACK_TAKEN) ? 0 : -1;
	}
	vdm_descriptor_out(m, DSDIO_VITA_WIN(ch), DSDIO_VITA_WIN(ch), len, 0, DSDIO_CHUNK);
	vdm_writel(m, CMD_DESCRIPTOR, DSDIO_TO_CP(ch));
	if (!vdm_wait_bits(m, ch, ACK_STREAM))
		return -1;
	for (done = 0; done < len; done += chunk) {
		chunk = min(len - done, (u32)DSDIO_CHUNK);
		if (done && !vdm_wait_bits(m, ch, ACK_TAKEN))
			return -1;
		vdm_copy_out(m, DSDIO_VITA_WIN(ch), in + done, chunk);
		vdm_writel(m, CMD_CHUNK, DSDIO_TO_CP(ch));
	}
	return vdm_wait_bits(m, ch, ACK_TAKEN) ? 0 : -1;
}

/* ---- DECI4p ---- */

static int vdm_deci_send(struct vdm *m, u16 dst, u32 protocol, unsigned int len)
{
	__le32 *h = (__le32 *)m->tx;
	unsigned int size = DECI_HEADER + len;

	h[0] = cpu_to_le32(size | DECI_MAGIC << 24);
	h[1] = cpu_to_le32(NODE_VITA | (u32)dst << 16);
	h[2] = cpu_to_le32(protocol);
	h[3] = 0;
	return vdm_transmit(m, MON_CHANNEL, size);
}

/* DCMP 2/4 status {subject 0x0F, state 1}, "manager ready", to the CP. */
static int vdm_announce_ready(struct vdm *m)
{
	__le32 *p = (__le32 *)(m->tx + DECI_HEADER);

	p[0] = cpu_to_le32(0x0402);
	p[1] = cpu_to_le32(0x0f | 1 << 8);
	return vdm_deci_send(m, NODE_CP, PROTOCOL_DCMP, 8);
}

/* DCMP 2/1 for BMON_PROTOCOL, registered and open (flags 3). */
static int vdm_announce_protocol(struct vdm *m)
{
	__le32 *p = (__le32 *)(m->tx + DECI_HEADER);

	p[0] = cpu_to_le32(0x0102);
	p[1] = cpu_to_le32(BMON_PROTOCOL);
	p[2] = cpu_to_le32(0x01000000 | BMON_VERSION);
	p[3] = cpu_to_le32(0x00020001);
	p[4] = cpu_to_le32(NODE_VITA);
	p[5] = cpu_to_le32(3);
	return vdm_deci_send(m, NODE_PC, PROTOCOL_DCMP, 0x18);
}

/* ---- READ ---- */

static bool vdm_allowed(struct vdm *m, phys_addr_t addr, u32 len)
{
	unsigned int i;

	for (i = 0; i < m->n_allowed; i++)
		if (addr >= m->allowed[i].start && addr + len - 1 <= m->allowed[i].end)
			return true;
	return false;
}

static int vdm_read_phys(struct vdm *m, phys_addr_t addr, u32 len, u8 *out)
{
	void *p;

	if (region_intersects(addr, len, IORESOURCE_SYSTEM_RAM, IORES_DESC_NONE) ==
	    REGION_INTERSECTS) {
		while (len) {
			unsigned long pfn = PHYS_PFN(addr);
			u32 off = offset_in_page(addr);
			u32 n = min_t(u32, len, PAGE_SIZE - off);

			if (!pfn_valid(pfn))
				return BMON_EFAULT;
			p = kmap_local_page(pfn_to_page(pfn));
			memcpy(out, p + off, n);
			kunmap_local(p);
			addr += n;
			out += n;
			len -= n;
		}
		return BMON_OK;
	}
	if (!vdm_allowed(m, addr, len))
		return BMON_EINVAL;
	/* CDRAM and other carve-outs: uncached, so every read sees the display's data */
	p = memremap(addr, len, MEMREMAP_WC);
	if (!p)
		return BMON_EFAULT;
	memcpy(out, p, len);
	memunmap(p);
	return BMON_OK;
}

/*
 * A node's first reg entry. /chosen carries no "ranges", so its
 * simple-framebuffer does not translate; its reg is a CPU address as it is.
 */
static int vdm_node_reg(struct device_node *np, struct resource *res)
{
	const __be32 *reg;
	u64 size;

	if (!of_address_to_resource(np, 0, res))
		return 0;
	reg = of_get_address(np, 0, &size, NULL);
	if (!reg || !size)
		return -EINVAL;
	*res = DEFINE_RES_MEM(of_read_number(reg, of_n_addr_cells(np)), size);
	return 0;
}

/* Memory the READ request may map besides System RAM. */
static void vdm_collect_allowed(struct vdm *m)
{
	struct device_node *np, *child;
	struct resource res;

	np = of_find_node_by_path("/reserved-memory");
	for_each_available_child_of_node(np, child) {
		if (m->n_allowed < ARRAY_SIZE(m->allowed) && !vdm_node_reg(child, &res))
			m->allowed[m->n_allowed++] = res;
	}
	of_node_put(np);
	for_each_compatible_node(np, NULL, "simple-framebuffer") {
		if (m->n_allowed < ARRAY_SIZE(m->allowed) && !vdm_node_reg(np, &res))
			m->allowed[m->n_allowed++] = res;
	}
}

/* ---- FB ---- */

struct vdm_fb {
	u64 paddr;
	u32 width, height, stride, fourcc;
	char source[96];
};

#ifdef VDM_DRM
struct vdm_drm_ctx {
	struct vdm_fb *fb;
	bool found;
};

/* The first active CRTC's primary plane, for a primary DRM minor under the display. */
static int vdm_drm_minor(struct device *kdev, void *data)
{
	struct vdm_drm_ctx *ctx = data;
	struct drm_minor *minor;
	struct drm_crtc *crtc;

	if (ctx->found || !kdev->class || strcmp(kdev->class->name, "drm"))
		return 0;
	minor = dev_get_drvdata(kdev);
	if (!minor || minor->type != DRM_MINOR_PRIMARY || !minor->dev)
		return 0;
	drm_for_each_crtc(crtc, minor->dev) {
		struct drm_plane *plane = crtc->primary;
		struct drm_plane_state *ps;

		if (!plane)
			continue;
		drm_modeset_lock(&crtc->mutex, NULL);
		drm_modeset_lock(&plane->mutex, NULL);
		ps = plane->state;
		if (crtc->state && crtc->state->active && ps && ps->fb && ps->crtc == crtc) {
			struct drm_framebuffer *fb = ps->fb;

			ctx->fb->paddr = drm_fb_dma_get_gem_addr(fb, ps, 0);
			ctx->fb->width = drm_rect_width(&ps->src) >> 16;
			ctx->fb->height = drm_rect_height(&ps->src) >> 16;
			ctx->fb->stride = fb->pitches[0];
			ctx->fb->fourcc = fb->format->format;
			snprintf(ctx->fb->source, sizeof(ctx->fb->source),
				 "drm %s %s crtc %u fb %u", dev_name(kdev),
				 dev_name(minor->dev->dev), crtc->base.id, fb->base.id);
			ctx->found = true;
		}
		drm_modeset_unlock(&plane->mutex);
		drm_modeset_unlock(&crtc->mutex);
		if (ctx->found)
			break;
	}
	return 0;
}

static bool vdm_fb_drm(struct vdm *m, struct vdm_fb *fb)
{
	struct vdm_drm_ctx ctx = { .fb = fb };
	struct platform_device *pdev;
	struct device_node *np;

	np = of_parse_phandle(m->dev->of_node, "sony,display", 0);
	if (!np)
		return false;
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return false;
	device_lock(&pdev->dev);
	if (pdev->dev.driver)
		device_for_each_child(&pdev->dev, &ctx, vdm_drm_minor);
	device_unlock(&pdev->dev);
	put_device(&pdev->dev);
	return ctx.found;
}
#else
static bool vdm_fb_drm(struct vdm *m, struct vdm_fb *fb)
{
	return false;
}
#endif

static const struct simplefb_format vdm_simplefb_formats[] = SIMPLEFB_FORMATS;

static bool vdm_fb_simple(struct vdm_fb *fb)
{
	struct device_node *np;
	struct resource res;
	const char *format;
	unsigned int i;

	for_each_compatible_node(np, NULL, "simple-framebuffer") {
		if (!of_device_is_available(np) || vdm_node_reg(np, &res) ||
		    of_property_read_u32(np, "width", &fb->width) ||
		    of_property_read_u32(np, "height", &fb->height) ||
		    of_property_read_u32(np, "stride", &fb->stride) ||
		    of_property_read_string(np, "format", &format))
			continue;
		for (i = 0; i < ARRAY_SIZE(vdm_simplefb_formats); i++)
			if (!strcmp(format, vdm_simplefb_formats[i].name))
				break;
		if (i == ARRAY_SIZE(vdm_simplefb_formats))
			continue;
		fb->paddr = res.start;
		fb->fourcc = vdm_simplefb_formats[i].fourcc;
		snprintf(fb->source, sizeof(fb->source), "simple-framebuffer %pOF %s", np,
			 format);
		of_node_put(np);
		return true;
	}
	return false;
}

/* ---- requests ---- */

static bool vdm_width_ok(u32 addr, u32 len, u32 width)
{
	return (width == 1 || width == 2 || width == 4) && !(addr % width) && !(len % width) &&
	       len && len <= BMON_DATA_MAX;
}

/* Handles one request; fills the reply and returns its data length. */
static unsigned int vdm_handle(struct vdm *m, const struct bmon_request *req,
			       struct bmon_reply *rep)
{
	u8 *out = (u8 *)(rep + 1);
	u32 op = le32_to_cpu(req->op);
	u32 arg[6];
	unsigned int i;
	int n;

	for (i = 0; i < ARRAY_SIZE(arg); i++)
		arg[i] = le32_to_cpu(req->arg[i]);
	rep->status = cpu_to_le32(BMON_OK);
	switch (op) {
	case BMON_HELLO:
		rep->result[0] = cpu_to_le32(BMON_VERSION);
		rep->result[1] = cpu_to_le32(cpumask_bits(cpu_online_mask)[0]);
		rep->result[3] = cpu_to_le32(jiffies_to_msecs(get_jiffies_64()));
		n = scnprintf(out, 128, "vita-dbgmon %d on Linux %s %s", BMON_VERSION,
			      init_utsname()->release, init_utsname()->version);
		return n;
	case BMON_READ: {
		int status;

		if (!vdm_width_ok(arg[0], arg[1], arg[2])) {
			rep->status = cpu_to_le32(BMON_EINVAL);
			return 0;
		}
		status = vdm_read_phys(m, arg[0], arg[1], out);
		rep->status = cpu_to_le32(status);
		if (status != BMON_OK)
			return 0;
		rep->result[0] = cpu_to_le32(arg[1]);
		return arg[1];
	}
	case BMON_FB: {
		struct vdm_fb fb = {};

		if (!vdm_fb_drm(m, &fb) && !vdm_fb_simple(&fb)) {
			rep->status = cpu_to_le32(BMON_EINVAL);
			return 0;
		}
		rep->result[0] = cpu_to_le32(fb.paddr);
		rep->result[1] = cpu_to_le32(fb.width | fb.height << 16);
		rep->result[2] = cpu_to_le32(fb.stride);
		rep->result[3] = cpu_to_le32(fb.fourcc);
		n = strlen(fb.source) + 1;
		memcpy(out, fb.source, n);
		return n;
	}
	}
	rep->status = cpu_to_le32(BMON_ENOSYS);
	return 0;
}

static void vdm_serve_request(struct vdm *m, unsigned int ch, unsigned int len)
{
	const struct bmon_request *req = (const void *)(m->rx + DECI_HEADER);
	struct bmon_reply *rep = (void *)(m->tx + DECI_HEADER);
	u16 src = le16_to_cpup((const __le16 *)(m->rx + 4));
	u16 dst = src < NODE_CP ? src : NODE_PC;
	unsigned int n;

	if (len < DECI_HEADER + sizeof(*req)) {
		dev_warn_ratelimited(m->dev, "short request (%u bytes) on port %u\n", len, ch);
		return;
	}
	memset(rep, 0, sizeof(*rep));
	rep->op = cpu_to_le32(le32_to_cpu(req->op) | BMON_REPLY);
	rep->tag = req->tag;
	n = vdm_handle(m, req, rep);
	if (vdm_deci_send(m, dst, BMON_PROTOCOL, sizeof(*rep) + n))
		dev_warn_ratelimited(m->dev, "reply to op %u lost: the CP took no packet\n",
				     le32_to_cpu(req->op));
}

static void vdm_poll(struct vdm *m)
{
	unsigned int ch;
	int len;

	if (!m->probe_logged && vdm_readl(m, DSDIO_CP_VERSION)) {
		m->probe_logged = true;
		dev_info(m->dev, "the CP probed the debug SDIO card\n");
	}
	/* Once the CP has probed, register; repeat every 2 s until it takes it. */
	if (m->probe_logged && !m->registered && time_after_eq(jiffies, m->next_announce)) {
		m->registered = !vdm_announce_ready(m) && !vdm_announce_protocol(m);
		if (m->registered)
			dev_info(m->dev, "registered DECI4p protocol %08x on port %u\n",
				 BMON_PROTOCOL, MON_CHANNEL);
		m->next_announce = jiffies + 2 * HZ;
	}
	/* Every port is drained, so the CP never blocks on one nobody reads. */
	for (ch = 0; ch < DSDIO_CHANNELS; ch++) {
		u32 protocol, size;

		len = vdm_receive(m, ch);
		if (!len)
			continue;
		if (len < 0) {
			dev_warn_ratelimited(m->dev, "port %u: receive failed (%d)\n", ch, len);
			continue;
		}
		if (len < DECI_HEADER)
			continue;
		size = le32_to_cpup((const __le32 *)m->rx) & 0xffffff;
		protocol = le32_to_cpup((const __le32 *)(m->rx + 8));
		if (size != len)
			continue;
		if (protocol == BMON_PROTOCOL) {
			vdm_serve_request(m, ch, len);
		} else if (protocol == PROTOCOL_DCMP && len >= 0x14 && ch == MON_CHANNEL) {
			u32 code = le32_to_cpup((const __le32 *)(m->rx + DECI_HEADER)) & 0xffff;

			/* 0/0 or 0/2: a PC connected; replay the registration as DfMgr does */
			if (code == 0x0000 || code == 0x0200)
				m->registered = !vdm_announce_protocol(m);
		}
	}
}

static int vdm_thread(void *data)
{
	struct vdm *m = data;

	set_freezable();
	vdm_card_init(m);
	while (!kthread_should_stop()) {
		if (try_to_freeze() || READ_ONCE(m->reinit)) {
			WRITE_ONCE(m->reinit, false);
			vdm_card_init(m);
		}
		vdm_poll(m);
		schedule_timeout_interruptible(msecs_to_jiffies(10));
	}
	return 0;
}

static int vdm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vdm *m;

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;
	m->dev = dev;
	m->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(m->base))
		return PTR_ERR(m->base);
	m->rx = devm_kzalloc(dev, DSDIO_MAX_MESSAGE, GFP_KERNEL);
	m->tx = devm_kzalloc(dev, DSDIO_MAX_MESSAGE, GFP_KERNEL);
	if (!m->rx || !m->tx)
		return -ENOMEM;
	vdm_collect_allowed(m);
	platform_set_drvdata(pdev, m);
	m->thread = kthread_run(vdm_thread, m, "vita-dbgmon");
	if (IS_ERR(m->thread))
		return PTR_ERR(m->thread);
	dev_info(dev, "debug monitor on the CP link, protocol %08x\n", BMON_PROTOCOL);
	return 0;
}

static void vdm_remove(struct platform_device *pdev)
{
	struct vdm *m = platform_get_drvdata(pdev);

	kthread_stop(m->thread);
	vdm_writel(m, 0, DSDIO_PRESENT);
}

/* The controller may have lost the card over a suspend: present a fresh one. */
static int vdm_resume(struct device *dev)
{
	struct vdm *m = dev_get_drvdata(dev);

	WRITE_ONCE(m->reinit, true);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(vdm_pm_ops, NULL, vdm_resume);

static const struct of_device_id vdm_of_match[] = {
	{ .compatible = "sony,vita-debug-sdio" },
	{ }
};
MODULE_DEVICE_TABLE(of, vdm_of_match);

static struct platform_driver vdm_driver = {
	.probe = vdm_probe,
	.remove = vdm_remove,
	.driver = {
		.name = "vita-dbgmon",
		.of_match_table = vdm_of_match,
		.pm = pm_sleep_ptr(&vdm_pm_ops),
	},
};
module_platform_driver(vdm_driver);

MODULE_DESCRIPTION("PlayStation Vita DevKit debug monitor over the CP board's debug SDIO link");
MODULE_LICENSE("GPL");
