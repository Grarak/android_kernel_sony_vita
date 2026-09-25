// SPDX-License-Identifier: GPL-2.0
/*
 * PlayStation Vita USB device controller.
 *
 * Kermit carries two Toshiba UDC2 device controllers, bus 1 at 0xE40C0000
 * (the DevKit's mini-USB debug port) and bus 2 at 0xE40D0000 (the
 * multi-connector, or micro-USB on the PCH-2000).  The UDC2 core registers
 * at +0x200 are documented in chapter 3.16 of Toshiba's TMPA910CRAXBG
 * datasheet.  The bridge around the core (+0x000..+0x03C) and the
 * per-endpoint DMA blocks (+0x400 + ep * 0x100) are Kermit's own; what is
 * known of them comes from Sony's udcd.skprx, whose sequences this driver
 * follows (addresses below are udcd.skprx 3.65 function addresses).
 *
 * Endpoint 0 moves data through its FIFO; the other endpoints are set to
 * direct access and move data through the DMA blocks only.  Their directions
 * are fixed: udcd gives IN endpoints the odd numbers and OUT endpoints the
 * even ones (0x81000000), and its DMA interrupt handler tells the two apart
 * by that parity (0x81006f90).
 *
 * The PHY, its clock gate and resets live in Pervasive: the mode select at
 * Misc+0x84, a two-bit power field per bus at Misc+0x17C, reset and gate
 * words at +0x90 of the reset and gate blocks, and in Pervasive2 (0xE3110000)
 * PHY ready (+0xF30), VBUS state (+0xF50) and the VBUS insertion and removal
 * latches (+0xF54).
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#define VITA_UDC_EPS		8

/* Kermit bridge */
#define BR_DMA_INT		0x004	/* bit n: endpoint n's DMA block has status */
#define BR_INT_MASK		0x010
#define BR_INT_CTRL		0x014
#define BR_INT_STATUS		0x024	/* bits 0-7 mirror UD2INT; write 1 to clear 16-21 */
#define BR_INT_SELECT		0x028
#define BR_UNK_2C		0x02c
#define BR_UNK_30		0x030
#define BR_POWER		0x034	/* 2 on suspend, 4 on resume */
#define BR_STATE		0x038

#define BR_INT_RESET_START	BIT(16)
#define BR_INT_RESET_END	BIT(17)
#define BR_INT_SUSPEND		BIT(18)
#define BR_INT_OTHER		GENMASK(21, 19)
#define BR_STATE_SUSPENDED	BIT(1)
#define BR_STATE_LINK		GENMASK(1, 0)
#define BR_STATE_FIFO_CLEAR	BIT(2)

/* Kermit per-endpoint DMA block, endpoints 1-7 */
#define DMA_BLOCK(n)		(0x400 + (n) * 0x100)
#define DMA_MAXP		0x00
#define DMA_BUSY		0x04
#define DMA_CONTROL		0x08
#define DMA_STATUS		0x0c	/* write 1 to clear */
#define DMA_CONFIG		0x10
#define DMA_DESC		0x18
#define DMA_ACTUAL		0x28

#define DMA_CONTROL_RUN		1
#define DMA_CONTROL_CANCEL	2
#define DMA_ST_DONE		BIT(0)
#define DMA_ST_2		BIT(1)
#define DMA_ST_4		BIT(2)
#define DMA_ST_8		BIT(3)
#define DMA_ST_10		BIT(4)
#define DMA_ST_20		BIT(5)
#define DMA_ST_STOP		BIT(6)	/* a busy channel stopped (inferred) */
#define DMA_ST_80		BIT(7)
#define DMA_ST_ABORT		BIT(8)
#define DMA_ST_20000		BIT(17)
#define DMA_ST_RESET		BIT(18)

/* DMA descriptor controls built by udcd (0x81006094) */
#define DESC_IN			0x2c
#define DESC_IN_ISO		0x3c
#define DESC_IN_MORE		0x20	/* a zero-length descriptor follows */
#define DESC_IN_ISO_MORE	0x30
#define DESC_OUT		0x0d
#define DESC_OUT_ISO		0x1d
#define DESC_END		0xffffffff

/* Toshiba UDC2 core */
#define UD2ADR			0x200
#define UD2FRM			0x204
#define UD2TMD			0x208
#define UD2CMD			0x20c
#define UD2BRQ			0x210
#define UD2WVL			0x214
#define UD2WIDX			0x218
#define UD2WLGTH		0x21c
#define UD2INT			0x220
#define UD2INTEP		0x224
#define UD2INTEPMSK		0x228
#define UD2INTRX0		0x22c
#define UD2EPMSZ(n)		(0x230 + (n) * 0x10)
#define UD2EPSTS(n)		(0x234 + (n) * 0x10)
#define UD2EPDSZ(n)		(0x238 + (n) * 0x10)
#define UD2EPFIFO(n)		(0x23c + (n) * 0x10)

#define ADR_DEFAULT		BIT(8)
#define ADR_ADDRESSED		BIT(9)
#define ADR_CONFIGURED		BIT(10)
#define ADR_SPEED		GENMASK(13, 12)
#define ADR_SPEED_HIGH		2

#define CMD_SETUP_FIN		0x1
#define CMD_SET_DATA0		0x2
#define CMD_EP_RESET		0x3
#define CMD_EP_STALL		0x4
#define CMD_EP_INVALID		0x5
#define CMD_EP_DISABLE		0x7
#define CMD_EP_ENABLE		0x8
#define CMD_ALL_EP_INVALID	0x9
#define CMD_USB_READY		0xa
#define CMD_SETUP_RECEIVED	0xb
#define CMD_EP_EOP		0xc
#define CMD_EP_FIFO_CLEAR	0xd
#define CMD_EP_TX_0DATA		0xe

#define INT_SETUP		BIT(0)
#define INT_STATUS_NAK		BIT(1)
#define INT_STATUS		BIT(2)
#define INT_RX_ZERO		BIT(3)
#define INT_SOF			BIT(4)
#define INT_EP0			BIT(5)
#define INT_EP			BIT(6)
#define INT_NAK			BIT(7)
#define INT_M_SOF		BIT(12)
/* udcd's mask: NAK, EP, SOF and RX_ZERO off (0x81006a94) */
#define INT_MASK_DEFAULT	0xd800

#define MSZ_DSET		BIT(12)
#define MSZ_TX_0DATA		BIT(15)

#define STS_PKT_DUAL		BIT(15)
#define STS_DIRECT		BIT(14)
#define STS_STATUS		GENMASK(11, 9)
#define STS_STATUS_STALL	(3 << 9)
#define STS_DISABLE		BIT(8)
#define STS_DIR_IN		BIT(7)
#define STS_TYPE_ISO		(1 << 2)
#define STS_TYPE_BULK		(2 << 2)
#define STS_TYPE_INT		(3 << 2)

/* Pervasive */
#define MISC_USB_MODE(bus)	(0x84 + (bus) * 4)	/* 1: device */
#define MISC_USB_POWER		0x17c			/* two bits per bus */
#define PERV_USB(bus)		(0x90 + (bus) * 4)	/* reset and gate words */
#define PERV_USB_HOST		BIT(0)
#define PERV_USB_DEVICE		BIT(1)
#define PERV_USB_PHY		BIT(3)
#define PERV2_PHY_READY		0xf30
#define PERV2_PHY_IRQ		0xf34
#define PERV2_VBUS		0xf50
#define PERV2_VBUS_IRQ		0xf54			/* insertion bit 4n, removal 4n+1 */
#define PERV2_F58		0xf58

struct vita_udc_desc {
	__le32 control;
	__le32 addr;
	__le32 length;
	__le32 next;
};

struct vita_udc_req {
	struct usb_request req;
	struct list_head queue;
};

struct vita_udc;

struct vita_udc_ep {
	struct usb_ep ep;
	struct vita_udc *udc;
	struct list_head queue;
	unsigned int num;
	bool in;
	bool iso;
	bool enabled;
	bool busy;		/* the head request is in the DMA block */
	bool halted;
	bool wedged;
	struct vita_udc_desc *desc;
	dma_addr_t desc_dma;
	char name[8];
};

enum vita_udc_ep0_state {
	EP0_IDLE,
	EP0_IN_DATA,		/* Control-RD data stage */
	EP0_IN_STATUS,		/* Control-RD data sent, Setup_Fin on the status NAK */
	EP0_OUT_DATA,		/* Control-WR data stage */
	EP0_NODATA,		/* waiting for the gadget's zero-length status request */
	EP0_STATUS,		/* Setup_Fin given, waiting for the status stage */
};

struct vita_udc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct device *dev;
	spinlock_t lock;

	void __iomem *regs;
	void __iomem *misc;
	void __iomem *reset;
	void __iomem *gate;
	void __iomem *perv2;
	unsigned int bus;

	struct gpio_desc *vbus_gpio;	/* PCH-1000: GPIO0 port 5 */
	struct gpio_desc *switch_gpio;	/* PCH-1000: GPIO0 port 24 */

	struct vita_udc_ep ep[VITA_UDC_EPS];
	struct vita_udc_desc *descs;
	dma_addr_t descs_dma;

	enum vita_udc_ep0_state ep0_state;
	struct usb_ctrlrequest ctrl;
	unsigned int ep0_sent;		/* bytes of the IN request written to the FIFO */
	bool ep0_zlp;			/* a zero-length packet ends the IN data stage */
	u8 ep0_buf[2];
	struct usb_request *ep0_req;	/* GET_STATUS answers */

	u32 int_mask;
	u8 address;
	u8 test_mode;
	bool running;			/* controller started, USB_Ready given */
	bool pullup;
	bool vbus;
	bool selfpowered;
	bool suspended;
};

static inline struct vita_udc_ep *to_vita_ep(struct usb_ep *ep)
{
	return container_of(ep, struct vita_udc_ep, ep);
}

static inline struct vita_udc_req *to_vita_req(struct usb_request *req)
{
	return container_of(req, struct vita_udc_req, req);
}

static inline struct vita_udc *to_vita_udc(struct usb_gadget *gadget)
{
	return container_of(gadget, struct vita_udc, gadget);
}

static inline u32 udc_read(struct vita_udc *udc, u32 reg)
{
	return readl(udc->regs + reg);
}

static inline void udc_write(struct vita_udc *udc, u32 reg, u32 val)
{
	writel(val, udc->regs + reg);
}

static inline u32 dma_read(struct vita_udc_ep *ep, u32 reg)
{
	return readl(ep->udc->regs + DMA_BLOCK(ep->num) + reg);
}

static inline void dma_write(struct vita_udc_ep *ep, u32 reg, u32 val)
{
	writel(val, ep->udc->regs + DMA_BLOCK(ep->num) + reg);
}

/* Misc+0x17C and Pervasive2 +0xF58 hold fields of both buses. */
static DEFINE_SPINLOCK(vita_udc_perv_lock);

static void perv_update(void __iomem *reg, u32 clear, u32 set)
{
	unsigned long flags;

	spin_lock_irqsave(&vita_udc_perv_lock, flags);
	writel((readl(reg) & ~clear) | set, reg);
	dsb(sy);
	spin_unlock_irqrestore(&vita_udc_perv_lock, flags);
}

static void udc_int_ack(struct vita_udc *udc, u32 bits)
{
	udc_write(udc, UD2INT, udc->int_mask | bits);
}

/*
 * Issue a UDC2 command and wait until it has taken effect: commands issued
 * during a transfer are held until it ends, and a second command for the same
 * endpoint would overwrite a held one (TMPA910 3.16.3.3 (1) 4).  The waits
 * are udcd's (0x81005b8c), with its 1 ms bound.
 */
static void udc_cmd(struct vita_udc *udc, unsigned int ep, unsigned int cmd)
{
	u32 val;

	udc_write(udc, UD2CMD, ep << 4 | cmd);

	switch (cmd) {
	case CMD_SET_DATA0:
		readl_poll_timeout_atomic(udc->regs + UD2EPSTS(ep), val,
					  !(val & GENMASK(13, 12)), 1, 1000);
		break;
	case CMD_EP_RESET:
		readl_poll_timeout_atomic(udc->regs + UD2EPSTS(ep), val,
					  !(val & GENMASK(11, 8)), 1, 1000);
		break;
	case CMD_EP_STALL:
		readl_poll_timeout_atomic(udc->regs + UD2EPSTS(ep), val,
					  (val & STS_STATUS) == STS_STATUS_STALL,
					  1, 1000);
		break;
	case CMD_EP_INVALID:
		readl_poll_timeout_atomic(udc->regs + UD2EPSTS(ep), val,
					  (val & STS_STATUS) == STS_STATUS,
					  1, 1000);
		break;
	case CMD_EP_DISABLE:
		readl_poll_timeout_atomic(udc->regs + UD2EPSTS(ep), val,
					  val & STS_DISABLE, 1, 1000);
		break;
	case CMD_EP_ENABLE:
		readl_poll_timeout_atomic(udc->regs + UD2EPSTS(ep), val,
					  !(val & STS_DISABLE), 1, 1000);
		break;
	case CMD_EP_FIFO_CLEAR:
		readl_poll_timeout_atomic(udc->regs + UD2EPDSZ(ep), val,
					  !val && !(udc_read(udc, UD2EPMSZ(ep)) &
						    MSZ_DSET), 1, 1000);
		break;
	case CMD_EP_TX_0DATA:
		readl_poll_timeout_atomic(udc->regs + UD2EPMSZ(ep), val,
					  val & MSZ_TX_0DATA, 1, 1000);
		break;
	default:
		break;
	}
}

/* ---------------------------------------------------------------------- */
/* PHY and controller power, after SceUsbServ and udcd */

static bool vita_udc_vbus_present(struct vita_udc *udc)
{
	bool vbus = readl(udc->perv2 + PERV2_VBUS) & BIT(udc->bus);

	/*
	 * Where VBUS comes through a GPIO, udcd wants both it and the
	 * Pervasive2 bit before it starts the controller (0x81006554).
	 */
	if (udc->vbus_gpio)
		vbus = vbus && gpiod_get_value_cansleep(udc->vbus_gpio) > 0;
	return vbus;
}

static int vita_udc_wait_phy(struct vita_udc *udc)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(udc->perv2 + PERV2_PHY_READY, val,
				 val & BIT(udc->bus), 10, 10000);
	writel(BIT(udc->bus), udc->perv2 + PERV2_PHY_IRQ);
	if (ret)
		dev_err(udc->dev, "USB PHY %u did not become ready\n", udc->bus);
	return ret;
}

/* The PHY after a resume, udcd's ten short polls (0x81008814): about 1 ms. */
static void vita_udc_wait_phy_atomic(struct vita_udc *udc)
{
	u32 val;

	if (readl_poll_timeout_atomic(udc->perv2 + PERV2_PHY_READY, val,
				      val & BIT(udc->bus), 10, 1000))
		dev_warn(udc->dev, "USB PHY %u not ready after resume\n", udc->bus);
	writel(BIT(udc->bus), udc->perv2 + PERV2_PHY_IRQ);
}

/*
 * Device mode, as SceUsbServ selects it before starting udcd on the bus
 * (usbserv.skprx 0x81000d48): hold the host and device sides and the PHY in
 * reset with their clocks off, select device mode, then release the device
 * side and the PHY.
 */
static int vita_udc_select_device_mode(struct vita_udc *udc)
{
	void __iomem *reset = udc->reset + PERV_USB(udc->bus);
	void __iomem *gate = udc->gate + PERV_USB(udc->bus);
	u32 all = PERV_USB_HOST | PERV_USB_DEVICE | PERV_USB_PHY;
	u32 device = PERV_USB_DEVICE | PERV_USB_PHY;

	perv_update(reset, 0, all);
	perv_update(gate, all, 0);
	writel(1, udc->misc + MISC_USB_MODE(udc->bus));
	dsb(sy);
	perv_update(gate, 0, device);
	perv_update(reset, device, 0);
	return vita_udc_wait_phy(udc);
}

/*
 * Quiesce the bus, ScePervasiveForDriver_4F64DEF6: everything back in reset
 * with its clocks off, device mode kept, the PHY out of reset and its power
 * field set to 2.  udcd does this when it enables the PHY and on every VBUS
 * removal (0x81006980, 0x81005d40).
 */
static void vita_udc_quiesce(struct vita_udc *udc)
{
	void __iomem *reset = udc->reset + PERV_USB(udc->bus);
	void __iomem *gate = udc->gate + PERV_USB(udc->bus);
	u32 all = PERV_USB_HOST | PERV_USB_DEVICE | PERV_USB_PHY;

	perv_update(reset, 0, all);
	perv_update(gate, all, 0);
	writel(1, udc->misc + MISC_USB_MODE(udc->bus));
	dsb(sy);
	perv_update(reset, PERV_USB_PHY, 0);
	perv_update(udc->misc + MISC_USB_POWER, 3 << (udc->bus * 2),
		    2 << (udc->bus * 2));
}

static void vita_udc_ep_reset_state(struct vita_udc *udc);

/*
 * Controller start on VBUS, udcd 0x8100671c: pulse the PHY reset with its
 * power field cleared, clock and release the device side, program the bridge,
 * then USB_Ready pulls D+ up.
 */
static int vita_udc_start_controller(struct vita_udc *udc)
{
	void __iomem *reset = udc->reset + PERV_USB(udc->bus);
	void __iomem *gate = udc->gate + PERV_USB(udc->bus);
	u32 device = PERV_USB_DEVICE | PERV_USB_PHY;
	unsigned long flags;
	int ret;

	perv_update(reset, 0, PERV_USB_PHY);
	perv_update(udc->misc + MISC_USB_POWER, 3 << (udc->bus * 2), 0);
	perv_update(gate, 0, device);
	perv_update(reset, device, 0);
	perv_update(udc->perv2 + PERV2_F58, 3 << (udc->bus * 4), 0);

	udc_write(udc, BR_UNK_2C, 0);
	udc_write(udc, BR_UNK_30, 0);
	ret = vita_udc_wait_phy(udc);
	if (ret)
		return ret;

	spin_lock_irqsave(&udc->lock, flags);
	udc->int_mask = INT_MASK_DEFAULT;
	udc_write(udc, BR_INT_MASK, 0xfffffffe);
	udc_write(udc, BR_INT_CTRL, 1);
	udc_write(udc, BR_INT_SELECT, 0x00380000);
	udc_int_ack(udc, INT_SOF);
	if (udc->switch_gpio)
		gpiod_set_value(udc->switch_gpio, 1);
	udc->address = 0;
	udc->ep0_state = EP0_IDLE;
	udc->suspended = false;
	udc->running = true;
	udc_write(udc, UD2EPMSZ(0), 64);
	udc_cmd(udc, 0, CMD_USB_READY);
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

static void vita_udc_nuke_all(struct vita_udc *udc, int status);

/* Controller stop, udcd's VBUS removal path (0x81005d40). */
static void vita_udc_stop_controller(struct vita_udc *udc)
{
	unsigned long flags;

	spin_lock_irqsave(&udc->lock, flags);
	udc->running = false;
	vita_udc_nuke_all(udc, -ESHUTDOWN);
	vita_udc_ep_reset_state(udc);
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	if (udc->switch_gpio)
		gpiod_set_value(udc->switch_gpio, 0);
	spin_unlock_irqrestore(&udc->lock, flags);

	vita_udc_quiesce(udc);
}

/* ---------------------------------------------------------------------- */
/* Requests */

static void vita_udc_done(struct vita_udc_ep *ep, struct vita_udc_req *req,
			  int status)
	__releases(&ep->udc->lock) __acquires(&ep->udc->lock)
{
	struct vita_udc *udc = ep->udc;

	list_del_init(&req->queue);
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	if (ep->num)
		usb_gadget_unmap_request(&udc->gadget, &req->req, ep->in);

	spin_unlock(&udc->lock);
	usb_gadget_giveback_request(&ep->ep, &req->req);
	spin_lock(&udc->lock);
}

/*
 * Stop an endpoint's DMA; returns the OUT byte count it had reached.  udcd
 * (0x81006260) only takes the run bit away from a busy channel and leaves
 * the request in flight until the channel reports the stop, DMA status bit
 * 6 or 8, whose interrupt stops it again, now idle, and ends the request
 * with an error (0x81006f90).  Here the stop is awaited in place, with
 * udcd's 1 ms bound, so a late stop interrupt cannot end the next request
 * instead.  The idle channel is then cancelled, the cancel awaited, and its
 * status acknowledged.
 */
static u32 vita_udc_dma_stop(struct vita_udc_ep *ep)
{
	struct vita_udc *udc = ep->udc;
	void __iomem *dma = udc->regs + DMA_BLOCK(ep->num);
	u32 val, actual;

	if (readl(dma + DMA_BUSY) & 1) {
		writel(readl(dma + DMA_CONTROL) & ~DMA_CONTROL_RUN,
		       dma + DMA_CONTROL);
		if (readl_poll_timeout_atomic(dma + DMA_STATUS, val,
					      (val & (DMA_ST_STOP | DMA_ST_ABORT)) ||
					      !(readl(dma + DMA_BUSY) & 1) ||
					      (udc_read(udc, BR_STATE) & BR_STATE_LINK),
					      1, 1000))
			dev_warn_once(udc->dev, "%s: DMA stop not reported within 1 ms\n",
				      ep->name);
	}
	actual = readl(dma + DMA_ACTUAL);
	writel(DMA_CONTROL_CANCEL, dma + DMA_CONTROL);
	readl_poll_timeout_atomic(dma + DMA_CONTROL, val,
				  val == DMA_CONTROL_CANCEL ||
				  (udc_read(udc, BR_STATE) & BR_STATE_LINK),
				  1, 1000);
	writel(0, dma + DMA_CONTROL);
	writel(readl(dma + DMA_STATUS), dma + DMA_STATUS);
	ep->busy = false;
	return actual;
}

/* Abort, udcd 0x81006260: stop the DMA, clear the FIFO if the bridge lets it. */
static u32 vita_udc_dma_abort(struct vita_udc_ep *ep)
{
	u32 actual = vita_udc_dma_stop(ep);

	if (udc_read(ep->udc, BR_STATE) & BR_STATE_FIFO_CLEAR)
		udc_cmd(ep->udc, ep->num, CMD_EP_FIFO_CLEAR);
	return actual;
}

/*
 * After a stop the head request could only start over from its first byte.
 * An OUT transfer that had not received a byte loses nothing by that and
 * stays queued; any other goes back with what is known of it, as udcd ends
 * every stopped transfer with an error.
 */
static void vita_udc_give_back_stopped(struct vita_udc_ep *ep, u32 actual,
				       int status)
{
	struct vita_udc_req *req;

	if (list_empty(&ep->queue))
		return;
	req = list_first_entry(&ep->queue, struct vita_udc_req, queue);
	actual = ep->in ? 0 : min(actual, req->req.length);
	if (!ep->in && !actual)
		return;
	req->req.actual = actual;
	vita_udc_done(ep, req, status);
}

static void vita_udc_nuke(struct vita_udc_ep *ep, int status)
{
	struct vita_udc_req *req;

	if (ep->num && ep->busy)
		vita_udc_dma_abort(ep);
	ep->busy = false;
	while (!list_empty(&ep->queue)) {
		req = list_first_entry(&ep->queue, struct vita_udc_req, queue);
		vita_udc_done(ep, req, status);
	}
}

static void vita_udc_nuke_all(struct vita_udc *udc, int status)
{
	unsigned int n;

	for (n = 0; n < VITA_UDC_EPS; n++)
		vita_udc_nuke(&udc->ep[n], status);
	udc->ep0_state = EP0_IDLE;
}

/*
 * Hand the head request to the endpoint's DMA block: one descriptor for the
 * whole buffer, and for an IN transfer that is a whole number of packets and
 * wants one, a chained zero-length descriptor (udcd 0x81006094, 0x81006158).
 */
static void vita_udc_dma_kick(struct vita_udc_ep *ep)
{
	struct vita_udc_req *req;
	struct vita_udc_desc *d = ep->desc;
	u32 maxp = usb_endpoint_maxp(ep->ep.desc);
	unsigned int len;

	if (ep->busy || ep->halted || list_empty(&ep->queue) ||
	    !ep->udc->running || ep->udc->suspended)
		return;
	req = list_first_entry(&ep->queue, struct vita_udc_req, queue);
	len = req->req.length;

	d[0].addr = cpu_to_le32(len ? req->req.dma : 0);
	d[0].length = cpu_to_le32(len);
	d[0].next = cpu_to_le32(DESC_END);
	if (!ep->in) {
		d[0].control = cpu_to_le32(ep->iso ? DESC_OUT_ISO : DESC_OUT);
	} else if (req->req.zero && len && !(len % maxp)) {
		d[0].control = cpu_to_le32(ep->iso ? DESC_IN_ISO_MORE : DESC_IN_MORE);
		d[0].next = cpu_to_le32(ep->desc_dma + sizeof(*d));
		d[1].control = cpu_to_le32(ep->iso ? DESC_IN_ISO : DESC_IN);
		d[1].addr = 0;
		d[1].length = 0;
		d[1].next = cpu_to_le32(DESC_END);
	} else {
		d[0].control = cpu_to_le32(ep->iso ? DESC_IN_ISO : DESC_IN);
	}

	ep->busy = true;
	dma_write(ep, DMA_DESC, ep->desc_dma);
	dma_write(ep, DMA_CONTROL, DMA_CONTROL_RUN);
}

/* ---------------------------------------------------------------------- */
/* Endpoint 0 */

static void vita_udc_ep0_stall(struct vita_udc *udc)
{
	udc_cmd(udc, 0, CMD_EP_STALL);
	udc->ep0_state = EP0_IDLE;
}

static void vita_udc_ep0_setup_fin(struct vita_udc *udc)
{
	udc_cmd(udc, 0, CMD_SETUP_FIN);
	udc->ep0_state = EP0_STATUS;
}

/*
 * Write the next packet of the IN data stage, udcd 0x81007ebc: 16 bits per
 * FIFO write, an odd last byte as a byte write, and EP_EOP for a short
 * packet.  A data stage that is a whole number of packets and shorter than
 * the host asked for ends with EP_TX_0DATA.
 */
static void vita_udc_ep0_write(struct vita_udc *udc)
{
	struct vita_udc_ep *ep = &udc->ep[0];
	struct vita_udc_req *req;
	unsigned int left, count, n;
	u8 *buf;

	if (list_empty(&ep->queue))
		return;
	req = list_first_entry(&ep->queue, struct vita_udc_req, queue);

	if (udc_read(udc, UD2EPDSZ(0)) ||
	    (udc_read(udc, UD2EPMSZ(0)) & (MSZ_DSET | MSZ_TX_0DATA)))
		return;		/* the previous packet has not gone yet */

	left = req->req.length - udc->ep0_sent;
	if (!left) {
		if (udc->ep0_zlp) {
			udc->ep0_zlp = false;
			udc_cmd(udc, 0, CMD_EP_TX_0DATA);
			return;
		}
		req->req.actual = req->req.length;
		udc->ep0_state = EP0_IN_STATUS;
		vita_udc_done(ep, req, 0);
		return;
	}

	count = min(left, 64u);
	buf = (u8 *)req->req.buf + udc->ep0_sent;
	for (n = 0; n + 1 < count; n += 2)
		udc_write(udc, UD2EPFIFO(0), buf[n] | buf[n + 1] << 8);
	if (n < count)
		writeb(buf[n], udc->regs + UD2EPFIFO(0));
	udc->ep0_sent += count;
	if (count < 64)
		udc_cmd(udc, 0, CMD_EP_EOP);
}

/* Read what the FIFO holds of the OUT data stage (udcd 0x810076d0, i_ep0). */
static void vita_udc_ep0_read(struct vita_udc *udc)
{
	struct vita_udc_ep *ep = &udc->ep[0];
	struct vita_udc_req *req;
	unsigned int size, n, room;
	u8 *buf;
	u32 val;

	if (list_empty(&ep->queue))
		return;		/* the FIFO keeps the packet until a request comes */
	req = list_first_entry(&ep->queue, struct vita_udc_req, queue);

	size = udc_read(udc, UD2EPDSZ(0)) & 0x7f;
	room = req->req.length - req->req.actual;
	buf = (u8 *)req->req.buf + req->req.actual;
	for (n = 0; n < size; n += 2) {
		val = udc_read(udc, UD2EPFIFO(0));
		if (n < room)
			buf[n] = val;
		if (n + 1 < size && n + 1 < room)
			buf[n + 1] = val >> 8;
	}
	req->req.actual += min(size, room);

	if (size < 64 || req->req.actual >= req->req.length ||
	    req->req.actual >= le16_to_cpu(udc->ctrl.wLength)) {
		vita_udc_done(ep, req, size > room ? -EOVERFLOW : 0);
		vita_udc_ep0_setup_fin(udc);
	}
}

static int vita_udc_ep0_queue(struct vita_udc *udc, struct vita_udc_req *req)
{
	struct vita_udc_ep *ep = &udc->ep[0];

	if (!list_empty(&ep->queue))
		return -EBUSY;

	switch (udc->ep0_state) {
	case EP0_IN_DATA:
		list_add_tail(&req->queue, &ep->queue);
		udc->ep0_sent = 0;
		/*
		 * A data stage shorter than the host asked for ends with a short
		 * packet, so a whole number of packets, or none, needs a
		 * zero-length one.
		 */
		udc->ep0_zlp = req->req.length < le16_to_cpu(udc->ctrl.wLength) &&
			       !(req->req.length % 64);
		vita_udc_ep0_write(udc);
		return 0;
	case EP0_OUT_DATA:
		list_add_tail(&req->queue, &ep->queue);
		if (udc_read(udc, UD2EPDSZ(0)))
			vita_udc_ep0_read(udc);
		return 0;
	case EP0_NODATA:
		/* The status stage of a request without data, or a delayed one. */
		if (req->req.length)
			return -EINVAL;
		list_add_tail(&req->queue, &ep->queue);
		vita_udc_ep0_setup_fin(udc);
		vita_udc_done(ep, req, 0);
		return 0;
	default:
		return -EL2HLT;
	}
}

static void vita_udc_ep0_reply_done(struct usb_ep *ep, struct usb_request *req)
{
}

static void vita_udc_ep0_reply(struct vita_udc *udc, u16 value)
{
	struct usb_request *req = udc->ep0_req;

	put_unaligned_le16(value, udc->ep0_buf);
	req->buf = udc->ep0_buf;
	req->length = min_t(u16, 2, le16_to_cpu(udc->ctrl.wLength));
	req->zero = false;
	req->complete = vita_udc_ep0_reply_done;
	udc->ep0_state = EP0_IN_DATA;
	vita_udc_ep0_queue(udc, to_vita_req(req));
}

static struct vita_udc_ep *vita_udc_find_ep(struct vita_udc *udc, u16 windex)
{
	unsigned int num = windex & USB_ENDPOINT_NUMBER_MASK;
	struct vita_udc_ep *ep;

	if (num >= VITA_UDC_EPS)
		return NULL;
	ep = &udc->ep[num];
	if (num && (!ep->enabled || ep->in != !!(windex & USB_DIR_IN)))
		return NULL;
	return ep;
}

static void vita_udc_ep_stall(struct vita_udc_ep *ep);
static void vita_udc_ep_clear_halt(struct vita_udc_ep *ep);

/* The standard requests this driver answers itself; true if it did. */
static bool vita_udc_std_request(struct vita_udc *udc)
{
	struct usb_ctrlrequest *ctrl = &udc->ctrl;
	u16 value = le16_to_cpu(ctrl->wValue);
	u16 index = le16_to_cpu(ctrl->wIndex);
	u8 recip = ctrl->bRequestType & USB_RECIP_MASK;
	struct vita_udc_ep *ep;

	if ((ctrl->bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD)
		return false;

	switch (ctrl->bRequest) {
	case USB_REQ_SET_ADDRESS:
		/* udcd sets the address in the SETUP handler (0x81007294). */
		udc->address = value & 0x7f;
		udc_write(udc, UD2ADR, udc->address |
			  (udc->address ? ADR_ADDRESSED : ADR_DEFAULT));
		usb_gadget_set_state(&udc->gadget, udc->address ?
				     USB_STATE_ADDRESS : USB_STATE_DEFAULT);
		vita_udc_ep0_setup_fin(udc);
		return true;

	case USB_REQ_GET_STATUS:
		if (!(ctrl->bRequestType & USB_DIR_IN))
			return false;
		if (recip == USB_RECIP_DEVICE) {
			vita_udc_ep0_reply(udc, udc->selfpowered <<
					   USB_DEVICE_SELF_POWERED);
			return true;
		}
		if (recip == USB_RECIP_ENDPOINT) {
			ep = vita_udc_find_ep(udc, index);
			if (!ep)
				vita_udc_ep0_stall(udc);
			else
				vita_udc_ep0_reply(udc, ep->halted);
			return true;
		}
		return false;

	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
		if (recip == USB_RECIP_ENDPOINT && value == USB_ENDPOINT_HALT) {
			ep = vita_udc_find_ep(udc, index);
			if (!ep || !ep->num) {
				vita_udc_ep0_stall(udc);
				return true;
			}
			if (ctrl->bRequest == USB_REQ_SET_FEATURE) {
				vita_udc_ep_stall(ep);
			} else if (!ep->wedged) {
				vita_udc_ep_clear_halt(ep);
			}
			vita_udc_ep0_setup_fin(udc);
			return true;
		}
		if (recip == USB_RECIP_DEVICE && ctrl->bRequest == USB_REQ_SET_FEATURE &&
		    value == USB_DEVICE_TEST_MODE) {
			/* Entered after the status stage (0x810076d0, i_status). */
			udc->test_mode = index >> 8;
			vita_udc_ep0_setup_fin(udc);
			return true;
		}
		if (recip == USB_RECIP_DEVICE && value == USB_DEVICE_REMOTE_WAKEUP) {
			vita_udc_ep0_setup_fin(udc);
			return true;
		}
		return false;

	default:
		return false;
	}
}

static void vita_udc_handle_setup(struct vita_udc *udc)
{
	struct usb_ctrlrequest *ctrl = &udc->ctrl;
	bool set_config;
	u32 brq;
	int ret;

	brq = udc_read(udc, UD2BRQ);
	ctrl->bRequestType = brq & 0xff;
	ctrl->bRequest = brq >> 8;
	ctrl->wValue = cpu_to_le16(udc_read(udc, UD2WVL));
	ctrl->wIndex = cpu_to_le16(udc_read(udc, UD2WIDX));
	ctrl->wLength = cpu_to_le16(udc_read(udc, UD2WLGTH));

	/* A new SETUP ends whatever the last control transfer left. */
	vita_udc_nuke(&udc->ep[0], -ECONNRESET);
	udc_cmd(udc, 0, CMD_SETUP_RECEIVED);

	if (!ctrl->wLength)
		udc->ep0_state = EP0_NODATA;
	else if (ctrl->bRequestType & USB_DIR_IN)
		udc->ep0_state = EP0_IN_DATA;
	else
		udc->ep0_state = EP0_OUT_DATA;

	if (vita_udc_std_request(udc))
		return;

	if (!udc->driver) {
		vita_udc_ep0_stall(udc);
		return;
	}

	set_config = (ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD &&
		     ctrl->bRequest == USB_REQ_SET_CONFIGURATION;
	if (set_config) {
		/*
		 * udcd sets the device state when the request arrives, before
		 * the status stage (0x81007294).
		 */
		if (le16_to_cpu(ctrl->wValue)) {
			udc_write(udc, UD2ADR, udc->address | ADR_CONFIGURED);
		} else {
			udc_write(udc, UD2ADR, udc->address | ADR_ADDRESSED);
			udc_cmd(udc, 0, CMD_ALL_EP_INVALID);
		}
	}

	spin_unlock(&udc->lock);
	ret = udc->driver->setup(&udc->gadget, ctrl);
	spin_lock(&udc->lock);

	if (ret < 0) {
		if (set_config)
			udc_write(udc, UD2ADR, udc->address | ADR_ADDRESSED);
		vita_udc_ep0_stall(udc);
	}
}

/* ---------------------------------------------------------------------- */
/* Interrupts */

static void vita_udc_bus_reset(struct vita_udc *udc)
{
	vita_udc_nuke_all(udc, -ESHUTDOWN);
	udc->address = 0;
	udc->suspended = false;
	if (udc->switch_gpio)
		gpiod_set_value(udc->switch_gpio, 1);
	if (udc->driver && udc->gadget.speed != USB_SPEED_UNKNOWN) {
		spin_unlock(&udc->lock);
		usb_gadget_udc_reset(&udc->gadget, udc->driver);
		spin_lock(&udc->lock);
	}
	udc->gadget.speed = USB_SPEED_UNKNOWN;
}

static void vita_udc_bus_reset_end(struct vita_udc *udc)
{
	u32 adr = udc_read(udc, UD2ADR);

	if (udc->switch_gpio)
		gpiod_set_value(udc->switch_gpio, 0);
	udc->gadget.speed = FIELD_GET(ADR_SPEED, adr) == ADR_SPEED_HIGH ?
			    USB_SPEED_HIGH : USB_SPEED_FULL;
	/* The data endpoints report through the DMA blocks. */
	udc_write(udc, UD2INTEPMSK, 0xff);
	usb_gadget_set_state(&udc->gadget, USB_STATE_DEFAULT);
}

/*
 * Suspend and resume, udcd 0x810076d0 on bridge bit 18.  udcd cancels every
 * endpoint on suspend, and on resume, once the PHY is back, aborts each one
 * with requests before its class drivers attach again.  Here the requests
 * stay queued across the suspend, no transfer starts while it lasts, and a
 * transfer the suspend stopped goes back only if it cannot start over.
 */
static void vita_udc_suspend_change(struct vita_udc *udc)
{
	struct usb_gadget_driver *driver = udc->driver;
	struct vita_udc_ep *ep;
	unsigned int n;
	u32 actual;

	if (udc_read(udc, BR_STATE) & BR_STATE_SUSPENDED) {
		if (udc->suspended)
			return;
		udc->suspended = true;
		udc_write(udc, BR_POWER, 2);
		for (n = 1; n < VITA_UDC_EPS; n++) {
			ep = &udc->ep[n];
			if (!ep->busy)
				continue;
			actual = vita_udc_dma_abort(ep);
			vita_udc_give_back_stopped(ep, actual, -ECONNRESET);
		}
		if (driver && driver->suspend) {
			spin_unlock(&udc->lock);
			driver->suspend(&udc->gadget);
			spin_lock(&udc->lock);
		}
	} else {
		udc_write(udc, BR_POWER, 4);
		vita_udc_wait_phy_atomic(udc);
		/* One SOF confirms the bus is back (0x810076d0). */
		udc->int_mask &= ~INT_M_SOF;
		udc_int_ack(udc, INT_SOF);
		if (!udc->suspended)
			return;
		udc->suspended = false;
		for (n = 1; n < VITA_UDC_EPS; n++) {
			ep = &udc->ep[n];
			if (list_empty(&ep->queue))
				continue;
			vita_udc_dma_abort(ep);
			vita_udc_dma_kick(ep);
		}
		if (driver && driver->resume) {
			spin_unlock(&udc->lock);
			driver->resume(&udc->gadget);
			spin_lock(&udc->lock);
		}
	}
}

static irqreturn_t vita_udc_irq(int irq, void *data)
{
	struct vita_udc *udc = data;
	u32 status, flags;

	spin_lock(&udc->lock);
	status = udc_read(udc, BR_INT_STATUS);
	flags = status & 0xff & ~(udc->int_mask >> 8);
	status &= BR_INT_RESET_START | BR_INT_RESET_END | BR_INT_SUSPEND |
		  BR_INT_OTHER;
	if (!flags && !status) {
		spin_unlock(&udc->lock);
		return IRQ_NONE;
	}
	/* udcd drops the controller's interrupts while VBUS is off (0x810076d0). */
	if (!udc->running ||
	    !(readl(udc->perv2 + PERV2_VBUS) & BIT(udc->bus))) {
		udc_int_ack(udc, flags);
		udc_write(udc, BR_INT_STATUS, status);
		spin_unlock(&udc->lock);
		return IRQ_HANDLED;
	}

	if (flags & INT_NAK)
		udc_int_ack(udc, INT_NAK);

	if (flags & INT_EP0) {
		udc_int_ack(udc, INT_EP0);
		if (udc->ep0_state == EP0_IN_DATA)
			vita_udc_ep0_write(udc);
		else if (udc->ep0_state == EP0_OUT_DATA)
			vita_udc_ep0_read(udc);
	}

	if (flags & INT_RX_ZERO)
		udc_int_ack(udc, INT_RX_ZERO);

	if (flags & INT_SETUP) {
		udc_int_ack(udc, INT_SETUP);
		vita_udc_handle_setup(udc);
	}

	if (flags & INT_STATUS_NAK) {
		udc_int_ack(udc, INT_STATUS_NAK);
		/* The host is in the status stage: the data stage is over. */
		if (udc->ep0_state == EP0_IN_STATUS ||
		    (udc->ep0_state == EP0_IN_DATA &&
		     list_empty(&udc->ep[0].queue)))
			vita_udc_ep0_setup_fin(udc);
	}

	if (flags & INT_STATUS) {
		udc_int_ack(udc, INT_STATUS);
		if (udc->ep0_state == EP0_STATUS)
			udc->ep0_state = EP0_IDLE;
		if (udc->test_mode) {
			udc_write(udc, UD2TMD, udc->test_mode);
			udc->test_mode = 0;
		}
	}

	if (flags & INT_SOF) {
		udc->int_mask |= INT_M_SOF;
		udc_int_ack(udc, INT_SOF);
	}

	if (status & BR_INT_RESET_START) {
		udc_write(udc, BR_INT_STATUS, BR_INT_RESET_START);
		vita_udc_bus_reset(udc);
	}
	if (status & BR_INT_RESET_END) {
		udc_write(udc, BR_INT_STATUS, BR_INT_RESET_END);
		vita_udc_bus_reset_end(udc);
	}
	if (status & BR_INT_SUSPEND) {
		udc_write(udc, BR_INT_STATUS, BR_INT_SUSPEND);
		vita_udc_suspend_change(udc);
	}
	if (status & BR_INT_OTHER)
		udc_write(udc, BR_INT_STATUS, status & BR_INT_OTHER);

	spin_unlock(&udc->lock);
	return IRQ_HANDLED;
}

/* One endpoint's DMA status, udcd 0x81006f90. */
static void vita_udc_dma_ep_irq(struct vita_udc_ep *ep, bool vbus)
{
	struct vita_udc_req *req = NULL;
	u32 st = dma_read(ep, DMA_STATUS);
	bool busy = ep->busy, done = false, count = false;
	int status = 0;

	if (!list_empty(&ep->queue))
		req = list_first_entry(&ep->queue, struct vita_udc_req, queue);

	if (!vbus) {
		/*
		 * udcd ignores DMA interrupts while VBUS is off and cancels
		 * everything when it handles the removal.  The line is level,
		 * so the status is acknowledged here, and the transfer it
		 * belongs to goes back now rather than staying in flight.
		 */
		dma_write(ep, DMA_STATUS, st);
		if (req && busy) {
			vita_udc_dma_abort(ep);
			vita_udc_done(ep, req, -ESHUTDOWN);
		}
		return;
	}

	if (st & DMA_ST_DONE) {
		dma_write(ep, DMA_STATUS, st & (DMA_ST_DONE | DMA_ST_4 | DMA_ST_8));
		done = true;
	} else if (st & (DMA_ST_4 | DMA_ST_8)) {
		dma_write(ep, DMA_STATUS, st & (DMA_ST_4 | DMA_ST_8));
		done = !ep->in;
	} else if (st & DMA_ST_10) {
		/*
		 * udcd ends an OUT transfer with bit 4 with its byte count and
		 * error -4 (0x81005a1c).  What the bit reports is unknown; a
		 * transfer larger than the buffer is a guess.
		 */
		dma_write(ep, DMA_STATUS, DMA_ST_10);
		done = !ep->in;
		status = -EOVERFLOW;
		count = true;
	} else if (st & DMA_ST_STOP) {
		dma_write(ep, DMA_STATUS, DMA_ST_STOP);
		vita_udc_dma_abort(ep);
		done = true;
		status = -EIO;
	} else if (st & DMA_ST_ABORT) {
		dma_write(ep, DMA_STATUS, DMA_ST_ABORT);
		vita_udc_dma_abort(ep);
		done = true;
		status = -ECONNRESET;
	} else {
		dma_write(ep, DMA_STATUS, st & (DMA_ST_2 | DMA_ST_20 | DMA_ST_80 |
						DMA_ST_20000 | DMA_ST_RESET));
		if (st & DMA_ST_RESET) {
			done = true;
			status = -ESHUTDOWN;
		}
	}

	/* The abort above has cleared busy already. */
	if (!done || !req || !busy)
		return;
	ep->busy = false;
	if (!status || count)
		req->req.actual = ep->in ? req->req.length :
				  min(dma_read(ep, DMA_ACTUAL), req->req.length);
	vita_udc_done(ep, req, status);
	vita_udc_dma_kick(ep);
}

static irqreturn_t vita_udc_dma_irq(int irq, void *data)
{
	struct vita_udc *udc = data;
	unsigned int n;
	u32 pending;
	bool vbus;

	spin_lock(&udc->lock);
	pending = udc_read(udc, BR_DMA_INT) & GENMASK(VITA_UDC_EPS - 1, 1);
	if (!pending) {
		spin_unlock(&udc->lock);
		return IRQ_NONE;
	}
	vbus = udc->running && (readl(udc->perv2 + PERV2_VBUS) & BIT(udc->bus));
	for (n = 1; n < VITA_UDC_EPS; n++)
		if (pending & BIT(n))
			vita_udc_dma_ep_irq(&udc->ep[n], vbus);
	spin_unlock(&udc->lock);
	return IRQ_HANDLED;
}

/*
 * The core only records a connect asked for while no gadget driver is bound,
 * and a recorded connect makes the one at bind a no-op
 * (usb_gadget_connect_locked), so the pullup would never happen. VBUS is
 * therefore reported only to a bound driver; the core assumes it present.
 */
static void vita_udc_report_vbus(struct vita_udc *udc)
{
	unsigned long flags;
	bool bound;

	spin_lock_irqsave(&udc->lock, flags);
	bound = udc->driver;
	spin_unlock_irqrestore(&udc->lock, flags);
	if (bound)
		usb_udc_vbus_handler(&udc->gadget, udc->vbus);
}

static void vita_udc_vbus_changed(struct vita_udc *udc)
{
	bool vbus = vita_udc_vbus_present(udc);

	if (vbus == udc->vbus)
		return;
	udc->vbus = vbus;
	dev_dbg(udc->dev, "VBUS %s\n", vbus ? "on" : "off");
	vita_udc_report_vbus(udc);
}

/* VBUS insertion and removal latches, udcd 0x81006870. */
static irqreturn_t vita_udc_vbus_irq(int irq, void *data)
{
	struct vita_udc *udc = data;
	u32 bits = 3 << (udc->bus * 4);
	u32 latched = readl(udc->perv2 + PERV2_VBUS_IRQ) & bits;

	if (!latched)
		return IRQ_NONE;
	writel(latched, udc->perv2 + PERV2_VBUS_IRQ);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t vita_udc_vbus_thread(int irq, void *data)
{
	vita_udc_vbus_changed(data);
	return IRQ_HANDLED;
}

/* ---------------------------------------------------------------------- */
/* Endpoint operations */

static void vita_udc_ep_reset_state(struct vita_udc *udc)
{
	unsigned int n;

	for (n = 1; n < VITA_UDC_EPS; n++) {
		udc->ep[n].enabled = false;
		udc->ep[n].halted = false;
		udc->ep[n].wedged = false;
		udc->ep[n].busy = false;
		udc->ep[n].ep.desc = NULL;
	}
}

/*
 * Stall a data endpoint, udcd 0x81006de0: stop its DMA, clear its FIFO
 * (always, unlike an abort) and give EP_Stall.  udcd issues the stall of a
 * busy channel from its stop interrupt; vita_udc_dma_stop has awaited that
 * stop already.  The transfer the stop cut off goes back to the gadget
 * unless it can start over once the halt is cleared.
 */
static void vita_udc_ep_stall(struct vita_udc_ep *ep)
{
	struct vita_udc *udc = ep->udc;
	bool busy = ep->busy;
	u32 actual;

	ep->halted = true;
	actual = vita_udc_dma_stop(ep);
	udc_cmd(udc, ep->num, CMD_EP_FIFO_CLEAR);
	udc_cmd(udc, ep->num, CMD_EP_STALL);
	if (busy)
		vita_udc_give_back_stopped(ep, actual, -ECONNRESET);
}

static void vita_udc_ep_clear_halt(struct vita_udc_ep *ep)
{
	struct vita_udc *udc = ep->udc;

	/* EP_Reset clears the stall and the data toggle (udcd 0x81007294). */
	if (ep->busy)
		vita_udc_dma_abort(ep);
	udc_cmd(udc, ep->num, CMD_EP_RESET);
	ep->halted = false;
	ep->wedged = false;
	vita_udc_dma_kick(ep);
}

/* Endpoint set-up, udcd 0x81006370. */
static int vita_udc_ep_enable(struct usb_ep *_ep,
			      const struct usb_endpoint_descriptor *desc)
{
	struct vita_udc_ep *ep = to_vita_ep(_ep);
	struct vita_udc *udc = ep->udc;
	unsigned long flags;
	u32 maxp, sts;
	int type;

	if (!ep->num || !desc || desc->bDescriptorType != USB_DT_ENDPOINT ||
	    usb_endpoint_dir_in(desc) != ep->in)
		return -EINVAL;
	maxp = usb_endpoint_maxp(desc);
	if (!maxp || maxp > ep->ep.maxpacket_limit)
		return -ERANGE;

	type = usb_endpoint_type(desc);
	sts = STS_DIRECT | (ep->in ? STS_DIR_IN : 0);
	if ((ep->num <= 2 && maxp <= 1024) || (ep->num > 2 && maxp <= 512))
		sts |= STS_PKT_DUAL;
	switch (type) {
	case USB_ENDPOINT_XFER_BULK:
		sts |= STS_TYPE_BULK;
		break;
	case USB_ENDPOINT_XFER_INT:
		sts |= STS_TYPE_INT;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		sts |= STS_TYPE_ISO;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&udc->lock, flags);
	ep->ep.desc = desc;
	ep->ep.maxpacket = maxp;
	ep->iso = type == USB_ENDPOINT_XFER_ISOC;
	ep->halted = false;
	ep->wedged = false;
	ep->busy = false;

	udc_write(udc, UD2EPMSZ(ep->num), maxp + (maxp & 1));
	udc_write(udc, UD2EPSTS(ep->num), sts);
	dma_write(ep, DMA_MAXP, maxp);
	udc_cmd(udc, ep->num, CMD_EP_DISABLE);
	udc_cmd(udc, ep->num, CMD_EP_RESET);
	dma_write(ep, DMA_CONFIG, 0x10000);
	ep->enabled = true;
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

static int vita_udc_ep_disable(struct usb_ep *_ep)
{
	struct vita_udc_ep *ep = to_vita_ep(_ep);
	struct vita_udc *udc = ep->udc;
	unsigned long flags;

	if (!ep->num)
		return -EINVAL;

	spin_lock_irqsave(&udc->lock, flags);
	vita_udc_nuke(ep, -ESHUTDOWN);
	if (udc->running)
		udc_cmd(udc, ep->num, CMD_EP_INVALID);
	ep->enabled = false;
	ep->ep.desc = NULL;
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

static struct usb_request *vita_udc_alloc_request(struct usb_ep *_ep,
						  gfp_t gfp)
{
	struct vita_udc_req *req;

	req = kzalloc(sizeof(*req), gfp);
	if (!req)
		return NULL;
	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void vita_udc_free_request(struct usb_ep *_ep, struct usb_request *req)
{
	kfree(to_vita_req(req));
}

static int vita_udc_ep_queue(struct usb_ep *_ep, struct usb_request *_req,
			     gfp_t gfp)
{
	struct vita_udc_ep *ep = to_vita_ep(_ep);
	struct vita_udc_req *req = to_vita_req(_req);
	struct vita_udc *udc = ep->udc;
	unsigned long flags;
	int ret;

	if (!_req || !_req->buf || !list_empty(&req->queue))
		return -EINVAL;

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	spin_lock_irqsave(&udc->lock, flags);
	if (!udc->running || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		ret = -ESHUTDOWN;
		goto out;
	}

	if (!ep->num) {
		ret = vita_udc_ep0_queue(udc, req);
		goto out;
	}

	if (!ep->enabled) {
		ret = -ESHUTDOWN;
		goto out;
	}
	ret = usb_gadget_map_request(&udc->gadget, _req, ep->in);
	if (ret)
		goto out;
	list_add_tail(&req->queue, &ep->queue);
	vita_udc_dma_kick(ep);
out:
	spin_unlock_irqrestore(&udc->lock, flags);
	return ret;
}

static int vita_udc_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct vita_udc_ep *ep = to_vita_ep(_ep);
	struct vita_udc *udc = ep->udc;
	struct vita_udc_req *req, *iter;
	unsigned long flags;
	bool head;
	u32 actual;

	spin_lock_irqsave(&udc->lock, flags);
	req = NULL;
	list_for_each_entry(iter, &ep->queue, queue) {
		if (&iter->req == _req) {
			req = iter;
			break;
		}
	}
	if (!req) {
		spin_unlock_irqrestore(&udc->lock, flags);
		return -EINVAL;
	}

	head = list_is_first(&req->queue, &ep->queue);
	if (head && ep->num && ep->busy) {
		actual = vita_udc_dma_abort(ep);
		req->req.actual = ep->in ? 0 : min(actual, req->req.length);
	}
	vita_udc_done(ep, req, -ECONNRESET);
	if (head && ep->num)
		vita_udc_dma_kick(ep);
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

static int vita_udc_ep_set_halt_wedge(struct usb_ep *_ep, int value, bool wedge)
{
	struct vita_udc_ep *ep = to_vita_ep(_ep);
	struct vita_udc *udc = ep->udc;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&udc->lock, flags);
	if (!udc->running) {
		ret = -ESHUTDOWN;
	} else if (!ep->num) {
		if (value)
			vita_udc_ep0_stall(udc);
	} else if (ep->iso) {
		ret = -EINVAL;
	} else if (value) {
		if (ep->in && !list_empty(&ep->queue)) {
			ret = -EAGAIN;
		} else {
			vita_udc_ep_stall(ep);
			ep->wedged = wedge;
		}
	} else {
		vita_udc_ep_clear_halt(ep);
	}
	spin_unlock_irqrestore(&udc->lock, flags);
	return ret;
}

static int vita_udc_ep_set_halt(struct usb_ep *ep, int value)
{
	return vita_udc_ep_set_halt_wedge(ep, value, false);
}

static int vita_udc_ep_set_wedge(struct usb_ep *ep)
{
	return vita_udc_ep_set_halt_wedge(ep, 1, true);
}

static const struct usb_ep_ops vita_udc_ep_ops = {
	.enable		= vita_udc_ep_enable,
	.disable	= vita_udc_ep_disable,
	.alloc_request	= vita_udc_alloc_request,
	.free_request	= vita_udc_free_request,
	.queue		= vita_udc_ep_queue,
	.dequeue	= vita_udc_ep_dequeue,
	.set_halt	= vita_udc_ep_set_halt,
	.set_wedge	= vita_udc_ep_set_wedge,
};

/* ---------------------------------------------------------------------- */
/* Gadget operations */

static int vita_udc_get_frame(struct usb_gadget *gadget)
{
	return udc_read(to_vita_udc(gadget), UD2FRM) & 0x7ff;
}

static int vita_udc_set_selfpowered(struct usb_gadget *gadget, int value)
{
	to_vita_udc(gadget)->selfpowered = value;
	return 0;
}

static int vita_udc_pullup(struct usb_gadget *gadget, int is_on)
{
	struct vita_udc *udc = to_vita_udc(gadget);

	if (!!is_on == udc->pullup)
		return 0;
	udc->pullup = is_on;
	if (is_on)
		return vita_udc_start_controller(udc);
	vita_udc_stop_controller(udc);
	return 0;
}

static int vita_udc_start(struct usb_gadget *gadget,
			  struct usb_gadget_driver *driver)
{
	struct vita_udc *udc = to_vita_udc(gadget);
	unsigned long flags;

	spin_lock_irqsave(&udc->lock, flags);
	udc->driver = driver;
	spin_unlock_irqrestore(&udc->lock, flags);
	if (!udc->vbus)
		usb_udc_vbus_handler(gadget, false);
	return 0;
}

static int vita_udc_stop(struct usb_gadget *gadget)
{
	struct vita_udc *udc = to_vita_udc(gadget);
	unsigned long flags;

	spin_lock_irqsave(&udc->lock, flags);
	vita_udc_nuke_all(udc, -ESHUTDOWN);
	udc->driver = NULL;
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

static struct usb_ep *vita_udc_match_ep(struct usb_gadget *gadget,
					struct usb_endpoint_descriptor *desc,
					struct usb_ss_ep_comp_descriptor *comp)
{
	struct vita_udc *udc = to_vita_udc(gadget);
	bool in = usb_endpoint_dir_in(desc);
	u32 maxp = usb_endpoint_maxp(desc);
	int n;

	/*
	 * Keep endpoints 1 and 2, the ones with the larger FIFOs, for bulk
	 * transfers: take the highest free endpoint for anything else.
	 */
	if (usb_endpoint_type(desc) == USB_ENDPOINT_XFER_BULK)
		return NULL;
	for (n = VITA_UDC_EPS - 1; n > 0; n--) {
		struct vita_udc_ep *ep = &udc->ep[n];

		if (ep->in == in && !ep->ep.claimed &&
		    maxp <= ep->ep.maxpacket_limit)
			return &ep->ep;
	}
	return NULL;
}

static const struct usb_gadget_ops vita_udc_gadget_ops = {
	.get_frame		= vita_udc_get_frame,
	.set_selfpowered	= vita_udc_set_selfpowered,
	.pullup			= vita_udc_pullup,
	.udc_start		= vita_udc_start,
	.udc_stop		= vita_udc_stop,
	.match_ep		= vita_udc_match_ep,
};

/* ---------------------------------------------------------------------- */

static void vita_udc_init_eps(struct vita_udc *udc)
{
	unsigned int n;

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	for (n = 0; n < VITA_UDC_EPS; n++) {
		struct vita_udc_ep *ep = &udc->ep[n];

		ep->udc = udc;
		ep->num = n;
		ep->in = n & 1;
		INIT_LIST_HEAD(&ep->queue);
		ep->desc = &udc->descs[n * 2];
		ep->desc_dma = udc->descs_dma + n * 2 * sizeof(*ep->desc);
		ep->ep.ops = &vita_udc_ep_ops;
		ep->ep.name = ep->name;
		if (!n) {
			strscpy(ep->name, "ep0", sizeof(ep->name));
			ep->ep.caps.type_control = true;
			ep->ep.caps.dir_in = true;
			ep->ep.caps.dir_out = true;
			usb_ep_set_maxpacket_limit(&ep->ep, 64);
			udc->gadget.ep0 = &ep->ep;
			continue;
		}
		snprintf(ep->name, sizeof(ep->name), "ep%u%s", n,
			 ep->in ? "in" : "out");
		ep->ep.caps.type_iso = true;
		ep->ep.caps.type_bulk = true;
		ep->ep.caps.type_int = true;
		ep->ep.caps.dir_in = ep->in;
		ep->ep.caps.dir_out = !ep->in;
		usb_ep_set_maxpacket_limit(&ep->ep, n <= 2 ? 1024 : 512);
		list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
	}
}

static void __iomem *vita_udc_map(struct platform_device *pdev, const char *name)
{
	struct resource *res;

	/*
	 * The Pervasive blocks are shared with other drivers (the reset
	 * controller claims its block), so map them without claiming.
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return IOMEM_ERR_PTR(-EINVAL);
	return devm_ioremap(&pdev->dev, res->start, resource_size(res)) ?:
	       IOMEM_ERR_PTR(-ENOMEM);
}

static int vita_udc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vita_udc *udc;
	int irq, dma_irq, vbus_irq, ret;
	u32 bus;

	udc = devm_kzalloc(dev, sizeof(*udc), GFP_KERNEL);
	if (!udc)
		return -ENOMEM;
	udc->dev = dev;
	spin_lock_init(&udc->lock);

	if (of_property_read_u32(dev->of_node, "vita,usb-bus", &bus) ||
	    bus < 1 || bus > 2)
		return dev_err_probe(dev, -EINVAL, "vita,usb-bus must be 1 or 2\n");
	udc->bus = bus;

	udc->regs = devm_platform_ioremap_resource_byname(pdev, "udc");
	if (IS_ERR(udc->regs))
		return PTR_ERR(udc->regs);
	udc->misc = vita_udc_map(pdev, "misc");
	udc->reset = vita_udc_map(pdev, "reset");
	udc->gate = vita_udc_map(pdev, "gate");
	udc->perv2 = vita_udc_map(pdev, "pervasive2");
	if (IS_ERR(udc->misc) || IS_ERR(udc->reset) || IS_ERR(udc->gate) ||
	    IS_ERR(udc->perv2))
		return dev_err_probe(dev, -EINVAL, "missing Pervasive registers\n");

	udc->vbus_gpio = devm_gpiod_get_optional(dev, "vbus", GPIOD_IN);
	if (IS_ERR(udc->vbus_gpio))
		return PTR_ERR(udc->vbus_gpio);
	udc->switch_gpio = devm_gpiod_get_optional(dev, "switch", GPIOD_OUT_LOW);
	if (IS_ERR(udc->switch_gpio))
		return PTR_ERR(udc->switch_gpio);

	irq = platform_get_irq_byname(pdev, "udc");
	dma_irq = platform_get_irq_byname(pdev, "dma");
	vbus_irq = platform_get_irq_byname(pdev, "vbus");
	if (irq < 0 || dma_irq < 0 || vbus_irq < 0)
		return -ENXIO;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	udc->descs = dmam_alloc_coherent(dev, VITA_UDC_EPS * 2 *
					 sizeof(*udc->descs),
					 &udc->descs_dma, GFP_KERNEL);
	if (!udc->descs)
		return -ENOMEM;

	udc->ep0_req = vita_udc_alloc_request(NULL, GFP_KERNEL);
	if (!udc->ep0_req)
		return -ENOMEM;

	udc->gadget.ops = &vita_udc_gadget_ops;
	udc->gadget.name = "vita-udc";
	udc->gadget.max_speed = USB_SPEED_HIGH;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.quirk_ep_out_aligned_size = true;
	vita_udc_init_eps(udc);

	ret = vita_udc_select_device_mode(udc);
	if (ret)
		goto err_req;
	vita_udc_quiesce(udc);

	ret = devm_request_irq(dev, irq, vita_udc_irq, 0, "vita-udc", udc);
	if (ret)
		goto err_req;
	ret = devm_request_irq(dev, dma_irq, vita_udc_dma_irq, 0,
			       "vita-udc-dma", udc);
	if (ret)
		goto err_req;
	ret = devm_request_threaded_irq(dev, vbus_irq, vita_udc_vbus_irq,
					vita_udc_vbus_thread, IRQF_ONESHOT,
					"vita-udc-vbus", udc);
	if (ret)
		goto err_req;
	if (udc->vbus_gpio) {
		int gpio_irq = gpiod_to_irq(udc->vbus_gpio);

		if (gpio_irq >= 0)
			ret = devm_request_threaded_irq(dev, gpio_irq, NULL,
							vita_udc_vbus_thread,
							IRQF_ONESHOT |
							IRQF_TRIGGER_RISING |
							IRQF_TRIGGER_FALLING,
							"vita-udc-vbus-gpio", udc);
		if (gpio_irq < 0 || ret)
			dev_warn(dev, "no VBUS GPIO interrupt\n");
	}

	platform_set_drvdata(pdev, udc);
	ret = usb_add_gadget_udc(dev, &udc->gadget);
	if (ret)
		goto err_req;

	/* The core assumes VBUS; vita_udc_start tells a bound driver otherwise. */
	udc->vbus = vita_udc_vbus_present(udc);
	dev_info(dev, "bus %u, VBUS %s\n", udc->bus, udc->vbus ? "on" : "off");
	return 0;

err_req:
	kfree(to_vita_req(udc->ep0_req));
	return ret;
}

static void vita_udc_remove(struct platform_device *pdev)
{
	struct vita_udc *udc = platform_get_drvdata(pdev);

	usb_del_gadget_udc(&udc->gadget);
	vita_udc_quiesce(udc);
	kfree(to_vita_req(udc->ep0_req));
}

static const struct of_device_id vita_udc_of_match[] = {
	{ .compatible = "vita,udc" },
	{ }
};
MODULE_DEVICE_TABLE(of, vita_udc_of_match);

static struct platform_driver vita_udc_driver = {
	.probe = vita_udc_probe,
	.remove = vita_udc_remove,
	.driver = {
		.name = "vita-udc",
		.of_match_table = vita_udc_of_match,
	},
};
module_platform_driver(vita_udc_driver);

MODULE_DESCRIPTION("PlayStation Vita USB device controller");
MODULE_LICENSE("GPL");
