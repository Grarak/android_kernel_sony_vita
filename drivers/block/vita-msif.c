// SPDX-License-Identifier: GPL-2.0
/*
 * PlayStation Vita Memory Card (SceMsif) block driver
 *
 * The Vita memory card speaks the Memory Stick PRO TPC protocol plus an
 * authentication layer (card commands 0x48-0x4B, see wiki.henkaku.xyz
 * Memory_Card). Authentication needs keys and the F00D rmauth service, so
 * it is left to the boot loader, which authenticates the card (as
 * vita-libbaremetal's msif_setup() does) and hands over with the card
 * powered, in parallel-4 mode and unlocked. This driver never resets the
 * controller or the card.
 *
 * Data moves by PIO through the TPC FIFO, one 512-byte sector per card
 * command, following libbaremetal's msif_read_sector(), the sequence the
 * bare-metal Linux loader uses on hardware. Writes mirror it with
 * MSPRO_CMD_WRITE_DATA / WRITE_LONG_DATA. That is the standard Memory Stick
 * PRO flow, but it has not been checked on hardware, so writes are off
 * unless vita_msif.writable=1.
 *
 * Register offsets: libbaremetal src/msif.c.
 *
 * Without child nodes the whole card is one disk, msblk0, whose SCE master
 * block the "sce" partition parser reads. With child nodes, which the
 * Android boot chain adds, Linux sees only the files they describe:
 *
 *	disk@0 {
 *		label = "emmc";
 *		vita,extents = <start count>, ...;
 *	};
 *
 * Each child becomes the disk vita<label>, the concatenation of its
 * extents (card sectors: first sector and sector count). The loader
 * resolved them from the files' exFAT allocation while the Vita OS still
 * ran, and no sector outside them is ever addressed.
 */

#include <linux/blk-mq.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/unaligned.h>

#define MSIF_COMMAND_REG	0x30
#define MSIF_DATA_REG		0x34
#define MSIF_STATUS_REG		0x38
#define MSIF_SYSTEM_REG		0x3C

#define MSIF_STATUS_TIMEOUT	0x0100
#define MSIF_STATUS_CRC_ERROR	0x0200
#define MSIF_STATUS_READY	0x1000
#define MSIF_STATUS_UNK1	0x2000
#define MSIF_STATUS_FIFO_RW	0x4000

#define MS_TPC_READ_LONG_DATA	0x2000
#define MS_TPC_READ_REG		0x4000
#define MS_TPC_GET_INT		0x7000
#define MS_TPC_SET_RW_REG_ADRS	0x8000
#define MS_TPC_EX_SET_CMD	0x9000
#define MS_TPC_WRITE_LONG_DATA	0xD000
#define MS_TPC_SET_CMD		0xE000

#define MS_INT_REG_CMDNK	0x01
#define MS_INT_REG_BREQ		0x20
#define MS_INT_REG_ERR		0x40
#define MS_INT_REG_CED		0x80

#define MSPRO_CMD_READ_DATA	0x20
#define MSPRO_CMD_WRITE_DATA	0x21
#define MSPRO_CMD_READ_ATRB	0x24
#define MSPRO_CMD_STOP		0x25

#define MS_STATUS_REG_ADDR	0x02
#define MS_STATUS_WP		0x01

#define MS_SECTOR_SIZE		512
#define MSIF_TIMEOUT_US		(1000 * 1000)

/* SCE master block (wiki.henkaku.xyz Partitions) */
#define SCE_MBR_MAGIC		"Sony Computer Entertainment Inc."
#define SCE_MBR_DEVICE_SIZE	0x24

static bool writable;
module_param(writable, bool, 0444);
MODULE_PARM_DESC(writable, "Allow writes to the memory card (untested on hardware)");

struct vita_msif_extent {
	u32 start;
	u32 count;
};

struct vita_msif_disk {
	struct vita_msif *ms;
	struct blk_mq_tag_set tag_set;
	struct gendisk *disk;
	/* NULL for the whole card */
	struct vita_msif_extent *extents;
	unsigned int nr_extents;
};

struct vita_msif {
	struct device *dev;
	void __iomem *regs;
	struct mutex lock;	/* serialises card commands */
	struct vita_msif_disk *disks;
	unsigned int nr_disks;
	bool wp;
};

static inline void ms_tpc(struct vita_msif *ms, u32 tpc, u32 size)
{
	writew(tpc | (size & 0x7FF), ms->regs + MSIF_COMMAND_REG);
}

static int ms_wait_status(struct vita_msif *ms, u16 bits, u16 errbits)
{
	u16 val;
	int ret;

	ret = readw_poll_timeout(ms->regs + MSIF_STATUS_REG, val,
				 val & (bits | errbits), 1, MSIF_TIMEOUT_US);
	if (ret)
		return ret;

	return (val & errbits) ? -EIO : 0;
}

static int ms_wait_ready(struct vita_msif *ms)
{
	return ms_wait_status(ms, MSIF_STATUS_READY,
			      MSIF_STATUS_TIMEOUT | MSIF_STATUS_CRC_ERROR);
}

static int ms_fifo_read(struct vita_msif *ms, void *buf, u32 size)
{
	u32 data[2];
	u8 *p = buf;
	int ret;

	while (size) {
		u32 n = min_t(u32, size, sizeof(data));

		ret = ms_wait_status(ms, MSIF_STATUS_FIFO_RW, 0);
		if (ret)
			return ret;

		data[0] = readl(ms->regs + MSIF_DATA_REG);
		data[1] = readl(ms->regs + MSIF_DATA_REG);
		memcpy(p, data, n);
		p += n;
		size -= n;
	}

	return 0;
}

static int ms_fifo_write(struct vita_msif *ms, const void *buf, u32 size)
{
	const u8 *p = buf;
	u32 data[2];
	int ret;

	while (size) {
		u32 n = min_t(u32, size, sizeof(data));

		memset(data, 0, sizeof(data));
		memcpy(data, p, n);

		ret = ms_wait_status(ms, MSIF_STATUS_FIFO_RW, 0);
		if (ret)
			return ret;

		writel(data[0], ms->regs + MSIF_DATA_REG);
		writel(data[1], ms->regs + MSIF_DATA_REG);
		p += n;
		size -= n;
	}

	return 0;
}

static int ms_get_int(struct vita_msif *ms, u8 *reg_int)
{
	int ret;

	ms_tpc(ms, MS_TPC_GET_INT, 1);
	ret = ms_fifo_read(ms, reg_int, 1);
	if (ret)
		return ret;

	return ms_wait_ready(ms);
}

/* Poll the card's INT register until @want is set; ERR/CMDNK abort. */
static int ms_wait_int(struct vita_msif *ms, u8 want)
{
	ktime_t timeout = ktime_add_us(ktime_get(), MSIF_TIMEOUT_US);
	u8 reg_int;
	int ret;

	for (;;) {
		ret = ms_get_int(ms, &reg_int);
		if (!ret) {
			if (reg_int & (MS_INT_REG_ERR | MS_INT_REG_CMDNK))
				return -EIO;
			if (reg_int & want)
				return 0;
		}
		if (ktime_after(ktime_get(), timeout))
			return -ETIMEDOUT;
		cpu_relax();
	}
}

static int ms_read_reg(struct vita_msif *ms, u8 addr, void *buf, u8 size)
{
	u8 adrs[4] = { addr, size, 0, 0 };
	int ret;

	ms_tpc(ms, MS_TPC_SET_RW_REG_ADRS, sizeof(adrs));
	ret = ms_fifo_write(ms, adrs, sizeof(adrs));
	if (!ret)
		ret = ms_wait_ready(ms);
	if (ret)
		return ret;

	ms_tpc(ms, MS_TPC_READ_REG, size);
	ret = ms_fifo_read(ms, buf, size);
	if (ret)
		return ret;

	return ms_wait_ready(ms);
}

static void ms_stop(struct vita_msif *ms)
{
	u8 cmd = MSPRO_CMD_STOP;

	ms_tpc(ms, MS_TPC_SET_CMD, 1);
	if (!ms_fifo_write(ms, &cmd, 1) && !ms_wait_ready(ms))
		ms_wait_int(ms, MS_INT_REG_CED);
}

/* One sector: the libbaremetal msif_read_sector() sequence, bounded. */
static int ms_xfer_sector(struct vita_msif *ms, u8 cmd, u32 sector,
			  void *buf, bool write)
{
	const u32 count = 1;
	u8 data[7] = {
		cmd, (count >> 8) & 0xFF, count & 0xFF,
		(sector >> 24) & 0xFF, (sector >> 16) & 0xFF,
		(sector >> 8) & 0xFF, sector & 0xFF,
	};
	int ret;

	ms_tpc(ms, MS_TPC_EX_SET_CMD, sizeof(data));
	ret = ms_fifo_write(ms, data, sizeof(data));
	if (!ret)
		ret = ms_wait_ready(ms);
	if (!ret)
		ret = ms_wait_status(ms, MSIF_STATUS_UNK1, 0);
	if (!ret)
		ret = ms_wait_int(ms, MS_INT_REG_BREQ);
	if (ret)
		goto err;

	if (write) {
		ms_tpc(ms, MS_TPC_WRITE_LONG_DATA, MS_SECTOR_SIZE);
		ret = ms_fifo_write(ms, buf, MS_SECTOR_SIZE);
	} else {
		ms_tpc(ms, MS_TPC_READ_LONG_DATA, MS_SECTOR_SIZE);
		ret = ms_fifo_read(ms, buf, MS_SECTOR_SIZE);
	}
	if (!ret)
		ret = ms_wait_ready(ms);
	if (!ret)
		ret = ms_wait_status(ms, MSIF_STATUS_UNK1, 0);
	if (!ret)
		ret = ms_wait_int(ms, MS_INT_REG_CED);
	if (!ret)
		return 0;
err:
	dev_err_ratelimited(ms->dev, "%s of sector %u failed: %d\n",
			    write ? "write" : "read", sector, ret);
	ms_stop(ms);
	return ret;
}

/* The card sector behind @sector of @md, or U32_MAX past its end. */
static u32 vita_msif_map(struct vita_msif_disk *md, sector_t sector)
{
	unsigned int i;

	if (!md->extents)
		return sector < U32_MAX ? sector : U32_MAX;
	for (i = 0; i < md->nr_extents; i++) {
		if (sector < md->extents[i].count)
			return md->extents[i].start + sector;
		sector -= md->extents[i].count;
	}
	return U32_MAX;
}

static blk_status_t vita_msif_queue_rq(struct blk_mq_hw_ctx *hctx,
				       const struct blk_mq_queue_data *bd)
{
	struct vita_msif_disk *md = hctx->queue->queuedata;
	struct vita_msif *ms = md->ms;
	struct request *rq = bd->rq;
	bool write = rq_data_dir(rq) == WRITE;
	sector_t sector = blk_rq_pos(rq);
	struct req_iterator iter;
	struct bio_vec bvec;
	blk_status_t status = BLK_STS_OK;

	blk_mq_start_request(rq);

	switch (req_op(rq)) {
	case REQ_OP_READ:
		break;
	case REQ_OP_WRITE:
		if (ms->wp || !writable) {
			status = BLK_STS_PROTECTION;
			goto out;
		}
		break;
	case REQ_OP_FLUSH:
		/* Every sector command completes (CED) before we return */
		goto out;
	default:
		status = BLK_STS_NOTSUPP;
		goto out;
	}

	mutex_lock(&ms->lock);
	rq_for_each_segment(bvec, rq, iter) {
		u8 *buf = bvec_kmap_local(&bvec);
		unsigned int off;

		for (off = 0; off < bvec.bv_len; off += MS_SECTOR_SIZE) {
			u32 card_sector = vita_msif_map(md, sector++);

			if (card_sector == U32_MAX ||
			    ms_xfer_sector(ms, write ? MSPRO_CMD_WRITE_DATA :
					   MSPRO_CMD_READ_DATA,
					   card_sector, buf + off, write)) {
				status = BLK_STS_IOERR;
				break;
			}
		}
		kunmap_local(buf);
		if (status)
			break;
	}
	mutex_unlock(&ms->lock);
out:
	blk_mq_end_request(rq, status);
	return BLK_STS_OK;
}

static const struct blk_mq_ops vita_msif_mq_ops = {
	.queue_rq = vita_msif_queue_rq,
};

static const struct block_device_operations vita_msif_fops = {
	.owner = THIS_MODULE,
};

/*
 * Card size: the SCE master block at sector 0 records it (+0x24, in blocks).
 * Otherwise take the end of the last DOS partition, and failing that the
 * Memory Stick PRO attribute area.
 */
static sector_t vita_msif_capacity(struct vita_msif *ms, u8 *buf)
{
	sector_t end = 0;
	int i;

	if (ms_xfer_sector(ms, MSPRO_CMD_READ_DATA, 0, buf, false))
		return 0;

	if (!memcmp(buf, SCE_MBR_MAGIC, strlen(SCE_MBR_MAGIC)))
		return get_unaligned_le32(buf + SCE_MBR_DEVICE_SIZE);

	if (get_unaligned_le16(buf + 0x1FE) == 0xAA55) {
		for (i = 0; i < 4; i++) {
			const u8 *p = buf + 0x1BE + 16 * i;
			sector_t e = (sector_t)get_unaligned_le32(p + 8) +
				     get_unaligned_le32(p + 12);

			end = max(end, e);
		}
		if (end)
			return end;
	}

	/*
	 * Attribute area: device information entries at +0x10, 12 bytes each.
	 * ID 0x10 is the MS PRO system information; for the Vita's class-3
	 * card (ID 0x13/0x14) SceMsif reads the unit count at +6 and sectors
	 * per unit at +0x20.
	 */
	if (ms_xfer_sector(ms, MSPRO_CMD_READ_ATRB, 0, buf, false))
		return 0;
	if (get_unaligned_be16(buf) != 0xa5c3)
		return 0;

	for (i = 0; i < buf[4] && i < 12; i++) {
		const u8 *e = buf + 0x10 + 12 * i;
		u32 addr = get_unaligned_be32(e);
		u8 id = e[8];
		const u8 *info;

		if (addr + 0x30 > MS_SECTOR_SIZE)
			continue;
		info = buf + addr;
		if (id == 0x13 || id == 0x14)
			return (sector_t)get_unaligned_be32(info + 6) *
			       get_unaligned_be32(info + 0x20);
		if (id == 0x10)
			return (sector_t)get_unaligned_be16(info + 6) *
			       get_unaligned_be16(info + 2) *
			       (get_unaligned_be16(info + 8) >> 9);
	}

	return 0;
}

static void vita_msif_del_disk(struct vita_msif_disk *md)
{
	del_gendisk(md->disk);
	put_disk(md->disk);
	blk_mq_free_tag_set(&md->tag_set);
}

/*
 * A registered major with contiguous minors: a disk without one gets its
 * dev_t from the extended range, where the kernel refuses minors, and
 * Android's vold numbers partitions from the disk's own minor.
 */
#define VITA_MSIF_MINORS	16
static int vita_msif_major;

static int vita_msif_add_disk(struct vita_msif *ms, struct vita_msif_disk *md,
			      const char *name, sector_t capacity)
{
	struct queue_limits lim = {
		.logical_block_size	= MS_SECTOR_SIZE,
		.max_hw_sectors		= 128,
	};
	int ret;

	md->ms = ms;
	md->tag_set.ops = &vita_msif_mq_ops;
	md->tag_set.nr_hw_queues = 1;
	md->tag_set.queue_depth = 16;
	md->tag_set.numa_node = NUMA_NO_NODE;
	md->tag_set.flags = BLK_MQ_F_BLOCKING;
	ret = blk_mq_alloc_tag_set(&md->tag_set);
	if (ret)
		return ret;

	md->disk = blk_mq_alloc_disk(&md->tag_set, &lim, md);
	if (IS_ERR(md->disk)) {
		ret = PTR_ERR(md->disk);
		goto err_tag_set;
	}

	md->disk->fops = &vita_msif_fops;
	md->disk->private_data = md;
	md->disk->major = vita_msif_major;
	md->disk->first_minor = (md - ms->disks) * VITA_MSIF_MINORS;
	md->disk->minors = VITA_MSIF_MINORS;
	strscpy(md->disk->disk_name, name, sizeof(md->disk->disk_name));
	set_capacity(md->disk, capacity);
	if (ms->wp || !writable)
		set_disk_ro(md->disk, true);

	/*
	 * Under the platform device, so Android's init makes
	 * /dev/block/platform/<msif>/by-name/ links for the first-stage mount.
	 */
	ret = device_add_disk(ms->dev, md->disk, NULL);
	if (ret)
		goto err_disk;

	dev_info(ms->dev, "%s: %llu sectors (%llu MiB) in %u extent(s)%s\n",
		 name, (unsigned long long)capacity,
		 (unsigned long long)capacity >> 11, md->nr_extents ?: 1,
		 ms->wp ? ", write-protected" : (writable ? "" : ", read-only"));
	return 0;

err_disk:
	put_disk(md->disk);
err_tag_set:
	blk_mq_free_tag_set(&md->tag_set);
	return ret;
}

/* One disk per child node with vita,extents; msblk0 for the card without any. */
static int vita_msif_add_disks(struct vita_msif *ms, sector_t card_sectors)
{
	struct device_node *child;
	unsigned int n = 0, i;
	int ret;

	ms->nr_disks = of_get_available_child_count(ms->dev->of_node);
	ms->disks = devm_kcalloc(ms->dev, max(ms->nr_disks, 1u),
				 sizeof(*ms->disks), GFP_KERNEL);
	if (!ms->disks)
		return -ENOMEM;
	if (!ms->nr_disks) {
		ms->nr_disks = 1;
		return vita_msif_add_disk(ms, &ms->disks[0], "msblk0", card_sectors);
	}

	for_each_available_child_of_node(ms->dev->of_node, child) {
		struct vita_msif_disk *md = &ms->disks[n];
		char name[DISK_NAME_LEN];
		const char *label;
		sector_t capacity = 0;
		int count;

		count = of_property_count_u32_elems(child, "vita,extents");
		if (of_property_read_string(child, "label", &label) || count <= 0 ||
		    count % 2) {
			ret = dev_err_probe(ms->dev, -EINVAL, "%pOF: needs label and vita,extents\n",
					    child);
			goto err;
		}
		md->nr_extents = count / 2;
		md->extents = devm_kcalloc(ms->dev, md->nr_extents,
					   sizeof(*md->extents), GFP_KERNEL);
		if (!md->extents) {
			ret = -ENOMEM;
			goto err;
		}
		of_property_read_u32_array(child, "vita,extents", (u32 *)md->extents,
					   count);
		for (i = 0; i < md->nr_extents; i++) {
			const struct vita_msif_extent *e = &md->extents[i];

			if (!e->count || e->start >= card_sectors ||
			    e->count > card_sectors - e->start) {
				ret = dev_err_probe(ms->dev, -EINVAL,
						    "%s: extent %u+%u is off the card\n",
						    label, e->start, e->count);
				goto err;
			}
			capacity += e->count;
		}

		snprintf(name, sizeof(name), "vita%s", label);
		ret = vita_msif_add_disk(ms, md, name, capacity);
		if (ret)
			goto err;
		n++;
	}
	return 0;

err:
	of_node_put(child);
	while (n--)
		vita_msif_del_disk(&ms->disks[n]);
	return ret;
}

static int vita_msif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vita_msif *ms;
	sector_t capacity;
	u8 status[1];
	u8 *buf;
	int ret;

	ms = devm_kzalloc(dev, sizeof(*ms), GFP_KERNEL);
	if (!ms)
		return -ENOMEM;

	ms->dev = dev;
	mutex_init(&ms->lock);

	ms->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ms->regs))
		return PTR_ERR(ms->regs);

	buf = devm_kmalloc(dev, MS_SECTOR_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&ms->lock);
	ret = ms_read_reg(ms, MS_STATUS_REG_ADDR, status, sizeof(status));
	if (!ret)
		ms->wp = status[0] & MS_STATUS_WP;
	capacity = vita_msif_capacity(ms, buf);
	mutex_unlock(&ms->lock);

	if (!capacity)
		return dev_err_probe(dev, -ENODEV,
				     "no authenticated memory card (did the loader run msif_setup?)\n");

	ret = vita_msif_add_disks(ms, capacity);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, ms);
	return 0;
}

static void vita_msif_remove(struct platform_device *pdev)
{
	struct vita_msif *ms = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < ms->nr_disks; i++)
		vita_msif_del_disk(&ms->disks[i]);
}

static const struct of_device_id vita_msif_of_match[] = {
	{ .compatible = "vita,msif" },
	{ }
};
MODULE_DEVICE_TABLE(of, vita_msif_of_match);

static struct platform_driver vita_msif_driver = {
	.driver = {
		.name = "vita-msif",
		.of_match_table = vita_msif_of_match,
	},
	.probe = vita_msif_probe,
	.remove = vita_msif_remove,
};

static int __init vita_msif_init(void)
{
	int ret;

	vita_msif_major = register_blkdev(0, "vitamsif");
	if (vita_msif_major < 0)
		return vita_msif_major;
	ret = platform_driver_register(&vita_msif_driver);
	if (ret)
		unregister_blkdev(vita_msif_major, "vitamsif");
	return ret;
}
module_init(vita_msif_init);

static void __exit vita_msif_exit(void)
{
	platform_driver_unregister(&vita_msif_driver);
	unregister_blkdev(vita_msif_major, "vitamsif");
}
module_exit(vita_msif_exit);

MODULE_DESCRIPTION("PlayStation Vita Memory Card (SceMsif) block driver");
MODULE_LICENSE("GPL");
