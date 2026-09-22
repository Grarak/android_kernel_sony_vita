// SPDX-License-Identifier: GPL-2.0
/*
 * Sony PlayStation Vita (SCE) master block partition table
 *
 * Every Vita storage device (eMMC, memory card, game card) starts with a
 * 512-byte master block (wiki.henkaku.xyz Partitions):
 *   0x000  "Sony Computer Entertainment Inc." (0x20 bytes)
 *   0x020  version (3)
 *   0x024  device size in 512-byte blocks
 *   0x050  up to 16 partition entries of 17 bytes; offset 0 ends the table
 *   0x1FE  0xAA55
 * Entry: +0x0 offset (blocks), +0x4 size (blocks), +0x8 code, +0x9 type
 * (6 FAT16, 7 exFAT, 0xDA raw), +0xA active, +0xB flags, +0xF unknown.
 * The partition's name follows from its code (ux0 on the memory card,
 * os0/vs0/ur0 on the eMMC, ...) and is exported as PARTNAME, so Android's
 * ueventd creates /dev/block/.../by-name/<name> links.
 *
 * On the eMMC the block device is encrypted and the master block reads as
 * noise unless the SoC's cryptor is set up; the magic check then fails and
 * the next parser runs.
 */

#include <linux/unaligned.h>
#include "check.h"

#define SCE_MAGIC		"Sony Computer Entertainment Inc."
#define SCE_ENTRIES		0x50
#define SCE_ENTRY_SIZE		17
#define SCE_MAX_ENTRIES		16

static const char * const sce_part_names[] = {
	[0x00] = "empty",	[0x01] = "idstor",	[0x02] = "sloader",
	[0x03] = "os0",		[0x04] = "vs0",		[0x05] = "vd0",
	[0x06] = "tm0",		[0x07] = "ur0",		[0x08] = "ux0",
	[0x09] = "gro0",	[0x0A] = "grw0",	[0x0B] = "ud0",
	[0x0C] = "sa0",		[0x0E] = "pd0",
};

int sce_partition(struct parsed_partitions *state)
{
	unsigned char *data;
	Sector sect;
	int i, slot = 1;

	data = read_part_sector(state, 0, &sect);
	if (!data)
		return -1;

	if (memcmp(data, SCE_MAGIC, strlen(SCE_MAGIC)) ||
	    get_unaligned_le16(data + 0x1FE) != 0xAA55) {
		put_dev_sector(sect);
		return 0;
	}

	for (i = 0; i < SCE_MAX_ENTRIES && slot < state->limit; i++) {
		const unsigned char *e = data + SCE_ENTRIES + i * SCE_ENTRY_SIZE;
		u32 offset = get_unaligned_le32(e);
		u32 size = get_unaligned_le32(e + 4);
		u8 code = e[8];
		struct partition_meta_info *info;
		char name[PARTITION_META_INFO_VOLNAMELTH];

		if (!offset)
			break;
		if (!size)
			continue;

		if (code < ARRAY_SIZE(sce_part_names) && sce_part_names[code])
			snprintf(name, sizeof(name), "%s%s", sce_part_names[code],
				 e[0xA] ? "" : (code == 0x02 || code == 0x03) ?
				 "_inactive" : "");
		else
			snprintf(name, sizeof(name), "sce%02x", code);

		put_partition(state, slot, offset, size);
		info = &state->parts[slot].info;
		strscpy(info->volname, name, sizeof(info->volname));
		state->parts[slot].has_info = true;
		slot++;
	}
	put_dev_sector(sect);

	seq_buf_puts(&state->pp_buf, "\n");
	return 1;
}
