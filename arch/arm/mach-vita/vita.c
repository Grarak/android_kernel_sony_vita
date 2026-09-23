/*
 * Device Tree support for PlayStation Vita
 *
 * Copyright (C) 2018 Sergi Granell
 *
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/init.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "vita.h"

static const char * const vita_dt_compat[] = {
	"sony,vita",
	"vita,vita",
	NULL,
};

static void __init vita_map_io(void)
{
	debug_ll_io_init();
	vita_earlyfb_map_io();
}

DT_MACHINE_START(VITA_DT, "PlayStation Vita (Device Tree Support)")
	.dt_compat	= vita_dt_compat,
	.l2c_aux_val	= 0,
	.l2c_aux_mask	= ~0,
	.map_io		= vita_map_io,
	.init_early	= vita_earlyfb_init,
MACHINE_END
