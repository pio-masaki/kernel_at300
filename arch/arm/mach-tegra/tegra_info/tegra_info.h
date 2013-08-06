/******************************************************************************
 * tegra_info.h - Linux kernel module for querying TEGRA information
 *
 * Copyright 2008-2011 Pegatron Corporation.
 *
 * DESCRIPTION:
 *	- This is the linux driver for querying TEGRA information
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 ******************************************************************************/


#ifndef __TEGRA_INFO_H__
#define __TEGRA_INFO_H__

#include "../fuse.h"

#define	DRIVER_NAME		    "tegra_info"
#define	DRIVER_VERSION		"0.1"


//###################################################################
// New method to query the CPU UID, revision, etc.
#define TEGRAINFO_IOC_MAGIC	123
#define TEGRAINFO_G_UID        _IOR(TEGRAINFO_IOC_MAGIC, 20, char[32])
#define TEGRAINFO_G_REVISION   _IOR(TEGRAINFO_IOC_MAGIC, 21, char[32])
#define TEGRAINFO_G_SKU        _IOR(TEGRAINFO_IOC_MAGIC, 22, int)

#endif

