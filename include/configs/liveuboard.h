/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Liveuboard.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#include "imx6_spl.h"

#define CONFIG_MACH_TYPE		MACH_TYPE_WANDBOARD_IMX6

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)

/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_SPEED		100000

/* MMC Configuration */
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

/* USB Configs */
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0

/* Ethernet Configuration */
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1

#define CONFIG_IMX6_PWM_PER_CLK		66000000
/* Framebuffer */
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
	"console=ttymxc0\0" \
	"splashpos=m,m\0" \
	"fdtfile=undefined\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_addr_r=0x18000000\0" \
	"fdt_addr=0x18000000\0" \

#define CONFIG_BOOTARGS    ""
#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0) \
	func(MMC, mmc, 1) \
	func(USB, usb, 0) \
	func(PXE, pxe, na) \
	func(DHCP, dhcp, na)

#include <config_distro_bootcmd.h>

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_SIZE			(8 * 1024)

#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0

#endif			       /* __CONFIG_H * */
