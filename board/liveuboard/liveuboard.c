// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2014 O.S. Systems Software LTDA.
 * Copyright (C) 2018 LiveU
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 * Author: Jemish Patel <jemish@liveu.tv>
 * This is file is created based "board/wandboard/wandboard.c"
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <asm/mach-imx/sata.h>
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <phy.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <mv88e6176.h>
#if defined(CONFIG_PWMCNTL_BACKLIGHT)
#include <pwm.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm	| PAD_CTL_SRE_FAST)

#define USDHC3_CD_GPIO		IMX_GPIO_NR(3, 9)
#define ETH_PHY_RESET		IMX_GPIO_NR(3, 29)
#define REV_DETECTION		IMX_GPIO_NR(2, 28)

#define I2C_BUS_0		0
#define SET_I2C_BUS(x)		I2C_BUS_##x
/* Enabled Marvell switch MV88E6176 u-boot post command and
   SMI device address generated using ADDR[4:1] pull up pin
   configuration of marvell switch
*/
#define MV88E6176_ADDRESS	0x1E

static bool with_pmic;

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK    | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD    | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	/* SOM MicroSD Card Detect */
	IOMUX_PADS(PAD_EIM_DA9__GPIO3_IO09  | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

static iomux_v3_cfg_t const enet_pads[] = {
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC    | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	/* AR8031 PHY Reset */
	IOMUX_PADS(PAD_EIM_D29__GPIO3_IO29    | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

/* Marvell MV88E6176 switch configuration */
static struct mv88e6176_sw_reg switch_conf[] = {
	/* port 1, FRONT_MDI, autoneg */
	{ PORT(0), PORT_PHY, FULL_DPX | FULL_DPX_FOR | NO_SPEED_FOR },
	{ PORT(1), PORT_PHY, FULL_DPX | FULL_DPX_FOR | SPEED_100_FOR },
	{ PORT(2), PORT_PHY, FULL_DPX | FULL_DPX_FOR | SPEED_100_FOR },
	{ PORT(5), PORT_PHY, FULL_DPX | FULL_DPX_FOR | SPEED_100_FOR | LINK_FOR | LINK_VAL },
	{ PORT(6), PORT_PHY, FULL_DPX | FULL_DPX_FOR | SPEED_100_FOR | LINK_FOR | LINK_VAL | TX_RGMII_TIM | RX_RGMII_TIM },
	{ PORT(0), PORT_CTRL, FORWARDING | EGRS_FLD_ALL },
	{ PORT(1), PORT_CTRL, FORWARDING | EGRS_FLD_ALL },
	{ PORT(2), PORT_CTRL, FORWARDING | EGRS_FLD_ALL },
	{ PORT(5), PORT_CTRL, FORWARDING | EGRS_FLD_ALL },
	{ PORT(6), PORT_CTRL, FORWARDING | EGRS_FLD_ALL },
	{ PHY(0), PHY_1000_CTRL, NO_ADV },
	{ PHY(1), PHY_1000_CTRL, NO_ADV },
	{ PHY(2), PHY_1000_CTRL, NO_ADV },
	{ PHY(0), PHY_CTRL, PHY_1_GBPS | AUTONEG_EN | FULL_DUPLEX },
	{ PHY(1), PHY_CTRL, PHY_100_MBPS | AUTONEG_EN | FULL_DUPLEX | PHY_RESET | 0x200 },
	{ PHY(2), PHY_CTRL, PHY_100_MBPS | AUTONEG_EN | FULL_DUPLEX | PHY_RESET | 0x200 },
	{ PHY(0), PHY_1000_CTRL, 0x0E00 },
	{ GLOBAL2, SCRATCH_MISC, (GP_PIN_CTRL3 << 8) | CLK_125MHZ | DATA_UPDATE }
};

static iomux_v3_cfg_t const rev_detection_pad[] = {
	IOMUX_PADS(PAD_EIM_EB0__GPIO2_IO28  | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
}

static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads);

	/* Reset AR8031 PHY */
	gpio_direction_output(ETH_PHY_RESET, 0);
	mdelay(10);
	gpio_set_value(ETH_PHY_RESET, 1);
	udelay(100);
}

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		/* LiveU: Added to detect eMMC as like SD card.
		   We don't have GPIO for card detection but mmc drivers in u-boot
		   needs it to acess all the read/write APIs
		*/
		ret = 1;
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	u32 index = 0;

	/*
	 * Following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    SD3
	 * mmc1                    SD4(eMMC)
	 */
	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
			SETUP_IOMUX_PADS(usdhc3_pads);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			usdhc_cfg[0].max_bus_width = 4;
			gpio_direction_input(USDHC3_CD_GPIO);
			break;
		case 1:
			SETUP_IOMUX_PADS(usdhc4_pads);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			usdhc_cfg[1].max_bus_width = 8;
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       index + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		if (ret)
			return ret;
	}

	return 0;
}

#ifdef CONFIG_MXC_SPI
iomux_v3_cfg_t const ecspi1_pads[] = {
	/* SS0/CS0 */
	IOMUX_PADS(PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D24__GPIO3_IO24  | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

iomux_v3_cfg_t const ecspi2_pads[] = {
	IOMUX_PADS(PAD_EIM_CS0__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_OE__ECSPI2_MISO  | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_CS1__ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_RW__GPIO2_IO26   | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D26__GPIO3_IO26  | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

void setup_spi(void)
{
	SETUP_IOMUX_PADS(ecspi1_pads);
	SETUP_IOMUX_PADS(ecspi2_pads);
}
#endif

int board_phy_config(struct phy_device *phydev)
{
	return 0;
}


static struct i2c_pads_info mx6q_i2c1_pad_info = {
	.scl = {
		.i2c_mode = MX6Q_PAD_EIM_D21__I2C1_SCL
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_EIM_D21__GPIO3_IO21
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_EIM_D28__I2C1_SDA
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_EIM_D28__GPIO3_IO28
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 28)
	}
};

struct i2c_pads_info mx6q_i2c2_pad_info = {
	.scl = {
		.i2c_mode = MX6Q_PAD_KEY_COL3__I2C2_SCL
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_KEY_COL3__GPIO4_IO12
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_KEY_ROW3__I2C2_SDA
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_KEY_ROW3__GPIO4_IO13
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

struct i2c_pads_info mx6q_i2c3_pad_info = {
	.scl = {
		.i2c_mode = MX6Q_PAD_GPIO_5__I2C3_SCL
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_GPIO_5__GPIO1_IO05
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_GPIO_16__I2C3_SDA
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_GPIO_16__GPIO7_IO11
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(7, 11)
	}
};


#if defined(CONFIG_VIDEO_IPUV3)
static iomux_v3_cfg_t const lcd_display_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK),
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02), /* HSync */
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03), /* VSync */
	IOMUX_PADS(PAD_DI0_PIN4__IPU1_DI0_PIN04	| MUX_PAD_CTRL(PAD_CTL_DSE_120ohm)), /* Contrast */
	IOMUX_PADS(PAD_DI0_PIN15__IPU1_DI0_PIN15), /* DISP0_DRDY */
	IOMUX_PADS(PAD_DISP0_DAT0__IPU1_DISP0_DATA00),
	IOMUX_PADS(PAD_DISP0_DAT1__IPU1_DISP0_DATA01),
	IOMUX_PADS(PAD_DISP0_DAT2__IPU1_DISP0_DATA02),
	IOMUX_PADS(PAD_DISP0_DAT3__IPU1_DISP0_DATA03),
	IOMUX_PADS(PAD_DISP0_DAT4__IPU1_DISP0_DATA04),
	IOMUX_PADS(PAD_DISP0_DAT5__IPU1_DISP0_DATA05),
	IOMUX_PADS(PAD_DISP0_DAT6__IPU1_DISP0_DATA06),
	IOMUX_PADS(PAD_DISP0_DAT7__IPU1_DISP0_DATA07),
	IOMUX_PADS(PAD_DISP0_DAT8__IPU1_DISP0_DATA08),
	IOMUX_PADS(PAD_DISP0_DAT9__IPU1_DISP0_DATA09),
	IOMUX_PADS(PAD_DISP0_DAT10__IPU1_DISP0_DATA10),
	IOMUX_PADS(PAD_DISP0_DAT11__IPU1_DISP0_DATA11),
	IOMUX_PADS(PAD_DISP0_DAT12__IPU1_DISP0_DATA12),
	IOMUX_PADS(PAD_DISP0_DAT13__IPU1_DISP0_DATA13),
	IOMUX_PADS(PAD_DISP0_DAT14__IPU1_DISP0_DATA14),
	IOMUX_PADS(PAD_DISP0_DAT15__IPU1_DISP0_DATA15),
	IOMUX_PADS(PAD_DISP0_DAT16__IPU1_DISP0_DATA16),
	IOMUX_PADS(PAD_DISP0_DAT17__IPU1_DISP0_DATA17),
	IOMUX_PADS(PAD_DISP0_DAT18__IPU1_DISP0_DATA18),
	IOMUX_PADS(PAD_DISP0_DAT19__IPU1_DISP0_DATA19),
	IOMUX_PADS(PAD_DISP0_DAT20__IPU1_DISP0_DATA20),
	IOMUX_PADS(PAD_DISP0_DAT21__IPU1_DISP0_DATA21),
	IOMUX_PADS(PAD_DISP0_DAT22__IPU1_DISP0_DATA22),
	IOMUX_PADS(PAD_DISP0_DAT23__IPU1_DISP0_DATA23),
	IOMUX_PADS(PAD_SD1_CMD__GPIO1_IO18 | MUX_PAD_CTRL(NO_PAD_CTRL)),
#if defined(CONFIG_PWMCNTL_BACKLIGHT)
	IOMUX_PADS(PAD_SD1_DAT1__PWM3_OUT | MUX_PAD_CTRL(NO_PAD_CTRL)),   /* LCDCNTL */
#else
	IOMUX_PADS(PAD_SD1_DAT1__GPIO1_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL)), /* LCDCNTL */
#endif
};

static void do_enable_hdmi(struct display_info_t const *dev)
{
	imx_enable_hdmi_phy();
}

static int detect_i2c(struct display_info_t const *dev)
{
	/* Always return as detected to make it simple and
	   to keep the existing logic as modular for future purpose */
	return 1;
}

static void enable_tianma_5wvga(struct display_info_t const *dev)
{
	SETUP_IOMUX_PADS(lcd_display_pads);
#if defined(CONFIG_PWMCNTL_BACKLIGHT)
	gpio_direction_output(IMX_GPIO_NR(1, 18), 1);
#else
	/* Backlight : GPIO1_IO18 & GPIO1_IO18 */
	gpio_direction_output(IMX_GPIO_NR(1, 18), 1);
	gpio_direction_output(IMX_GPIO_NR(1, 17), 1);
#endif

}

struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	/* I2C bus number (I2C2) at which CTP is connected */
	.bus	= 1,
	/* I2C slave address of CTP, it could be 0x48/0x49/0x4a/0x4b */
	.addr	= 0x48,
	/* LCD display pixel format RGB888 24-bit */
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_i2c,
	.enable	= enable_tianma_5wvga,
	/* Timings inserted based on
		1) Table 5.2.2 on Page# 10 of EVK module,
		2) Kernel file "Documentation/fb/framebuffer.txt" and
		3) From https://community.freescale.com/docs/DOC-93617
	*/
	.mode	= {
		.name           = "TM050RVHG01-00",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 33333,
		.left_margin    = 48,
		.right_margin   = 40,
		.upper_margin   = 32,
		.lower_margin   = 13,
		.hsync_len      = 40,
		.vsync_len      = 3,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	imx_setup_hdmi();

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	/* Disable LCD backlight */
	SETUP_IOMUX_PAD(PAD_DI0_PIN4__GPIO4_IO20);
	gpio_direction_input(IMX_GPIO_NR(4, 20));
}
#endif /* CONFIG_VIDEO_IPUV3 */

int last_stage_init(void)
{
	/* configure MV88E6176 switch */
	char *name = "FEC";

	if (miiphy_set_current_dev(name))
		return 0;

	mv88e6176_sw_program(name, MV88E6176_ADDRESS, switch_conf,
	ARRAY_SIZE(switch_conf));

	mv88e6176_sw_reset(name, MV88E6176_ADDRESS);
	return 0;
}


int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();

	return cpu_eth_init(bis);
}

int board_early_init_f(void)
{
	setup_iomux_uart();
#ifdef CONFIG_SATA
	setup_sata();
#endif

	return 0;
}

#define PMIC_I2C_BUS		2

int power_init_board(void)
{
	struct pmic *p;
	u32 reg;

	/* configure PFUZE100 PMIC */
	power_pfuze100_init(PMIC_I2C_BUS);
	p = pmic_get("PFUZE100");
	if (p && !pmic_probe(p)) {
		pmic_reg_read(p, PFUZE100_DEVICEID, &reg);
		printf("PMIC:  PFUZE100 ID=0x%02x\n", reg);
		with_pmic = true;

		/* Set VGEN2 to 1.5V and enable */
		pmic_reg_read(p, PFUZE100_VGEN2VOL, &reg);
		reg &= ~(LDO_VOL_MASK);
		reg |= (LDOA_1_50V | (1 << (LDO_EN)));
		pmic_reg_write(p, PFUZE100_VGEN2VOL, reg);
	}

	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	  MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	  MAKE_CFGVAL(0x40, 0x20, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

static bool is_revc1(void)
{
	SETUP_IOMUX_PADS(rev_detection_pad);
	gpio_direction_input(REV_DETECTION);

	if (gpio_get_value(REV_DETECTION))
		return true;
	else
		return false;
}

static bool is_revd1(void)
{
	if (with_pmic)
		return true;
	else
		return false;
}

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	if (is_mx6dqp())
		env_set("board_rev", "MX6QP");
	else if (is_mx6dq())
		env_set("board_rev", "MX6Q");
	else
		env_set("board_rev", "MX6DL");

	if (is_revd1())
		env_set("board_name", "D1");
	else if (is_revc1())
		env_set("board_name", "C1");
	else
		env_set("board_name", "B1");
#endif
	return 0;
}

#if defined(CONFIG_CONFIGURE_FAN)
#define FAN_CONTROLLER_SLAVE_ADDR_18		(0x18)
int configure_fan(uchar i2c_bus_number, uchar chipAddr)
{
	uint   regAddr;
	uchar   value;

	i2c_set_bus_num(i2c_bus_number);

	regAddr = 0x0; value = 0x69;
	if (i2c_write(chipAddr, regAddr, 1, &value, 1) != 0)
		printf("Error writing the fan controller 0x%x register.\n",
				regAddr);

	regAddr = 0x1; value = 0x3;
	if (i2c_write(chipAddr, regAddr, 1, &value, 1) != 0)
		printf("Error writing the fan controller 0x%x register.\n",
				regAddr);

	regAddr = 0x4; value = 0x80;
	if (i2c_write(chipAddr, regAddr, 1, &value, 1) != 0)
		printf("Error writing the fan controller 0x%x register.\n",
				regAddr);

	regAddr = 0x20; value = 0x1d;
	if (i2c_write(chipAddr, regAddr, 1, &value, 1) != 0)
		printf("Error writing the fan controller 0x%x register.\n",
				regAddr);

	regAddr = 0x24; value = 0x51;
	if (i2c_write(chipAddr, regAddr, 1, &value, 1) != 0)
		printf("Error writing the fan controller 0x%x register.\n",
				regAddr);

	regAddr = 0x25; value = 0x51;
	if (i2c_write(chipAddr, regAddr, 1, &value, 1) != 0)
		printf("Error writing the fan controller 0x%x register.\n",
				regAddr);

	regAddr = 0x21; value = 0x60;
	if (i2c_write(chipAddr, regAddr, 1, &value, 1) != 0)
		printf("Error writing the fan controller 0x%x register.\n",
				regAddr);

	return 0;
}
#endif

int misc_init_r(void)
{
#ifdef CONFIG_CONFIGURE_FAN
	configure_fan(SET_I2C_BUS(0), FAN_CONTROLLER_SLAVE_ADDR_18);
#endif
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6q_i2c1_pad_info);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6q_i2c2_pad_info);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6q_i2c3_pad_info);
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

	return 0;
}

int checkboard(void)
{
	if (is_revd1())
		puts("Board: Liveuboard rev D1\n");
	else if (is_revc1())
		puts("Board: Liveuboard rev C1\n");
	else
		puts("Board: Liveuboard rev B1\n");

	return 0;
}
