// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2014 O.S. Systems Software LTDA.
 * Copyright (C) 2018 LiveU
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 * Author: Jemish Patel <jemish@liveu.tv>
 * This is file is created based "board/wandboard/wandboard.c"
 */

#include <common.h>
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
#include <spi_eeprom.h>
#include <liveu_board.h>

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

#define UART_BUFFER_CONTROL	IMX_GPIO_NR(1, 6)
#define GPIO_DRY_CONTACT2_EN	IMX_GPIO_NR(1, 7)
#define GPIO_REAR_USB_EN	IMX_GPIO_NR(1, 8)
#define GPIO_PCI_RESET		IMX_GPIO_NR(1, 16)
#define GPIO_LCD_ON_OFF		IMX_GPIO_NR(1, 18)
#define LCD_CAP_RST		IMX_GPIO_NR(1, 24)
#define GPIO_DRY_CONTACT1_EN	IMX_GPIO_NR(1, 24)
#define USDHC3_CD_GPIO		IMX_GPIO_NR(3, 9)
#define GPIO_WIFI_DISABLE	IMX_GPIO_NR(3, 19)
#define GPIO_WIFI_EN		IMX_GPIO_NR(3, 20)
#define ETH_PHY_RESET		IMX_GPIO_NR(3, 29)
#define GPIO_WLAN_VOLATGE	IMX_GPIO_NR(3, 31)
#define GPIO_OSC_32_BT_EN	IMX_GPIO_NR(4, 15)
#define GPIO_BOOT_POR_EN	IMX_GPIO_NR(7, 13)

#define LIVEU_MODEL_TYPE_BIT0	IMX_GPIO_NR(1, 18)
#define LIVEU_MODEL_TYPE_BIT1	IMX_GPIO_NR(1, 24)
#define LIVEU_MODEL_TYPE_BIT2	IMX_GPIO_NR(2, 3)
#define LIVEU_MODEL_TYPE_BIT3	IMX_GPIO_NR(2, 2)

#define GPIO2_DATA_REGISTER	(0x20a0000)
#define CCM_CCGR_1		(0x20c406c)
#define ECSPI1_CLK_ENABLE	(0x3)
#define ECSPI2_CLK_ENABLE	(0xC)
#define ECSPI1_CLK_DISABLE	(0xfffffffc)
#define ECSPI2_CLK_DISABLE	(0xfffffff3)
#define ETHERNET_MAC_OFFSET	(19)
#define USBETHERNET_MAC_OFFSET	(25)
#define MAC_ADDRESS_SIZE	(6)
#define MAX_BUFFER_SIZE		(50)
#define MAX_CMDLINE_SIZE	(300)
#define USBETHADDR_ENV_SIZE	(22)
#define FEC_ADDR_LOW		(0x21880E4) /* Low 32bits MAC address */
#define FEC_ADDR_HIGH		(0x21880E8) /* High 16bits MAC address */
#define I2C_BUS_0		(0)
#define I2C_BUS_1		(1)
#define I2C_BUS_2		(2)
#define SET_I2C_BUS(x)		I2C_BUS_##x
/* Enabled Marvell switch MV88E6176 u-boot post command and
   SMI device address generated using ADDR[4:1] pull up pin
   configuration of marvell switch
*/
#define MV88E6176_ADDRESS		(0x1E)

#define DEFAULT_BRIGHTNESS_LEVEL	(20)
#define CORECARD_REVISION_OFFSET	(50)
#define LU600_BOARD_DEFAULT_HARDWARE_REVISION	('B')
#define LU300_BOARD_DEFAULT_HARDWARE_REVISION	('C')
#define GPIO_CONFIGURATION_STABLE_DURATION_MS	(150)

_model_type model_type;
unsigned int liveu_model_type_gpio[] = {
				LIVEU_MODEL_TYPE_BIT0,
				LIVEU_MODEL_TYPE_BIT1,
				LIVEU_MODEL_TYPE_BIT2,
				LIVEU_MODEL_TYPE_BIT3
				};
#define LIVEU_MODEL_TYPE_GPIOS	ARRAY_SIZE(liveu_model_type_gpio)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const gpio_pads[] = {
	IOMUX_PADS(PAD_GPIO_7__GPIO1_IO07     | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_8__GPIO1_IO08     | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_DAT0__GPIO1_IO16   | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_CMD__GPIO1_IO18    | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_RX_ER__GPIO1_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D0__GPIO2_IO00   | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D2__GPIO2_IO02   | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D3__GPIO2_IO03   | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D19__GPIO3_IO19    | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D20__GPIO3_IO20    | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D31__GPIO3_IO31    | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_19__GPIO4_IO05    | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW4__GPIO4_IO15   | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_6__GPIO1_IO06   	 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const uart1_2_pads[] = {
	IOMUX_PADS(PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
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

/* Marvell MV88E6176 switch VLAN configuration */
static struct mv88e6176_sw_reg switch_vlan_conf[] = {
	/* Clear VLAN ID Table */
	{ GLOBAL1, VTU_OPS, VTU_BUSY_BIT | VTU_FLUSH_ALL},

	/* Create VLAN 1 */
	/* Load VTU Entry */
	{ GLOBAL1, VTU_FID, 0x0},
	{ GLOBAL1, VTU_SID, 0x0},
	{ GLOBAL1, VSTU_DATA_P0TO3, 0x3300},
	{ GLOBAL1, VSTU_DATA_P4TO6, 0x0003},
	{ GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_1_ID},
	{ GLOBAL1, VTU_OPS, VTU_BUSY_BIT | VTU_LOAD_ENTRY},
	/* Load STU Entry */
	{ GLOBAL1, VTU_FID, 0x0},
	{ GLOBAL1, VTU_SID, 0x0},
	{ GLOBAL1, VSTU_DATA_P0TO3, 0xCCCC},
	{ GLOBAL1, VSTU_DATA_P4TO6, 0x0CCC},
	{ GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_1_ID},
	{ GLOBAL1, VTU_OPS, VTU_BUSY_BIT | STU_LOAD_ENTRY},

	/* Create VLAN 5 */
	/* Load VTU Entry */
	{ GLOBAL1, VTU_FID, 0x0},
	{ GLOBAL1, VTU_SID, 0x0},
	{ GLOBAL1, VSTU_DATA_P0TO3, 0x3333},
	{ GLOBAL1, VSTU_DATA_P4TO6, 0x0003},
	{ GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_5_ID},
	{ GLOBAL1, VTU_OPS, VTU_BUSY_BIT | VTU_LOAD_ENTRY},
	/* Load STU Entry */
	{ GLOBAL1, VTU_FID, 0x0},
	{ GLOBAL1, VTU_SID, 0x0},
	{ GLOBAL1, VSTU_DATA_P0TO3, 0xCCCC},
	{ GLOBAL1, VSTU_DATA_P4TO6, 0x0CCC},
	{ GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_5_ID},
	{ GLOBAL1, VTU_OPS, VTU_BUSY_BIT | STU_LOAD_ENTRY},

	/* Enable IEEE 802.1Q VLAN on PORT */
	{ PORT(0), PORT_CONTROL2, 0x2c80},
	{ PORT(1), PORT_CONTROL2, 0x2c80},
	{ PORT(2), PORT_CONTROL2, 0x2c80},
	{ PORT(3), PORT_CONTROL2, 0x2c80},
	{ PORT(4), PORT_CONTROL2, 0x2c80},
	{ PORT(5), PORT_CONTROL2, 0x2c80},
	{ PORT(6), PORT_CONTROL2, 0x2c80}
};

static struct mv88e6176_sw_reg spare_vlans_vtu_entry[] = {
    /* Load VTU Entry */
    { GLOBAL1, VTU_FID, 0x0},
    { GLOBAL1, VTU_SID, 0x0},
    { GLOBAL1, VSTU_DATA_P0TO3, 0x3332},
    { GLOBAL1, VSTU_DATA_P4TO6, 0x0233}
};

static struct mv88e6176_sw_reg spare_vlans_stu_entry[] = {
    /* Load STU Entry */
    { GLOBAL1, VTU_FID, 0x0},
    { GLOBAL1, VTU_SID, 0x0},
    { GLOBAL1, VSTU_DATA_P0TO3, 0xCCCC},
    { GLOBAL1, VSTU_DATA_P4TO6, 0x0CCC}
};

static struct mv88e6176_sw_reg spare_vlans_vlan_id[] = {
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_10_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_11_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_12_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_13_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_14_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_15_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_16_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_17_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_18_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_19_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_20_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_21_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_22_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_23_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_24_ID},
    { GLOBAL1, VTU_VID, VTU_VALID_BIT | VLAN_25_ID}
};

static struct mv88e6176_sw_reg spare_vlans_stu_load[] = {
    { GLOBAL1, VTU_OPS, VTU_BUSY_BIT | STU_LOAD_ENTRY}
};

static struct mv88e6176_sw_reg spare_vlans_vtu_load[] = {
    { GLOBAL1, VTU_OPS, VTU_BUSY_BIT | VTU_LOAD_ENTRY}
};

static void setup_iomux_gpio(void)
{
	SETUP_IOMUX_PADS(gpio_pads);
}

static void setup_iomux_uart(void)
{
	u32 gpio32;
	gpio32 = __raw_readl(GPIO2_DATA_REGISTER);
	if (gpio32 & 0x1)
		SETUP_IOMUX_PADS(uart1_pads);
	else
		SETUP_IOMUX_PADS(uart1_2_pads);
}

static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads);

	gpio_direction_output(ETH_PHY_RESET, 0);
	udelay(500);
	gpio_set_value(ETH_PHY_RESET, 1);
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

void update_uboot_env_in_bootargs(const char *env_variable, const char *env_value)
{
	char buffer[MAX_BUFFER_SIZE] = {0};
	char cmdline_args[MAX_CMDLINE_SIZE];
	const char  *env = env_get("bootargs");

	if (!env_variable || !*env_variable || !env_value) {
		printf("Invalid Arguments in %s!\n", __func__);
		return;
	}

	if (!env) {
		printf("bootargs not defined!\n");
		return;
	}

	strcpy(cmdline_args, env);
	sprintf(buffer," %s=", env_variable);
	char *cmdline_env_variable = strstr(cmdline_args, buffer);

	if (cmdline_env_variable) {
		char cmdline_env_value_for_strtok_input[MAX_CMDLINE_SIZE];
		char *cmdline_env_value = strstr(cmdline_env_variable, "=");
		++cmdline_env_value;
		strcpy(cmdline_env_value_for_strtok_input, cmdline_env_value);

		char *env_value_saved = strtok(cmdline_env_value_for_strtok_input, " ");

		if (!strcmp(env_value_saved, env_value))
			return;

		char *cmdline_remaining_env = strtok(NULL, "\0");
		strcpy(cmdline_env_value, env_value);
		strcat(cmdline_env_value, " ");
		strcat(cmdline_env_value, cmdline_remaining_env);
	} else {
		strcat(buffer, env_value);
		strcat(cmdline_args, buffer);
	}

	env_set("bootargs", cmdline_args);
}

int is_valid_mac_address(unsigned char *addr)
{
	if (is_zero_ethaddr(addr)) {
		printf("mac address is all 0x00\n");
		return -1;
	} else if (is_broadcast_ethaddr(addr)) {
		printf("mac address is all 0xff\n");
		return -1;
	} else if (is_multicast_ethaddr(&addr[5])) {
		printf("mulitcast address!\n");
		return -1;
	}
	else
		return 0;
}

#ifdef CONFIG_MXC_SPI
int setup_eth_mac_address(void)
{
	int ret = 0;
	unsigned char mac_address[MAC_ADDRESS_SIZE];
	unsigned int offset = ETHERNET_MAC_OFFSET;
	char buffer[MAX_BUFFER_SIZE];
	char *env;

	env = env_get("eth0_mac_offset");
	if (env != NULL) {
		strict_strtoul(env, 10,(long unsigned int *) &offset);
	}
	else {
		printf("Environment variable eth0_mac_offset is not set, reading from default offset\n");
		offset = ETHERNET_MAC_OFFSET;
	}
	ret = read_eeprom(offset, mac_address, MAC_ADDRESS_SIZE);
	if (ret) {
		printf("Failed to read eth0 mac address\n");
		return ret;
	}
	else {
		if (!is_valid_mac_address(mac_address)) {
			memset(buffer, 0x00, sizeof(buffer));
			sprintf(buffer, "setenv ethaddr %.2x:%.2x:%.2x:%.2x:%.2x:%.2x",
					mac_address[5], mac_address[4], mac_address[3], mac_address[2], mac_address[1], mac_address[0]);
			run_command(buffer, 0);
			writel((mac_address[5] << 24) | ( mac_address[4] << 16) | (mac_address[3] << 8) | (mac_address[2]), FEC_ADDR_LOW);
			writel((mac_address[1] << 24) | ( mac_address[0] << 16) | 0x8808, FEC_ADDR_HIGH);
		}
	}
	return ret;
}

int setup_usbeth_mac_address(void)
{
	int ret = 0;
	unsigned char mac_address[MAC_ADDRESS_SIZE];
	unsigned int offset = ETHERNET_MAC_OFFSET;
	char buffer[MAX_BUFFER_SIZE];
	char env_value[MAX_BUFFER_SIZE];

	char* env = env_get("eth1_mac_offset");
	if (env) {
		strict_strtoul(env, 10, (long unsigned int *)&offset);
	}
	else {
		printf("Environment variable eth1_mac_offset is not set, reading from default offset\n");
		offset = USBETHERNET_MAC_OFFSET;
	}

	ret = read_eeprom(offset, mac_address, MAC_ADDRESS_SIZE);
	if (ret) {
		printf("Failed to read eth1 mac address\n");
		return ret;
	} else {
		if (!is_valid_mac_address(mac_address)) {
			memset(buffer, 0x00, sizeof(buffer));
			sprintf(buffer,
				"setenv usbethaddr %.2x:%.2x:%.2x:%.2x:%.2x:%.2x",
				mac_address[5], mac_address[4], mac_address[3], mac_address[2], mac_address[1], mac_address[0]);

			run_command(buffer, 0);

			memset(env_value, 0x00, sizeof(env_value));
			sprintf(env_value,"%.2x:%.2x:%.2x:%.2x:%.2x:%.2x",
				mac_address[5], mac_address[4], mac_address[3], mac_address[2], mac_address[1], mac_address[0]);
			update_uboot_env_in_bootargs("usbethaddr", env_value);
		}
	}

	return ret;
}

int setup_mac_addresses(void)
{
	int ret = 0;

	ret = setup_eth_mac_address();
	if (ret) {
		printf("Failed to set eth0 mac address\n");
		return ret;
	}
	ret = setup_usbeth_mac_address();
	if (ret) {
		printf("Failed to set eth1 mac address\n");
		return ret;
	}

	return ret;
}

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
	IOMUX_PADS(PAD_GPIO_18__GPIO7_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

void ecspi1_clock_enable(void)
{
	u32 ecspi_clk;
	ecspi_clk = __raw_readl(CCM_CCGR_1);
	ecspi_clk |= ECSPI1_CLK_ENABLE;
	__raw_writel(ecspi_clk, CCM_CCGR_1);
}

void ecspi2_clock_enable(void)
{
	u32 ecspi_clk;

	ecspi_clk = __raw_readl(CCM_CCGR_1);
	ecspi_clk |= ECSPI2_CLK_ENABLE;
	__raw_writel(ecspi_clk, CCM_CCGR_1);
}

void ecspi1_clock_disable(void)
{
	u32 ecspi_clk;
	ecspi_clk = __raw_readl(CCM_CCGR_1);
	ecspi_clk &= ECSPI1_CLK_DISABLE;
	__raw_writel(ecspi_clk, CCM_CCGR_1);
}

void ecspi2_clock_disable(void)
{
	u32 ecspi_clk;

	ecspi_clk = __raw_readl(CCM_CCGR_1);
	ecspi_clk &= ECSPI2_CLK_DISABLE;
	__raw_writel(ecspi_clk, CCM_CCGR_1);
}

void spi_clock_enable(void)
{
	ecspi1_clock_enable();
	ecspi2_clock_enable();
}


void spi_clock_disable(void)
{
	ecspi1_clock_disable();
	ecspi2_clock_disable();
}

void setup_clock_and_chipselect(void)
{
	spi_clock_enable();

	gpio_direction_output(GPIO_BOOT_POR_EN , 1);
	gpio_direction_output(ECSPI2_SPI_CHIPSELECT, 0);
	gpio_direction_output(EEPROM_WRITE_PROTECT, 1);
}

void setup_spi(void)
{
	SETUP_IOMUX_PADS(ecspi1_pads);
	SETUP_IOMUX_PADS(ecspi2_pads);
	setup_clock_and_chipselect();
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

#if defined(CONFIG_PWMCNTL_BACKLIGHT)
void add_parameter_brightness(int brightness)
{
	char buffer[MAX_BUFFER_SIZE];
	char env_value[MAX_BUFFER_SIZE];

	if (brightness < BRIGHTNESS_LEVEL_MIN || brightness > BRIGHTNESS_LEVEL_MAX)
		return;

	memset(buffer, 0x00, sizeof(buffer));
	sprintf(buffer, "setenv brightness %.2d", brightness);
	run_command(buffer, 0);

	memset(env_value, 0x00, sizeof(env_value));
	sprintf(env_value,"%.2d", brightness);
	update_uboot_env_in_bootargs("brightness", env_value);
}

unsigned int get_brightess(void)
{
	unsigned int brightness = DEFAULT_BRIGHTNESS_LEVEL;
	char* env = env_get("brightness");

	if (env)
		strict_strtoul(env, 10, (long unsigned int *)&brightness);

	return brightness;
}

int map_brightness_level_to_dutycycle_percentage(unsigned int brightness)
{
	return (MINIMUM_DUTYCYCLE_PERCENTAGE + (brightness * STEP_SIZE));
}

unsigned int compute_dutycycle(int pwm, unsigned int brightness, unsigned int period)
{
	int dutycycle_percentage;
	if ((brightness < BRIGHTNESS_LEVEL_MIN) || (brightness > BRIGHTNESS_LEVEL_MAX))
		return period;
	else {
		dutycycle_percentage = map_brightness_level_to_dutycycle_percentage(brightness);
		return ((period * dutycycle_percentage)/100);
	}
}

int enable_backlight(void)
{
	unsigned int dutycycle = CLOCK_CYCLE_PERIOD;
	unsigned int brightness = DEFAULT_BRIGHTNESS_LEVEL;

	/* Initialize PWM 3 */
	if (pwm_init(PWM3, 0, 0))
		puts("error init pwm for backlight\n");
	brightness = get_brightess();
	dutycycle = compute_dutycycle(PWM3, brightness, CLOCK_CYCLE_PERIOD);
	if (pwm_config(PWM3, dutycycle, CLOCK_CYCLE_PERIOD))
		puts("error config pwm for backlight\n");
	if (pwm_enable(PWM3))
		puts("error enable pwm for backlight\n");
	add_parameter_brightness(brightness);
	return 0;
}
#endif

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
	switch (model_type)
	{
		case BOARD_LU600:
			gpio_direction_output(GPIO_LCD_ON_OFF, 1);
			return 1;
		break;
		case BOARD_LU300:
		case BOARD_LU610:
		default:
			return 0;
		break;
	}
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
} },{
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
} },};
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

void get_corecard_revision(unsigned char *hardware_revision)
{
	unsigned int offset = CORECARD_REVISION_OFFSET;

	if( read_eeprom(offset, hardware_revision, 1) ) {
		printf("Warning: failed read corecard revision info, setting 'B' as default\n");
		if (BOARD_LU600 == model_type)
			*hardware_revision = LU600_BOARD_DEFAULT_HARDWARE_REVISION;
		else if ((BOARD_LU300 == model_type) || (BOARD_LU610 == model_type))
			*hardware_revision = LU300_BOARD_DEFAULT_HARDWARE_REVISION;
	}
	/* if reading EEPROM doesn't have value in between 'E' to 'Z' */
	if( !((*hardware_revision >= 'E') && (*hardware_revision <= 'Z')) )
	{
		if (BOARD_LU600 == model_type)
			*hardware_revision = LU600_BOARD_DEFAULT_HARDWARE_REVISION;
		else if ((BOARD_LU300 == model_type) || (BOARD_LU610 == model_type))
			*hardware_revision = LU300_BOARD_DEFAULT_HARDWARE_REVISION;
	}
}

int get_hardware_revision(void)
{
	char buffer[MAX_BUFFER_SIZE];
	char env_value[MAX_BUFFER_SIZE];
	unsigned char corecard_hardware_revision;

	get_corecard_revision(&corecard_hardware_revision);
	memset(buffer, 0x00, sizeof(buffer));
	sprintf(buffer, "setenv hwrev %c", corecard_hardware_revision);
	run_command(buffer, 0);

	memset(env_value, 0x00, sizeof(env_value));
	sprintf(env_value,"%c", corecard_hardware_revision);
	update_uboot_env_in_bootargs("hwrev", env_value);
	return 0;
}

#define IOEXP1_SLAVE_ADDRESS	(0x21)
#define IOEXP2_SLAVE_ADDRESS	(0x22)

int configure_ioexpanders(void)
{
	uchar   chipAddr1 = IOEXP1_SLAVE_ADDRESS;
	uchar   chipAddr2 = IOEXP2_SLAVE_ADDRESS;
	uint   regAddr;
	uchar   value, readback_value;

	i2c_set_bus_num(0);

	/* IO Expander (with slave address 0x21) default settings */
	regAddr = 0x6; value = 0xff;
	if (i2c_write(chipAddr1, regAddr, 1, &value, 1) != 0)
		printf("Error writing the ioexpander 0x%x 0x%x register.\n",
				chipAddr1, regAddr);
	readback_value = 0;
	if(i2c_read(chipAddr1, regAddr, 1, &readback_value, 1) != 0)
		printf("Readback of the ioexpander 0x%x 0x%x register failed."
				"Readback value = 0x%x\n", chipAddr1, regAddr, readback_value);

	if(value != readback_value)
		printf("Warning: Readback value (0x%x) is different from set value (0x%x)\n",
				readback_value, value);

	regAddr = 0x7; value = 0xff;
	if (i2c_write(chipAddr1, regAddr, 1, &value, 1) != 0)
		printf("Error writing the ioexpander 0x%x 0x%x register.\n",
				chipAddr1, regAddr);
	readback_value = 0;
	if(i2c_read(chipAddr1, regAddr, 1, &readback_value, 1) != 0)
		printf("Readback of the ioexpander 0x%x 0x%x register failed."
				"Readback value = 0x%x\n", chipAddr1, regAddr, readback_value);

	if(value != readback_value)
		printf("Warning: Readback value (0x%x) is different from set value (0x%x)\n",
				readback_value, value);

	regAddr = 0x2; value = 0xff;
	if (i2c_write(chipAddr1, regAddr, 1, &value, 1) != 0)
		printf("Error writing the ioexpander 0x%x 0x%x register.\n",
				chipAddr1, regAddr);
	readback_value = 0;
	if(i2c_read(chipAddr1, regAddr, 1, &readback_value, 1) != 0)
		printf("Readback of the ioexpander 0x%x 0x%x register failed."
				"Readback value = 0x%x\n", chipAddr1, regAddr, readback_value);

	if(value != readback_value)
		printf("Warning: Readback value (0x%x) is different from set value (0x%x)\n",
				readback_value, value);

	regAddr = 0x3; value = 0xff;
	if (i2c_write(chipAddr1, regAddr, 1, &value, 1) != 0)
		printf("Error writing the ioexpander 0x%x 0x%x register.\n",
				chipAddr1, regAddr);
	readback_value = 0;
	if(i2c_read(chipAddr1, regAddr, 1, &readback_value, 1) != 0)
		printf("Readback of the ioexpander 0x%x 0x%x register failed."
				"Readback value = 0x%x\n", chipAddr1, regAddr, readback_value);

	if(value != readback_value)
		printf("Warning: Readback value (0x%x) is different from set value (0x%x)\n",
				readback_value, value);

	/* IO Expander (with slave address 0x22) default settings */
	regAddr = 0x6; value = 0xe7;
	if (i2c_write(chipAddr2, regAddr, 1, &value, 1) != 0)
		printf("Error writing the ioexpander 0x%x 0x%x register.\n",
				chipAddr2, regAddr);
	readback_value = 0;
	if(i2c_read(chipAddr2, regAddr, 1, &readback_value, 1) != 0)
		printf("Readback of the ioexpander 0x%x 0x%x register failed."
				"Readback value = 0x%x\n", chipAddr2, regAddr, readback_value);

	if(value != readback_value)
		printf("Warning: Readback value (0x%x) is different from set value (0x%x)\n",
				readback_value, value);

	regAddr = 0x7; value = 0xff;
	if (i2c_write(chipAddr2, regAddr, 1, &value, 1) != 0)
		printf("Error writing the ioexpander 0x%x 0x%x register.\n",
				chipAddr2, regAddr);
	readback_value = 0;
	if(i2c_read(chipAddr2, regAddr, 1, &readback_value, 1) != 0)
		printf("Readback of the ioexpander 0x%x 0x%x register failed."
				"Readback value = 0x%x\n", chipAddr2, regAddr, readback_value);

	if(value != readback_value)
		printf("Warning: Readback value (0x%x) is different from set value (0x%x)\n",
				readback_value, value);

	regAddr = 0x2; value = 0xef;
	if (i2c_write(chipAddr2, regAddr, 1, &value, 1) != 0)
		printf("Error writing the ioexpander 0x%x 0x%x register.\n",
				chipAddr1, regAddr);
	readback_value = 0;
	if(i2c_read(chipAddr2, regAddr, 1, &readback_value, 1) != 0)
		printf("Readback of the ioexpander 0x%x 0x%x register failed."
				"Readback value = 0x%x\n", chipAddr2, regAddr, readback_value);

	if(value != readback_value)
		printf("Warning: Readback value (0x%x) is different from set value (0x%x)\n",
				readback_value, value);

	regAddr = 0x3; value = 0x8b;
	if (i2c_write(chipAddr2, regAddr, 1, &value, 1) != 0)
		printf("Error writing the ioexpander 0x%x 0x%x register.\n",
				chipAddr2, regAddr);
	readback_value = 0;
	if(i2c_read(chipAddr2, regAddr, 1, &readback_value, 1) != 0)
		printf("Readback of the ioexpander 0x%x 0x%x register failed."
				"Readback value = 0x%x\n", chipAddr2, regAddr, readback_value);

	if(value != readback_value)
		printf("Warning: Readback value (0x%x) is different from set value (0x%x)\n",
				readback_value, value);
	return 0;
}

static void bring_ctp_out_of_reset(void)
{
	gpio_direction_output(LCD_CAP_RST, 1);
}

int last_stage_init(void)
{
	/* configure MV88E6176 switch */
	char *name = "FEC";
	int vlan_id_offset;

	if (miiphy_set_current_dev(name))
		return 0;

	mv88e6176_sw_program(name, MV88E6176_ADDRESS, switch_conf,
			ARRAY_SIZE(switch_conf));

	mv88e6176_sw_program(name, MV88E6176_ADDRESS, switch_vlan_conf,
			ARRAY_SIZE(switch_vlan_conf));

	for (vlan_id_offset = 0; vlan_id_offset < TOTAL_SPARE_VLAN; vlan_id_offset++) {
		mv88e6176_sw_program(name, MV88E6176_ADDRESS, spare_vlans_vtu_entry,
			ARRAY_SIZE(spare_vlans_vtu_entry));
		mv88e6176_sw_program(name, MV88E6176_ADDRESS, &spare_vlans_vlan_id[vlan_id_offset], 1);
		mv88e6176_sw_program(name, MV88E6176_ADDRESS, spare_vlans_vtu_load,
			ARRAY_SIZE(spare_vlans_vtu_load));

		mv88e6176_sw_program(name, MV88E6176_ADDRESS, spare_vlans_stu_entry,
			ARRAY_SIZE(spare_vlans_stu_entry));
		mv88e6176_sw_program(name, MV88E6176_ADDRESS, &spare_vlans_vlan_id[vlan_id_offset], 1);
		mv88e6176_sw_program(name, MV88E6176_ADDRESS, spare_vlans_stu_load,
			ARRAY_SIZE(spare_vlans_stu_load));
	}

	mv88e6176_sw_reset(name, MV88E6176_ADDRESS);

	if (BOARD_LU600 == model_type)
		configure_ioexpanders();

	if (BOARD_LU600 == model_type)
		bring_ctp_out_of_reset();

	return 0;
}


int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();

	return cpu_eth_init(bis);
}

int board_early_init_f(void)
{
	setup_iomux_gpio();
	setup_iomux_uart();

	return 0;
}

void configure_usb_gpios(void)
{
	gpio_direction_output(GPIO_DRY_CONTACT2_EN, 0);
	gpio_direction_output(GPIO_REAR_USB_EN, 0);
	gpio_direction_output(GPIO_DRY_CONTACT1_EN, 0);
}

void configure_pci_wlan_gpios(void)
{
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	clrbits_le32(&iomuxc_regs->gpr[12], IOMUXC_GPR12_APPS_LTSSM_ENABLE);

	gpio_direction_output(GPIO_WIFI_EN, 0);
	gpio_direction_output(GPIO_WIFI_DISABLE, 0);
	gpio_direction_output(GPIO_PCI_RESET, 0);
	gpio_direction_output(GPIO_OSC_32_BT_EN, 0);

	mdelay(100);

	gpio_set_value(GPIO_WIFI_DISABLE, 1);
	gpio_set_value(GPIO_WIFI_EN, 1);

	mdelay(100);
	gpio_set_value(GPIO_OSC_32_BT_EN, 1);
}

static void configure_wl18xx_gpio(void)
{
	gpio_direction_output(GPIO_WLAN_VOLATGE , 1);
	gpio_direction_output(GPIO_WIFI_EN , 1);
}

int setup_gpios(void)
{
	int ret = 0;
	if (BOARD_LU600 == model_type) {
		configure_wl18xx_gpio();
	} else {
		configure_pci_wlan_gpios();
		if (BOARD_LU610 == model_type) {
			configure_usb_gpios();
		}
	}
	return ret;
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
#endif
	return 0;
}

#define I2C_EEPROM_ADDRESS	0x57
#define I2C_TIMEOUT_RETRY_COUNT	3

int vic_card_detection(void)
{
	char buffer[MAX_BUFFER_SIZE];
	char env_value[MAX_BUFFER_SIZE];
	uchar chipAddr = I2C_EEPROM_ADDRESS;
	uint regAddr = 0x0;
	uchar value;
	int ret = 0, retry_count = I2C_TIMEOUT_RETRY_COUNT;

	i2c_set_bus_num(0);

	do {
		ret = i2c_read(chipAddr, regAddr, 1, &value, 1);
		if(ret == 0) {
			strncpy(env_value, "AVIC", MAX_BUFFER_SIZE);
			break;
		} else if (ret == -EREMOTEIO) {
			strncpy(env_value, "CVIC", MAX_BUFFER_SIZE);
			break;
		} else {
			printf("VIC card detection (i2c) error %d, retrying\n", ret);
			retry_count--;
			mdelay(10);
		}
	}
	while(retry_count >= 0);

	if(retry_count == -1) {
		printf("Warning: fail to detect VIC type after %d retries\n",
			I2C_TIMEOUT_RETRY_COUNT);
		strncpy(env_value, "AVIC", MAX_BUFFER_SIZE);
		printf("Setting explicitly VIC type as 'AVIC'");
	}

	memset(buffer, 0x00, sizeof(buffer));
	sprintf(buffer, "setenv vic_type %s", env_value);
	run_command(buffer, 0);

	update_uboot_env_in_bootargs("vic_type", env_value);
	return 0;
}

/* HW Team changed EEPROM slave address to 0x55, due to conflict with HDMI slave */
#define EEPROM_SLAVE_ADDRESS (0x55)
static int detect_i2c_eeprom(void)
{
	return (0 == i2c_set_bus_num(2)) &&
			(0 == i2c_probe(EEPROM_SLAVE_ADDRESS));
}

static void configure_uart_normal(void)
{
	gpio_direction_output(UART_BUFFER_CONTROL, 0);
}

static void configure_uart_crossed(void)
{
	gpio_direction_output(UART_BUFFER_CONTROL, 1);
}

static void check_if_jig_is_connected_with_eeprom_on_i2c(void)
{
	/* Set to low as per HW team's input */
	if (detect_i2c_eeprom())
		configure_uart_crossed();
	else
		configure_uart_normal();
}

#if defined(CONFIG_CONFIGURE_FAN)
#define FAN_CONTROLLER_SLAVE_ADDR_18		(0x18)
#define FAN_CONTROLLER_SLAVE_ADDR_19		(0x19)
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

int check_if_modem_board_present(void)
{
	uchar chipAddr = 0x20;
	uint   regAddr = 0x00;
	uchar   value;

	i2c_set_bus_num(1);
	return i2c_read(chipAddr, regAddr, 1, &value, 1);
}

int misc_init_r(void)
{
	check_if_jig_is_connected_with_eeprom_on_i2c();
#ifdef CONFIG_CONFIGURE_FAN
	configure_fan(SET_I2C_BUS(0), FAN_CONTROLLER_SLAVE_ADDR_18);
	if (BOARD_LU610 == model_type) {
		configure_fan(SET_I2C_BUS(1), FAN_CONTROLLER_SLAVE_ADDR_18);
		if (!check_if_modem_board_present())
			configure_fan(SET_I2C_BUS(1), FAN_CONTROLLER_SLAVE_ADDR_19);
	}
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
	return 0;
}

void set_liveu_model_type(char *env_value)
{
	char buffer[MAX_BUFFER_SIZE];

	memset(buffer, 0x00, sizeof(buffer));
	sprintf(buffer, "setenv model_type %s", env_value);
	run_command(buffer, 0);

	update_uboot_env_in_bootargs("model_type", env_value);
}

int get_liveu_model_type(void)
{
	int i = 0;
	int gpio_value[LIVEU_MODEL_TYPE_GPIOS];
	char env_value[MAX_BUFFER_SIZE];

	for (i = 0; i < LIVEU_MODEL_TYPE_GPIOS; i++) {
		gpio_direction_input(liveu_model_type_gpio[i]);
	}
	mdelay(GPIO_CONFIGURATION_STABLE_DURATION_MS);
	for (i = 0; i < LIVEU_MODEL_TYPE_GPIOS; i++) {
		gpio_value[i] = gpio_get_value(liveu_model_type_gpio[i]);
	}
	if ( 0 == gpio_value[3]) {
		model_type = BOARD_LU600;
	} else {
		for (i = 0 ; i < LIVEU_MODEL_TYPE_GPIOS; i++) {
			model_type |= (gpio_value[i] << i);
		}
	}
	puts("BOARD: ");
	switch ((int)model_type)
	{
		case BOARD_LU600:
			puts("LU600\n");
			strcpy(env_value, "LU600");
		break;
		case BOARD_LU300:
		case BOARD_LU300_WORKAROUND_VALUE:
			model_type = BOARD_LU300;
			puts("LU300\n");
			strcpy(env_value, "LU300");
		break;
		case BOARD_LU610:
		case BOARD_LU610_WORKAROUND_VALUE:
			model_type = BOARD_LU610;
			puts("LU610\n");
			strcpy(env_value, "LU610");
		break;
		default:
			puts("Invalid\n");
	}
	for (i = 0; i < LIVEU_MODEL_TYPE_GPIOS; i++) {
		gpio_free(liveu_model_type_gpio[i]);
	}
	set_liveu_model_type(env_value);
	return 0;
}
