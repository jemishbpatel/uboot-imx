/*
 * (C) Copyright 2015-16, LiveU Ltd.
 *
 * Marvell switch configuration driver declaration
 *
 * Auther Hitesh Viradiya & Jemish Patel.
 *
 * Nitsan: Added Extra VLANs configuration constants
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MV88E6176_H
#define __MV88E6176_H

#include <common.h>

/* PHY registers */
#define PHY(itf)	(itf)

#define PHY_CTRL	0x00
#define PHY_100_MBPS	0x2000
#define PHY_1_GBPS	0x0040
#define AUTONEG_EN	0x1000
#define AUTONEG_RST	0x0200
#define FULL_DUPLEX	0x0100
#define PHY_PWR_DOWN	0x0800
#define PHY_RESET	0x8000

#define PHY_STATUS	0x01
#define AN1000FIX	0x0001

#define PHY_SPEC_CTRL	0x10
#define SPEC_PWR_DOWN	0x0004
#define AUTO_MDIX_EN	0x0060

#define PHY_1000_CTRL	0x9

#define NO_ADV		0x0000
#define ADV_1000_FDPX	0x0200
#define ADV_1000_HDPX	0x0100

#define PHY_PAGE	0x16

#define AN1000FIX_PAGE	0x00fc

/* PORT or MAC registers */
#define PORT(itf)	(itf+0x10)
#define GLOBAL2         0x1c
#define GLOBAL1         0x1b
#define VTU_OPS		0x5
#define VTU_FID		0x2
#define VTU_SID		0x3
#define VTU_VID		0x6
/* VTU/STU Data Register for Port 0 to Port 3 */
#define VSTU_DATA_P0TO3	0x7
/* VTU/STU Data Register for Port 4 to Port 6 */
#define VSTU_DATA_P4TO6	0x8
#define PORT_CONTROL2	0x8

#define VTU_BUSY_BIT	0x8000
#define VTU_FLUSH_ALL	0x1000
#define VTU_LOAD_ENTRY	0x3000
#define STU_LOAD_ENTRY	0x5000
#define VTU_VALID_BIT	0x1000
#define VLAN_1_ID	0x1
#define VLAN_5_ID	0x5
/* VLANS for xTender communication */
#define VLAN_10_ID	0xa
#define VLAN_11_ID	0xb
#define VLAN_12_ID	0xc
#define VLAN_13_ID	0xd
#define VLAN_14_ID	0xe
#define VLAN_15_ID	0xf
#define VLAN_16_ID	0x10
#define VLAN_17_ID	0x11
#define VLAN_18_ID	0x12
#define VLAN_19_ID	0x13
#define VLAN_20_ID	0x14
#define VLAN_21_ID	0x15
#define VLAN_22_ID	0x16
#define VLAN_23_ID	0x17
#define VLAN_24_ID	0x18
#define VLAN_25_ID	0x19

#define TOTAL_SPARE_VLAN 16

/* Egress Rate Control register */
#define EGRS_RATE_CTRL	0x9
/* Egress Rate Control 2 register */
#define EGRS_RATE_CTRL2	0xa

#define PORT_STATUS	0x00
#define NO_PHY_DETECT	0x0000

#define PORT_PHY	0x01
#define SCRATCH_MISC	0x1a
#define GP_PIN_CTRL3	0x6b
#define RX_RGMII_TIM	0x8000
#define TX_RGMII_TIM	0x4000
#define FLOW_CTRL_EN	0x0080
#define FLOW_CTRL_FOR	0x0040
#define LINK_VAL	0x0020
#define LINK_FOR	0x0010
#define FULL_DPX	0x0008
#define FULL_DPX_FOR	0x0004
#define NO_SPEED_FOR	0x0003
#define SPEED_1000_FOR	0x0002
#define SPEED_100_FOR	0x0001
#define SPEED_10_FOR	0x0000
#define CLK_125MHZ	0x0007
#define PORT_CTRL	0x04
#define FORWARDING	0x0003
#define EGRS_FLD_ALL	0x000c
#define PORT_DIS	0x0000
#define SPEED_200BASE	0x1000
#define DATA_UPDATE	0x1000

struct mv88e6176_sw_reg {
	u8 port;
	u8 reg;
	u16 value;
};

int mv88e6176_sw_reset(const char *devname, u8 phy_addr);
int mv88e6176_sw_program(const char *devname, u8 phy_addr,
	struct mv88e6176_sw_reg *regs, int regs_nb);

#endif /* __MV88E6176_H */
