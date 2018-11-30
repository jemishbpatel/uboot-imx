/*
 * copyright (c)2016 LiveU - All rights reserved.
 *
 * This software is authored by LiveU and is LiveU'
 *  intelletual property,including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by LiveU.
 *
 * This file may not be distributed, copied or reproduced in any manner,
 * electronic or otherwise, without the written consent of LiveU.
 *
 * This file contains defination of APIs for EEPROM (AT25 : SPI EEPROM) driver.
 */
#include <common.h>
#include <config.h>
#include <command.h>
#include <spi.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <spi_eeprom.h>

#define AT25_WREN       0x06            /* latch the write enable */
#define AT25_RDSR       0x05            /* read status register */
#define AT25_READ       0x03            /* read byte(s) */
#define AT25_WRITE      0x02            /* write byte(s)/sector */

#define SPI_DEFAULT_BUS            (1)
#define SPI_DEFAULT_CHIPSELECT     (0)
#define SPI_DEFAULT_MODE           (0)
#define DEFAULT_EEPROM_OFFSET      (0)
#define SPI_CLOCK 		   (1000000)
#define EEPROM_SIZE                (128)
#define MINIMUM_ARGUMENTS          (3)
#define MAXIMUM_ARGUMENTS          (4)
#define DELAY_WRITE_EANBLE_STABLE  (5000)
#define DELAY_WRITE_STABLE         (50000)
#define ADDRESS_LSB_MASK	   (0x00ff)
#define ADDRESS_MSB_MASK	   (0xff00)

static unsigned int     bus = SPI_DEFAULT_BUS;
static unsigned int     cs = SPI_DEFAULT_CHIPSELECT;
static unsigned int     mode = SPI_DEFAULT_MODE ;
struct spi_slave *slave;

static void inline enable_chip_select(void)
{
	gpio_set_value(ECSPI2_SPI_CHIPSELECT, 0);
}

static void inline disable_chip_select(void)
{
        gpio_set_value(ECSPI2_SPI_CHIPSELECT, 1);
}

static int init_spi_slave(void)
{
	int ret = 0;
        slave = spi_setup_slave(bus, cs, SPI_CLOCK, mode);
        if (!slave) {
                printf("Invalid device %d:%d\n", bus, cs);
                return -EINVAL;
        }
        ret = spi_claim_bus(slave);
        if (ret)
                return ret;
	return ret;
}

static int read_eeprom_byte(struct spi_slave *slave, unsigned short address,
		unsigned char *value)
{
	int ret = 0;
	unsigned char dout[3], byte;
	dout[0] = AT25_READ;
	dout[1] = (address & ADDRESS_MSB_MASK) >> 8;
	dout[2] = address & ADDRESS_LSB_MASK;
	enable_chip_select();
	ret = spi_xfer(slave,  (sizeof(dout) * 8), dout, &byte, 0);
	ret = spi_xfer(slave,  (sizeof(dout) * 8), dout, &byte, 0);
	disable_chip_select();
	*value = byte;
	return ret;
}

static int write_eeprom_byte(struct spi_slave *slave, unsigned short address,
		unsigned char value)
{
	int ret = 0;
	unsigned char dout[4], cmd;
	cmd = AT25_WREN;

	enable_chip_select();
	ret = spi_xfer(slave, 8, &cmd, NULL, 0);
	disable_chip_select();
	udelay(DELAY_WRITE_EANBLE_STABLE);

	enable_chip_select();
	dout[0] = AT25_WRITE;
	dout[1] = (address & ADDRESS_MSB_MASK) >> 8;
	dout[2] = address & ADDRESS_LSB_MASK;
	dout[3] = value;
	ret = spi_xfer(slave, (sizeof(dout) * 8), dout, NULL, 0);
	disable_chip_select();

	return ret;
}

int read_eeprom(unsigned char address, unsigned char *buf, int size)
{
	int i , j, ret = 0;

	ret = init_spi_slave();
        if (ret)
                return ret;

	if((address + size) > (EEPROM_SIZE -1)) {
		printf("Wrong size given!");
		return -1;
	}
	for( i = address, j =0; i < (address + size) ; i++, j++) {
		ret |= read_eeprom_byte(slave, i, &buf[j]);
	}
	return ret;
}

int write_eeprom(unsigned char address, unsigned char *buf, int size)
{
        int i , j, ret = 0;

	ret = init_spi_slave();
        if (ret)
                return ret;

        if((address + size) > (EEPROM_SIZE -1)) {
                printf("Wrong size given!");
                return -1;
        }
        for( i = address, j =0; i < (address + size) ; i++, j++) {
                ret |= write_eeprom_byte(slave, i, buf[j]);
		udelay(DELAY_WRITE_STABLE);
        }
        return ret ;
}

void usage(void)
{
	printf("Usage:\n");
	printf("spi_eeprom r <off>\n");
	printf("spi_eeprom w <off> <value>\n");
}

static int do_eeprom_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{

	int ret;
	unsigned short offset = DEFAULT_EEPROM_OFFSET;
	unsigned char  value;

	if(argc < MINIMUM_ARGUMENTS) {
		usage();
		return -1;
	}

	ret = init_spi_slave();
        if (ret)
                return ret;

	if(MINIMUM_ARGUMENTS == argc) {
		if(0 == strcmp(argv[1], "r")) {
			offset = simple_strtoul(argv[2], NULL, 10);
                        ret = read_eeprom_byte(slave, offset, &value);
                        printf("0x%x\n", value);

		}
	} else if(MAXIMUM_ARGUMENTS == argc) {
		if(0 != strcmp(argv[1], "w")) {
			usage();
			return -1;
		} else {
			offset = simple_strtoul(argv[2], NULL, 10);
			value = simple_strtoul(argv[3], NULL, 16);
			ret = write_eeprom_byte(slave, offset, value);
		}
	} else {
		usage();
		return -1;
	}
	return 0;
}

U_BOOT_CMD(
        spi_eeprom, 4,      1,      do_eeprom_test,
        "SPI EEPROM utility command",
        "spi_eeprom r <off>\n"
        "spi_eeprom w <off> <value>\n"
);

