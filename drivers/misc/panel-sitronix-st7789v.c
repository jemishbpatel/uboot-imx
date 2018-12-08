#include <common.h>
#include <config.h>
#include <command.h>
#include <spi.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <panel-sitronix-st7789v.h>

#define LCD_SPI_BUS		0
#define LCD_SPI_CHIPSELECT	0
#define LCD_SPI_MODE		0
#define SPI_CLOCK		1000000

enum st7789v_prefix {
	ST7789V_COMMAND = 0,
	ST7789V_DATA = 1,
};

static void inline enable_chip_select(void)
{
        gpio_set_value(ECSPI1_SPI_CHIPSELECT_PANEL_ST7789V, 0);
}

static void inline disable_chip_select(void)
{
        gpio_set_value(ECSPI1_SPI_CHIPSELECT_PANEL_ST7789V, 1);
}

static int st7789v_spi_write(struct spi_slave *slave, enum st7789v_prefix prefix, u8 data)
{
	int ret = 0;
	u16 txbuf = ((prefix & 1) << 8) | data;

	ret = spi_xfer_nbit(slave, 9 , &txbuf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret)
		printf("failed to write st7789v\n");
	return ret;
}

static int st7789v_command(struct spi_slave *slave, u8 cmd)
{
	return st7789v_spi_write(slave, ST7789V_COMMAND, cmd);
}

static int st7789v_data(struct spi_slave *slave, u8 cmd)
{
	return st7789v_spi_write(slave, ST7789V_DATA, cmd);
}

struct spi_slave * init_spi_slave(unsigned int bus, unsigned int cs, unsigned mode)
{
	struct spi_slave *slave;
        int ret = 0;

        slave = spi_setup_slave(bus, cs, SPI_CLOCK, mode);
        if (!slave) {
                printf("Invalid device %d:%d\n", bus, cs);
                return NULL;
        }
        ret = spi_claim_bus(slave);
        if (ret)
                return NULL;

        return slave;
}

void configure_panel_st7789v(void)
{
	struct spi_slave *slave = init_spi_slave(LCD_SPI_BUS, LCD_SPI_CHIPSELECT, LCD_SPI_MODE);

	st7789v_command(slave, 0x11);
	mdelay(120);
	st7789v_command(slave, 0x36);
	st7789v_data(slave, 0x00);

	/* pixel format */
	st7789v_command(slave, 0x3a);
	st7789v_data(slave, 0x66);

	/* porch control */
	st7789v_command(slave, 0xb2);
	st7789v_data(slave, 0xc);
	st7789v_data(slave, 0xc);
	st7789v_data(slave, 0x0);
	st7789v_data(slave, 0x33);
	st7789v_data(slave, 0x33);

	/* GCTRL */
	st7789v_command(slave, 0xb7);
	st7789v_data(slave, 0x35);

	/* VCOMS */
	st7789v_command(slave, 0xbb);
	st7789v_data(slave, 0x2b);

	/* LCMCTRL */
	st7789v_command(slave, 0xc0);
	st7789v_data(slave, 0x2c);

	/* VDVVRHEN */
	st7789v_command(slave, 0xc2);
	st7789v_data(slave, 0x1);

	/* VHRS */
	st7789v_command(slave, 0xc3);
	st7789v_data(slave, 0xf);

	/* VDVS */
	st7789v_command(slave, 0xc4);
	st7789v_data(slave, 0x20);

	/* FRCCTRL 2*/
	st7789v_command(slave, 0xc6);
	st7789v_data(slave, 0xf);

	/* PWCCTRL1 */
	st7789v_command(slave, 0xd0);
	st7789v_data(slave, 0xa4);
	st7789v_data(slave, 0xa1);

	/* PVGAMCTRL */
	st7789v_command(slave, 0xe0);
	st7789v_data(slave, 0xd0);
	st7789v_data(slave, 0xa);
	st7789v_data(slave, 0xe);
	st7789v_data(slave, 0x8);
	st7789v_data(slave, 0x9);
	st7789v_data(slave, 0x7);
	st7789v_data(slave, 0x2d);
	st7789v_data(slave, 0x33);
	st7789v_data(slave, 0x3d);
	st7789v_data(slave, 0x34);
	st7789v_data(slave, 0xa);
	st7789v_data(slave, 0xa);
	st7789v_data(slave, 0x1b);
	st7789v_data(slave, 0x28);

	/* NVGAMCTRL */
	st7789v_command(slave, 0xe1);
	st7789v_data(slave, 0xd0);
	st7789v_data(slave, 0xa);
	st7789v_data(slave, 0xf);
	st7789v_data(slave, 0x8);
	st7789v_data(slave, 0x8);
	st7789v_data(slave, 0x7);
	st7789v_data(slave, 0x2e);
	st7789v_data(slave, 0x54);
	st7789v_data(slave, 0x40);
	st7789v_data(slave, 0x34);
	st7789v_data(slave, 0x9);
	st7789v_data(slave, 0xb);
	st7789v_data(slave, 0x1b);
	st7789v_data(slave, 0x28);

	/* Enter invert mode */
	st7789v_command(slave, 0x21);

	/* RAMCTRL */
	st7789v_command(slave, 0xb0);
	st7789v_data(slave, 0x11);
	st7789v_data(slave, 0xF0);

	/* RGBCTRL */
	st7789v_command(slave, 0xb1);
	st7789v_data(slave, 0xcc);
	st7789v_data(slave, 0x8);
	st7789v_data(slave, 0x14);

	/* display on */
	st7789v_command(slave, 0x29);
}
