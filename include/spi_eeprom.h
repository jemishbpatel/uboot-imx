/*
 * Copyright (c)2016 LiveU - All rights reserved.
 *
 * This software is authored by LiveU and is LiveU's
 * intelletual property,including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by LiveU.
 *
 * This file may not be distributed, copied or reproduced in any manner,
 * electronic or otherwise, without the written consent of LiveU.
 *
 * This file contains declaration of APIs used in PCA9555 (IO expander chip from
 * Texas Instruments) driver from user space.
 */
#ifndef _SPI_EEPROM_H
#define _SPI_EEPROM_H

#define ECSPI2_SPI_CHIPSELECT   (58)
#define EEPROM_WRITE_PROTECT    (90)

int write_eeprom(unsigned char address, unsigned char *buf, int size);
int read_eeprom(unsigned char address, unsigned char *buf, int size);

#endif
/*
 * Copyright (c)2016 LiveU - All rights reserved.
 *
 * This software is authored by LiveU and is LiveU'
 * intelletual property,including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by LiveU.
 *
 * This file may not be distributed, copied or reproduced in any manner,
 * electronic or otherwise, without the written consent of LiveU.
 */

