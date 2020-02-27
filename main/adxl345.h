/*
 * Copyright (C) 2020 Rebecca Cran <rebecca@bsdio.com>.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ADXL345_H_
#define ADXL345_H_

#define ADXL345_ADDR 0x53
#define ADXL345_DEVID 0xE5

enum ADXL_REGMAP
{
	ADXL345_REG_DEVID = 0,
	ADXL345_REG_THRESH_TAP = 29,
	ADXL345_REG_OFSX = 30,
	ADXL345_REG_OFSY = 31,
	ADXL345_REG_OFSZ = 32,
	ADXL345_REG_DUR = 33,
	ADXL345_REG_LATENT = 34,
	ADXL345_REG_WINDOW = 35,
	ADXL345_REG_THRESH_ACT = 36,
	ADXL345_REG_THRESH_INACT = 37,
	ADXL345_REG_TIME_INACT = 38,
	ADXL345_REG_ACT_INACT_CTL = 39,
	ADXL345_REG_THRESH_FF = 40,
	ADXL345_REG_TIME_FF = 41,
	ADXL345_REG_TAP_AXES = 42,
	ADXL345_REG_ACT_TAP_STATUS = 43,
	ADXL345_REG_BW_RATE = 44,
	ADXL345_REG_POWER_CTL = 45,
	ADXL345_REG_INT_ENABLE = 46,
	ADXL345_REG_INT_MAP = 47,
	ADXL345_REG_INT_SOURCE = 48,
	ADXL345_REG_DATA_FORMAT = 49,
	ADXL345_REG_DATAX0 = 50,
	ADXL345_REG_DATAX1 = 51,
	ADXL345_REG_DATAY0 = 52,
	ADXL345_REG_DATAY1 = 53,
	ADXL345_REG_DATAZ0 = 54,
	ADXL345_REG_DATAZ1 = 55,
	ADXL345_REG_FIFO_CTL = 56,
	ADXL345_REG_FIFO_STATUS = 57
};

struct adxl345_data
{
	unsigned int x;
	unsigned int y;
	unsigned int z;
};


void adxl345_write_register(enum ADXL_REGMAP reg, unsigned char value);
void adxl345_read_axes(struct adxl345_data *out);
unsigned char adxl345_read_register(enum ADXL_REGMAP reg);
void adxl345_setup_i2c(void);


#endif /* ADXL345_H_ */
