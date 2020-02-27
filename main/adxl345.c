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

#include <driver/i2c.h>
#include <esp_log.h>

#include "adxl345.h"

static const char* TAG = "CANdueADXL345";

void adxl345_write_register(enum ADXL_REGMAP reg, unsigned char value)
{
	esp_err_t ret;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ret = i2c_master_start(cmd);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_WRITE, true);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_write_byte(cmd, reg, true);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_write_byte(cmd, value, true);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_stop(cmd);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
	ESP_ERROR_CHECK(ret);
	i2c_cmd_link_delete(cmd);
}


static unsigned char adxl345_multiread(i2c_cmd_handle_t cmd, bool ack)
{
	esp_err_t ret;
	unsigned char data = 0;

	ret = i2c_master_read_byte(cmd, &data, ack);
	ESP_ERROR_CHECK(ret);

	return data;
}

void adxl345_read_axes(struct adxl345_data *out)
{
	esp_err_t ret;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ret = i2c_master_start(cmd);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_WRITE, true);
	ESP_ERROR_CHECK(ret);

	ret = i2c_master_write_byte(cmd, ADXL345_REG_DATAX0, true);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_start(cmd);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_READ, true);
	ESP_ERROR_CHECK(ret);

	out->x  = adxl345_multiread(cmd, true);
	out->x |= adxl345_multiread(cmd, true) << 8;

	out->y  = adxl345_multiread(cmd, true);
	out->y |= adxl345_multiread(cmd, true) << 8;

	out->z  = adxl345_multiread(cmd, true);
	out->z |= adxl345_multiread(cmd, false) << 8;

	ret = i2c_master_stop(cmd);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
	ESP_ERROR_CHECK(ret);
	i2c_cmd_link_delete(cmd);
}

unsigned char adxl345_read_register(enum ADXL_REGMAP reg)
{
	esp_err_t ret;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	unsigned char data = 0;
	ret = i2c_master_start(cmd);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_WRITE, true);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_write_byte(cmd, reg, true);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_start(cmd);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_READ, true);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_read_byte(cmd, &data, true);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_stop(cmd);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
	ESP_ERROR_CHECK(ret);
	i2c_cmd_link_delete(cmd);

	return data;
}

void adxl345_setup_i2c(void)
{
	esp_err_t ret;

	/* Set up the I2C master */
	i2c_config_t i2c;
	i2c.mode = I2C_MODE_MASTER;
	i2c.scl_io_num =GPIO_NUM_22;
	i2c.sda_io_num = GPIO_NUM_21;
	i2c.scl_pullup_en = GPIO_PULLUP_ENABLE;
	i2c.sda_pullup_en = GPIO_PULLUP_ENABLE;
	i2c.master.clk_speed = 100000; /* 100 kHz */
	ret = i2c_param_config(I2C_NUM_0, &i2c);
	ESP_ERROR_CHECK(ret);
	ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
	ESP_ERROR_CHECK(ret);

	/* Check that we have an accelerometer installed */
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	unsigned char data = 0;
	ret = i2c_master_start(cmd);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_WRITE, true);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_write_byte(cmd, ADXL345_REG_DEVID, true);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_start(cmd);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_READ, true);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_read_byte(cmd, &data, true);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_stop(cmd);
	ESP_ERROR_CHECK(ret);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
	ESP_ERROR_CHECK(ret);
	i2c_cmd_link_delete(cmd);

	if (data != ADXL345_DEVID)
	{
		ESP_LOGE(TAG, "ERROR: ADXL345 not found");
		while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
	}

	adxl345_write_register(ADXL345_REG_POWER_CTL, 8); // Enable measurements
}
