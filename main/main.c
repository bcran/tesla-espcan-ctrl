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

#include <string.h>
#include <unistd.h>

#include <esp_bt.h>
#include <esp_system.h>
#include <esp_gap_bt_api.h>
#include <esp_spp_api.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <driver/can.h>
#include <driver/adc.h>
#include <driver/i2c.h>
#include <esp_log.h>

#include "model3_can.h"

static const char* TAG = "CANdue";

uint32_t bthandle = 0;

#define ADXL345_ADDR 0x53
#define ADXL345_DEVID 0xE5

struct msgheader
{
	uint32_t identifier;
	uint32_t size;
};


static esp_bd_addr_t photon_btpeer = {0x00, 0x22, 0xec, 0x05, 0x9d, 0xe5}; // Photon desktop
static esp_bd_addr_t imx6_btpeer   = {0x00, 0x22, 0xEC, 0x05, 0x98, 0xD0}; // i.MX6 Sololite EVK

static void verify_bluetooth_peer(esp_bd_addr_t bda);
static void candue_bt_spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
static void candue_bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void candue_setup_bluetooth(void);
static void candue_setup_i2c(void);
static void send_msg_to_btpeer(int id, int len, void *data);
static void process_can_msg(can_message_t *msg);


// Only allow specific devices to connect to the ESP32 board
static void verify_bluetooth_peer(esp_bd_addr_t bda)
{
	if (memcmp(bda, photon_btpeer, sizeof(esp_bd_addr_t)) != 0 &&
		memcmp(bda, imx6_btpeer, sizeof(esp_bd_addr_t)) != 0) {
		ESP_LOGE(TAG, "Unauthorized Bluetooth® peer attempted to connect");
		esp_bt_gap_remove_bond_device(bda);
		esp_bt_controller_deinit();
		esp_restart();
	}
}

static void candue_bt_spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
	esp_err_t ret;
	const uint8_t *bda;

	switch (event)
	{
	case ESP_SPP_INIT_EVT:
		ret = esp_bt_dev_set_device_name("ESP32CANDue");
				ESP_ERROR_CHECK(ret);
		ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
		ESP_ERROR_CHECK(ret);
		ret = esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_MASTER, 0, "ESP32CANDueSPP");
		ESP_ERROR_CHECK(ret);
		ESP_LOGI(TAG, "Initialized Bluetooth®");
		bda = esp_bt_dev_get_address();
		ESP_LOGI(TAG, "Controller address is %02x:%02x:%02x:%02x:%02x:%02x", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
		break;
	case ESP_SPP_DISCOVERY_COMP_EVT:
		ESP_LOGI(TAG, "SPP Discovery complete");
		break;
	case ESP_SPP_OPEN_EVT:
		// Occurs after the start event
		ESP_LOGI(TAG, "SPP OPEN EVT");
		bthandle = param->open.handle;
		break;
	case ESP_SPP_CLOSE_EVT:
		ESP_LOGI(TAG, "Bluetooth® peer disconnected");
		bthandle = 0;
		break;
	case ESP_SPP_START_EVT:
		break;
	case ESP_SPP_CL_INIT_EVT:
		ESP_LOGI(TAG, "SPP CL INIT");
		break;
	case ESP_SPP_DATA_IND_EVT:
		ESP_LOGI(TAG, "GOT SPP DATA");
		break;
	// SPP connection congested state change
	case ESP_SPP_CONG_EVT:
		ESP_LOGI(TAG, "CONG");
		break;
	case ESP_SPP_WRITE_EVT:
		ESP_LOGI(TAG, "WRITE");
		break;
	case ESP_SPP_SRV_OPEN_EVT:
		ESP_LOGI(TAG, "Bluetooth® peer connected");
		verify_bluetooth_peer(param->srv_open.rem_bda);
		bthandle = param->srv_open.handle;
		break;
	default:
		ESP_LOGW(TAG, "Unhandled SPP event %d", event);
		break;
	}

}

static void candue_bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
	switch (event) {
		case ESP_BT_GAP_AUTH_CMPL_EVT:
			if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
			{
				verify_bluetooth_peer(param->auth_cmpl.bda);
			} else {
				ESP_LOGW(TAG, "Authentication failed: %d\n", param->auth_cmpl.stat);
			}

			/* Set GPIO0 low, to enable bootloader mode on reset
				gpio_set_level(0, 0);
				esp_restart();
			*/
			break;

		case ESP_BT_GAP_PIN_REQ_EVT:
		case ESP_BT_GAP_CFM_REQ_EVT:
		case ESP_BT_GAP_KEY_NOTIF_EVT:
		case ESP_BT_GAP_KEY_REQ_EVT:
		case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
			break;
		default:
			ESP_LOGW(TAG, "Unknown GAP callback event %d", event);
	}
}

static void candue_setup_bluetooth(void)
{
	esp_err_t ret;

	// Set up the Bluetooth stack
	esp_bt_controller_config_t btcfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&btcfg);
	ESP_ERROR_CHECK(ret);

	ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
	ESP_ERROR_CHECK(ret);
	ret = esp_bluedroid_init();
	ESP_ERROR_CHECK(ret);
	ret = esp_bluedroid_enable();
	ESP_ERROR_CHECK(ret);
	ret = esp_bt_gap_register_callback(candue_bt_gap_callback);
	ESP_ERROR_CHECK(ret);
	ret = esp_spp_register_callback(candue_bt_spp_callback);
	ESP_ERROR_CHECK(ret);
	ret = esp_spp_init(ESP_SPP_MODE_CB);
	ESP_ERROR_CHECK(ret);

	// Configure authentication for no input no output
	esp_bt_io_cap_t mode = ESP_BT_IO_CAP_NONE;
	esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &mode, sizeof(mode));

}

static void candue_setup_i2c(void)
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
	ret = i2c_master_write_byte(cmd, 0, true);
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
		while (1) { sleep(60); }
	}
}

void app_main(void)
{
	esp_err_t ret;

	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NOT_FOUND) {
		ESP_ERROR_CHECK(ret);
	}

	candue_setup_bluetooth();

	// Configure the CAN transceiver
	// Tx is GPIO 17 and Rx GPIO 18 on the ESP32 CANDue V2
	can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(GPIO_NUM_17, GPIO_NUM_16, CAN_MODE_LISTEN_ONLY);
	can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
	can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

	ret = can_driver_install(&g_config, &t_config, &f_config);
	ESP_ERROR_CHECK(ret);
	ret = can_start();
	ESP_ERROR_CHECK(ret);

	// ADC1 CH5 is connected to the ESP32 CANDue ADC0 input, which contains a Analog Devices TMP36 temperature sensor
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_6);

	candue_setup_i2c();

	ESP_LOGI(TAG, "Initialized. Running.");

	can_message_t msg;
	while (1) {
		ret = can_receive(&msg, pdMS_TO_TICKS(10000));

		if (ret == ESP_OK) {
			process_can_msg(&msg);
		}
		else if (ret == ESP_ERR_TIMEOUT)
			continue;
		else if (ret == ESP_ERR_INVALID_ARG)
			ESP_LOGE(TAG, "CAN: invalid argument");
		else if (ret == ESP_ERR_INVALID_STATE)
			ESP_LOGE(TAG, "CAN: invalid state");
		else
			ESP_LOGE(TAG, "CAN: err %d", ret);
	}
}

static void send_msg_to_btpeer(int id, int len, void *data)
{
	esp_err_t ret;
	struct msgheader hdr;
	hdr.identifier = id;
	hdr.size = sizeof(hdr) + len;

	ret = esp_spp_write(bthandle, sizeof(hdr), (uint8_t*)&hdr);
	ESP_ERROR_CHECK(ret);
	esp_spp_write(bthandle, len, (uint8_t*)data);
}

static void process_can_msg(can_message_t *msg)
{
	struct model3_can_id257_u_ispeed_t speed;
	struct model3_can_id528_unix_time_t time;
	struct model3_can_id2_d2_bmsv_alimits_t bmsv;
	struct model3_can_id3_d8_elevation_t elevation;
	struct model3_can_id352_bm_senergy_t kwh;
	struct model3_can_id118_drive_system_status_t drivesystem;

	int ret = true;
	switch (msg->identifier)
	{
	case MODEL3_CAN_ID257_U_ISPEED_FRAME_ID:
		ret = model3_can_id257_u_ispeed_unpack(&speed, msg->data, msg->data_length_code);
		if (ret)
			send_msg_to_btpeer(msg->identifier, sizeof(speed), &speed);
		break;
	case MODEL3_CAN_ID528_UNIX_TIME_FRAME_ID:
		ret = model3_can_id528_unix_time_unpack(&time, msg->data, msg->data_length_code);
		if (ret)
			send_msg_to_btpeer(msg->identifier, sizeof(time), &time);
		break;
	case MODEL3_CAN_ID2_D2_BMSV_ALIMITS_FRAME_ID:
		ret = model3_can_id2_d2_bmsv_alimits_unpack(&bmsv, msg->data, msg->data_length_code);
		if (ret)
			send_msg_to_btpeer(msg->identifier, sizeof(bmsv), &bmsv);
		break;
	case MODEL3_CAN_ID3_D8_ELEVATION_FRAME_ID:
		ret = model3_can_id3_d8_elevation_unpack(&elevation, msg->data, msg->data_length_code);
		if (ret)
			send_msg_to_btpeer(msg->identifier, sizeof(elevation), &elevation);
		break;
	case MODEL3_CAN_ID352_BM_SENERGY_FRAME_ID:
		ret = model3_can_id352_bm_senergy_unpack(&kwh, msg->data, msg->data_length_code);
		if (ret)
			send_msg_to_btpeer(msg->identifier, sizeof(kwh), &kwh);
		break;
	case MODEL3_CAN_ID118_DRIVE_SYSTEM_STATUS_FRAME_ID:
		ret = model3_can_id118_drive_system_status_unpack(&drivesystem, msg->data, msg->data_length_code);
		if (ret)
			send_msg_to_btpeer(msg->identifier, sizeof(drivesystem), &drivesystem);
	}

	if (!ret)
		ESP_LOGE(TAG, "error occurred unpacking CAN data");

	float temperature = (((adc1_get_raw(ADC1_CHANNEL_5) / 4095) * 2000) - 500) * 0.1;
	(void)temperature;
}
