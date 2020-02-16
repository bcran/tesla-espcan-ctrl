#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_bt.h"
#include "esp_system.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/can.h"
#include "driver/adc.h"
#include "driver/i2c.h"

#include "model3_can.h"

#include "esp_log.h"

static const char* TAG = "CANDueMain";

uint32_t bthandle = 0;

static esp_bd_addr_t photon_btpeer = {0x00, 0x22, 0xec, 0x05, 0x9d, 0xe5}; /* Photon */
static esp_bd_addr_t imx6_btpeer = {0x00, 0x22, 0xEC, 0x05, 0x98, 0xD0};


void process_can_msg(can_message_t *msg);

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void sppcb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
	esp_err_t ret;

	switch (event)
	{
	case ESP_SPP_INIT_EVT:
		ret = esp_bt_dev_set_device_name("ESP32CANDue");
				ESP_ERROR_CHECK(ret);
		ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
		ESP_ERROR_CHECK(ret);
		ret = esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_MASTER, 0, "ESP32CANDueSPP");
		ESP_ERROR_CHECK(ret);
		ESP_LOGI(TAG, "Initialized Bluetooth\n");
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
		ESP_LOGI(TAG, "SPP CLOSE");
		bthandle = 0;
		break;
	case ESP_SPP_START_EVT:
		ESP_LOGI(TAG, "SPP START");
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
		ESP_LOGI(TAG, "OPEN SRV EVT");
		break;
	default:
		ESP_LOGI(TAG, "Unhandled SPP event %d", event);
		break;
	}

}

void gapcb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
	switch (event) {
	case ESP_BT_GAP_AUTH_CMPL_EVT:
		if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
		{
			ESP_LOGI(TAG, "Authentication successful: %s\n", param->auth_cmpl.device_name);
			esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
			if (memcmp(param->auth_cmpl.bda, photon_btpeer, sizeof(param->auth_cmpl.bda)) != 0 &&
				memcmp(param->auth_cmpl.bda, imx6_btpeer, sizeof(param->auth_cmpl.bda)) != 0) {
				ESP_LOGE(TAG, "Unauthorized Bluetooth Peer attempted to connect");
				esp_bt_gap_remove_bond_device(param->auth_cmpl.bda);
				esp_bt_controller_deinit();
				esp_restart();
			}

		} else {
			ESP_LOGI(TAG, "Authentication failed: %d\n", param->auth_cmpl.stat);
		}

		/* Set GPIO0 low, to enable bootloader mode on reset */
//		gpio_set_level(0, 0);
//		esp_restart();
		break;

	case ESP_BT_GAP_PIN_REQ_EVT:
		ESP_LOGI(TAG, "PIN REQ EVT\n");
		break;
	case ESP_BT_GAP_CFM_REQ_EVT:
		ESP_LOGI(TAG, "Please compare PIN code %d\n", param->cfm_req.num_val);
		esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
		break;
	case ESP_BT_GAP_KEY_NOTIF_EVT:
		ESP_LOGI(TAG, "GAP Key: %d\n", param->key_notif.passkey);
		break;
	case ESP_BT_GAP_KEY_REQ_EVT:
		ESP_LOGI(TAG, "Passkey requested\n");
		break;
	case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
		break;
	default:
		ESP_LOGI(TAG, "Unknown GAP callback event %d\n", event);
	}
}

void i2c_intr(void *param)
{

}

#define ADXL345_ADDR 0x53
#define ADXL345_DEVID 0xE5

void app_main(void)
{
	esp_err_t ret;
#if 0
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
#endif

	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NOT_FOUND) {
		ESP_ERROR_CHECK(ret);
	}

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

	esp_bt_controller_config_t btcfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&btcfg);
	ESP_ERROR_CHECK(ret);

	ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
	ESP_ERROR_CHECK(ret);
	ret = esp_bluedroid_init();
	ESP_ERROR_CHECK(ret);
	ret = esp_bluedroid_enable();
	ESP_ERROR_CHECK(ret);
	ret = esp_bt_gap_register_callback(gapcb);
	ESP_ERROR_CHECK(ret);
	ret = esp_spp_register_callback(sppcb);
	ESP_ERROR_CHECK(ret);
	ret = esp_spp_init(ESP_SPP_MODE_CB);
	ESP_ERROR_CHECK(ret);
	esp_bt_io_cap_t cap = ESP_BT_IO_CAP_IO;
	ret = esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &cap, sizeof(uint8_t));

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
		printf("ERROR: ADXL345 not found\n");
		while (1) { sleep(60); }
	}

	printf("Initialized. Running program.\n");

	can_message_t msg;
	while (1) {
		ret = can_receive(&msg, pdMS_TO_TICKS(10000));

		if (ret == ESP_OK) {
			process_can_msg(&msg);
		}
		else if (ret == ESP_ERR_TIMEOUT)
			printf("rx tmo\n");
		else if (ret == ESP_ERR_INVALID_ARG)
			printf("rx arg\n");
		else if (ret == ESP_ERR_INVALID_STATE)
			printf("CAN driver na\n");
		else
			printf("CAN err %d\n", ret);
	}

}

struct msgheader
{
	uint32_t identifier;
	uint32_t size;
};

void process_can_msg(can_message_t *msg)
{
	struct model3_can_id257_u_ispeed_t speed;
	struct model3_can_id528_unix_time_t time;
	struct model3_can_id2_d2_bmsv_alimits_t bmsv;
	struct model3_can_id3_d8_elevation_t elevation;
	struct model3_can_id352_bm_senergy_t kwh;
	struct model3_can_id118_drive_system_status_t drivesystem;

	struct msgheader hdr;
	hdr.identifier = msg->identifier;

	int ret;
	switch (msg->identifier)
	{
	case MODEL3_CAN_ID257_U_ISPEED_FRAME_ID:
		ret = model3_can_id257_u_ispeed_unpack(&speed, msg->data, msg->data_length_code);
		hdr.size = sizeof(hdr) + sizeof(speed);
		esp_spp_write(bthandle, sizeof(hdr), (uint8_t*)&hdr);
		esp_spp_write(bthandle, sizeof(speed), (uint8_t*)&speed);
		break;
	case MODEL3_CAN_ID528_UNIX_TIME_FRAME_ID:
		ret = model3_can_id528_unix_time_unpack(&time, msg->data, msg->data_length_code);
		hdr.size = sizeof(hdr) + sizeof(time);
		esp_spp_write(bthandle, sizeof(hdr), (uint8_t*)&hdr);
		esp_spp_write(bthandle, sizeof(time), (uint8_t*)&time);
		break;
	case MODEL3_CAN_ID2_D2_BMSV_ALIMITS_FRAME_ID:
		ret = model3_can_id2_d2_bmsv_alimits_unpack(&bmsv, msg->data, msg->data_length_code);
		hdr.size = sizeof(hdr) + sizeof(bmsv);
		esp_spp_write(bthandle, sizeof(hdr), (uint8_t*)&hdr);
		esp_spp_write(bthandle, sizeof(bmsv), (uint8_t*)&bmsv);
		break;
	case MODEL3_CAN_ID3_D8_ELEVATION_FRAME_ID:
		ret = model3_can_id3_d8_elevation_unpack(&elevation, msg->data, msg->data_length_code);
		hdr.size = sizeof(hdr) + sizeof(elevation);
		esp_spp_write(bthandle, sizeof(hdr), (uint8_t*)&hdr);
		esp_spp_write(bthandle, sizeof(elevation), (uint8_t*)&elevation);
		break;
	case MODEL3_CAN_ID352_BM_SENERGY_FRAME_ID:
		ret = model3_can_id352_bm_senergy_unpack(&kwh, msg->data, msg->data_length_code);
		hdr.size = sizeof(hdr) + sizeof(kwh);
		esp_spp_write(bthandle, sizeof(hdr), (uint8_t*)&hdr);
		esp_spp_write(bthandle, sizeof(kwh), (uint8_t*)&kwh);
		break;
	case MODEL3_CAN_ID118_DRIVE_SYSTEM_STATUS_FRAME_ID:
		ret = model3_can_id118_drive_system_status_unpack(&drivesystem, msg->data, msg->data_length_code);
		hdr.size = sizeof(hdr) + sizeof(drivesystem);
		esp_spp_write(bthandle, sizeof(hdr), (uint8_t*)&hdr);
		esp_spp_write(bthandle, sizeof(drivesystem), (uint8_t*)&drivesystem);
	}

	float temperature = (((adc1_get_raw(ADC1_CHANNEL_5) / 4095) * 2000) - 500) * 0.1;
	(void)temperature;
}
