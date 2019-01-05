#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "bme280_hal.h"

#include <u8g2.h>
#include "u8g2_hal.h"


static const char *TAG = "App_Main";

// SDA - GPIO21
// SCL - GPIO22
#define PIN_SDA 21
#define PIN_SCL 22

//Wifi
#define WIFI_SSID "acer"
#define WIFI_PSK  "1234567890123"


static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

static int BME280_Delay = 10;



esp_err_t event_handler(void *ctx, system_event_t *event)
{
	return ESP_OK;
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id) {
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);

		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
		break;
	default:
		break;
	}
	return ESP_OK;
}


static void wifi_init(void)
{
	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	wifi_config_t wifi_config = {
			.sta = {
					.ssid = WIFI_SSID,
					.password = WIFI_PSK,
			},
	};
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_LOGI(TAG, "start the WIFI SSID:[%s]", WIFI_SSID);
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_LOGI(TAG, "Waiting for wifi");
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
}

static void i2c_init()
{
	i2c_config_t i2c_config;

	i2c_config.mode = I2C_MODE_MASTER;
	i2c_config.sda_io_num = PIN_SDA;
	i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
	i2c_config.scl_io_num = PIN_SCL;
	i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
	i2c_config.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &i2c_config));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, i2c_config.mode, 0, 0, 0));
}

void BME280_task(void *pvParameter)
{
	bme280_buffer = xQueueCreate(1, sizeof(struct sensor_t));
	bme280_hal_init();
	while (1)
	{
		stream_sensor_data_forced_mode();
		vTaskDelay((BME280_Delay*1000) / portTICK_RATE_MS);
	}

}

void oled_task(void *ignore)
{
	u8g2_t u8g2;
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(
			&u8g2,
			U8G2_R0,
			u8g2_i2c_byte_cb,
			u8g2_gpio_and_delay_cb);
	u8x8_SetI2CAddress(&u8g2.u8x8,0x78);

	u8g2_InitDisplay(&u8g2);

	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	u8g2_ClearBuffer(&u8g2);

	u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
	u8g2_DrawStr(&u8g2, 2,17,"Arrancamos!");
	u8g2_SendBuffer(&u8g2);
	vTaskDelay(2000 / portTICK_RATE_MS);
	u8g2_ClearBuffer(&u8g2);
	u8g2_ClearDisplay(&u8g2);

	struct sensor_t datos;

	while(1){
		xQueueReceive(bme280_buffer, &datos, portMAX_DELAY);
		char buf[9];
		u8g2_ClearBuffer(&u8g2);
		sprintf(buf, "%.1fºC - %.1f%%", datos.temp, datos.hum );
		u8g2_DrawStr(&u8g2, 4,17, buf);
		u8g2_SendBuffer(&u8g2);
	}

	vTaskDelete(NULL);
}

void app_main(void)
{
	ESP_LOGI(TAG, "[APP] Startup..");
	ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
	ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

	esp_log_level_set("*", ESP_LOG_INFO);
	nvs_flash_init();
	wifi_init();

	i2c_init();
	xTaskCreatePinnedToCore(&BME280_task, "BME280_task", 768*3, NULL, 2, NULL, 1);
	xTaskCreatePinnedToCore(&oled_task, "Oled_task", 768*3, NULL, 2, NULL, 1);
}

