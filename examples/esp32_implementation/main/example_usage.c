#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//BMP280 components
#include "bmp280_i2c.h"
#include "bmp280_i2c_hal.h"

static const char *TAG = "example_usage";

void app_main(void)
{
    esp_err_t err;
    uint8_t id = 0;

    bmp280_i2c_hal_init();

    err = bmp280_i2c_reset();
    if(err != ESP_OK) ESP_LOGE(TAG, "Error setting the device!");

    err += bmp280_i2c_read_part_number(&id);
    if(err == ESP_OK){
        ESP_LOGI(TAG, "Part number: 0x%02x", id);
    } 
    else{
        ESP_LOGE(TAG, "Unable to read part number!");
    } 

    if (err == ESP_OK && id == 0x58)
    {
        ESP_LOGI(TAG, "BMP280 initialization successful");
        uint16_t pressure;
        uint8_t temperature;
        while(1)
        {
            //Reading here
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    else{
        ESP_LOGE(TAG, "BMP280 initialization failed!");
    }
}
