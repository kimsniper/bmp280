/*
 * Copyright (c) 2022, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "bmp280_spi_hal.h" 

//Hardware Specific Components
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

//SPI User Defines
#define SPI_MASTER_LCD_HOST         HSPI_HOST
#define SPI_MASTER_MISO             25
#define SPI_MASTER_MOSI             23
#define SPI_MASTER_CLK              19
#define SPI_MASTER_CS               22

int16_t bmp280_spi_hal_init()
{
    int16_t err = BMP280_OK;

    //User implementation here

    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=SPI_MASTER_MISO,
        .mosi_io_num=SPI_MASTER_MOSI,
        .sclk_io_num=SPI_MASTER_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=32
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=26*1000*1000,           //Clock out at 26 MHz
        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(SPI_MASTER_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the slave to the SPI bus
    ret=spi_bus_add_device(SPI_MASTER_LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    return err == ret == (ESP_OK) ? BMP280_OK :  BMP280_ERR;
}

int16_t bmp280_spi_hal_read(uint8_t address, uint8_t *reg, uint8_t *data, uint16_t count)
{
    int16_t err = BMP280_OK;

    //User implementation here


    return err == BMP280_OK ? BMP280_OK :  BMP280_ERR;
}

int16_t bmp280_spi_hal_write(uint8_t address, uint8_t *data, uint16_t count)
{
    int16_t err = BMP280_OK;

    //User implementation here


    return err == BMP280_OK ? BMP280_OK :  BMP280_ERR;
}

void bmp280_spi_hal_ms_delay(uint32_t ms) {

    //User implementation here
    
    vTaskDelay(pdMS_TO_TICKS(ms));
}
