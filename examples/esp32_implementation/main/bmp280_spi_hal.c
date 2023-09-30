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

#ifdef CONFIG_IDF_TARGET_ESP32
#  ifdef CONFIG_EXAMPLE_USE_SPI1_PINS
#    define EEPROM_HOST    SPI1_HOST
// Use default pins, same as the flash chip.
#    define PIN_NUM_MISO 7
#    define PIN_NUM_MOSI 8
#    define PIN_NUM_CLK  6
#  else
#    define EEPROM_HOST    HSPI_HOST
#    define PIN_NUM_MISO 18
#    define PIN_NUM_MOSI 23
#    define PIN_NUM_CLK  19
#  endif

#  define PIN_NUM_CS   13
#elif defined CONFIG_IDF_TARGET_ESP32S2
#  define EEPROM_HOST    SPI2_HOST

#  define PIN_NUM_MISO 37
#  define PIN_NUM_MOSI 35
#  define PIN_NUM_CLK  36
#  define PIN_NUM_CS   34
#elif defined CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C6
#  define EEPROM_HOST    SPI2_HOST

#  define PIN_NUM_MISO 2
#  define PIN_NUM_MOSI 7
#  define PIN_NUM_CLK  6
#  define PIN_NUM_CS   10

#elif CONFIG_IDF_TARGET_ESP32S3
#  define EEPROM_HOST    SPI2_HOST

#  define PIN_NUM_MISO 13
#  define PIN_NUM_MOSI 11
#  define PIN_NUM_CLK  12
#  define PIN_NUM_CS   10

#elif CONFIG_IDF_TARGET_ESP32H2
#  define EEPROM_HOST    SPI2_HOST

#  define PIN_NUM_MISO 0
#  define PIN_NUM_MOSI 5
#  define PIN_NUM_CLK  4
#  define PIN_NUM_CS   1
#endif

/* BMP280 specific macros */
#define SPI_READ_MASK              	((uint8_t)0x80U)
#define SPI_WRITE_MASK              ((uint8_t)0x7FU)

int16_t bmp280_spi_hal_init()
{
    int16_t err = BMP280_OK;

    //User implementation here

	esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num = SPI_MASTER_MISO,
        .mosi_io_num = SPI_MASTER_MOSI,
        .sclk_io_num = SPI_MASTER_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    //Initialize the SPI bus
    ret = spi_bus_initialize(SPI_MASTER_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

	if(ret != ESP_OK)
    {
		err = BMP280_ERR;
	}

    return err;
}

int16_t bmp280_spi_hal_read(uint8_t reg, uint8_t *data, uint16_t count)
{
    int16_t err = BMP280_OK;

    //User implementation here

    /* For BMP280, bit 7 of the register data will WR bit */
    uint8_t reg_to_write = (reg & SPI_WRITE_MASK);


    return err;
}

int16_t bmp280_spi_hal_write(uint8_t *data, uint16_t count)
{
    int16_t err = BMP280_OK;

    //User implementation here
    data[0] &= SPI_WRITE_MASK;


    return err;
}

void bmp280_spi_hal_ms_delay(uint32_t ms) {

    //User implementation here
}
