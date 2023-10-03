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
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/task.h"

//SPI User Defines
#define SPI_MASTER_MISO             19
#define SPI_MASTER_MOSI             23
#define SPI_MASTER_CLK              18
#define SPI_MASTER_CS               5

/* BMP280 specific macros */
#define SPI_WR_MASK              	((uint8_t)0x80U)

spi_device_handle_t handle;

int16_t bmp280_spi_hal_init()
{
    int16_t err = BMP280_OK;

    //User implementation here

	esp_err_t ret;

    gpio_reset_pin( SPI_MASTER_CS );
	gpio_set_direction( SPI_MASTER_CS, GPIO_MODE_OUTPUT );
	gpio_set_level( SPI_MASTER_CS, 0 );

    spi_bus_config_t buscfg={
        .miso_io_num = SPI_MASTER_MISO,
        .mosi_io_num = SPI_MASTER_MOSI,
        .sclk_io_num = SPI_MASTER_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    //Initialize the SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    
    spi_device_interface_config_t devcfg={
        .clock_speed_hz = 4000000,  // 4 MHz
        .mode = 0,                  //SPI mode 0
        .spics_io_num = SPI_MASTER_CS,     
        .queue_size = 1,
    };

    ret |= spi_bus_add_device(SPI2_HOST, &devcfg, &handle);

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

    spi_transaction_t SPITransaction;
    esp_err_t ret;

    /* For BMP280, bit 7 of the register data will be the WR bit */
    uint8_t reg_to_write = (reg | SPI_WR_MASK);

    SPITransaction.length = 8U + (count * 8U);
	SPITransaction.tx_buffer = &reg_to_write;
	SPITransaction.rx_buffer = data;
    ret = spi_device_transmit( handle, &SPITransaction );

    if(ret != ESP_OK)
    {
		err = BMP280_ERR;
	}


    return err;
}

int16_t bmp280_spi_hal_write(uint8_t *data, uint16_t count)
{
    int16_t err = BMP280_OK;

    //User implementation here

    esp_err_t ret;
	spi_transaction_t SPITransaction;

    /* For BMP280, bit 7 of the register data will be the WR bit */
    data[0] &= ~(SPI_WR_MASK);

    SPITransaction.length = 8U + (count * 8U);
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	ret = spi_device_transmit( handle, &SPITransaction );

    if(ret != ESP_OK)
    {
		err = BMP280_ERR;
	}


    return err;
}

void bmp280_spi_hal_ms_delay(uint32_t ms) {

    //User implementation here

    vTaskDelay(pdMS_TO_TICKS(ms));
}
