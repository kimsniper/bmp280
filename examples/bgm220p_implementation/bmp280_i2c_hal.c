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

#include "bmp280_i2c_hal.h" 

//Hardware Specific Components
#include <stdio.h>
#include "em_i2c.h"
#include "em_cmu.h"
#include "sl_udelay.h"

//I2C User Defines
#define I2CSPM_TRANSFER_TIMEOUT   300000
#define I2C_SCL_PIN               2
#define I2C_SDA_PIN               3
#define I2C_IO_PORT               gpioPortD
#define I2C_RECOVER_NUM_CLOCKS    1
#define I2CSPM_SCL_HOLD_TIME      1

int16_t bmp280_i2c_hal_init()
{
    int16_t err = BMP280_OK;

    int i;
    CMU_ClockEnable(cmuClock_I2C1, true);

    // Use default settings
    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

    // Using PD3 (SDA) and PD2 (SCL)
    GPIO_PinModeSet(I2C_IO_PORT, I2C_SCL_PIN, gpioModeWiredAndPullUpFilter, 1);
    GPIO_PinModeSet(I2C_IO_PORT, I2C_SDA_PIN, gpioModeWiredAndPullUpFilter, 1);

    /* In some situations, after a reset during an I2C transfer, the slave
         device may be left in an unknown state. Send 9 clock pulses to
         set slave in a defined state. */
    for (i = 0; i < I2C_RECOVER_NUM_CLOCKS; i++) {
      GPIO_PinOutClear(I2C_IO_PORT, I2C_SCL_PIN);
      bmp280_i2c_hal_ms_delay(I2CSPM_SCL_HOLD_TIME);
      GPIO_PinOutSet(I2C_IO_PORT, I2C_SCL_PIN);
      bmp280_i2c_hal_ms_delay(I2CSPM_SCL_HOLD_TIME);
    }

    // Route I2C pins to GPIO
    GPIO->I2CROUTE[1].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;
    GPIO->I2CROUTE[1].SCLROUTE = (uint32_t)((I2C_SCL_PIN << _GPIO_I2C_SCLROUTE_PIN_SHIFT)
                                            | (I2C_IO_PORT << _GPIO_I2C_SCLROUTE_PORT_SHIFT));
    GPIO->I2CROUTE[1].SDAROUTE = (uint32_t)((I2C_SDA_PIN << _GPIO_I2C_SDAROUTE_PIN_SHIFT)
                                            | (I2C_IO_PORT << _GPIO_I2C_SDAROUTE_PORT_SHIFT));

    // Initialize the I2C
    I2C_Init(I2C1, &i2cInit);

    // Enable automatic STOP on NACK
    //I2C1->CTRL = I2C_CTRL_AUTOSN;


    return err == BMP280_OK ? BMP280_OK :  BMP280_ERR;
}

int16_t bmp280_i2c_hal_read(uint8_t address, uint8_t *reg, uint8_t *data, uint16_t count)
{
    int16_t err = BMP280_OK;

    // Transfer structure
    I2C_TransferSeq_TypeDef i2cTransfer;
    I2C_TransferReturn_TypeDef result;

    uint32_t timeout = I2CSPM_TRANSFER_TIMEOUT;

    // Initialize I2C transfer
    i2cTransfer.addr          = address << 1;
    i2cTransfer.flags         = I2C_FLAG_WRITE_READ; // must write target address before reading
    i2cTransfer.buf[0].data   = reg;
    i2cTransfer.buf[0].len    = 1;
    i2cTransfer.buf[1].data   = data;
    i2cTransfer.buf[1].len    = count;

    result = I2C_TransferInit(I2C1, &i2cTransfer);

    // Send data
    while (result == i2cTransferInProgress && timeout--) {
        result = I2C_Transfer(I2C1);
    }

    if (result != i2cTransferDone) {
        err = BMP280_ERR;
    }


    return err == BMP280_OK ? BMP280_OK :  BMP280_ERR;
}

int16_t bmp280_i2c_hal_write(uint8_t address, uint8_t *data, uint16_t count)
{
    int16_t err = BMP280_OK;

    // Transfer structure
    I2C_TransferSeq_TypeDef i2cTransfer;
    I2C_TransferReturn_TypeDef result;

    uint32_t timeout = I2CSPM_TRANSFER_TIMEOUT;

    // Initialize I2C transfer
    i2cTransfer.addr          = address << 1;
    i2cTransfer.flags         = I2C_FLAG_WRITE_WRITE;
    i2cTransfer.buf[0].data   = &data[0];
    i2cTransfer.buf[0].len    = 1;
    i2cTransfer.buf[1].data   = (count == 1) ? NULL : &data[1];
    i2cTransfer.buf[1].len    = count - 1;

    result = I2C_TransferInit(I2C1, &i2cTransfer);

    // Send data
    while (result == i2cTransferInProgress && timeout--) {
        result = I2C_Transfer(I2C1);
    }

    if (result != i2cTransferDone) {
        err = BMP280_ERR;
    }


    return err == BMP280_OK ? BMP280_OK :  BMP280_ERR;
}

void bmp280_i2c_hal_ms_delay(uint32_t ms) {

  sl_udelay_wait(ms * 1000);
    
}
