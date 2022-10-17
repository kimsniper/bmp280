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

#include "bmp280_i2c.h" 
#include "bmp280_i2c_hal.h" 

bmp280_err_t bmp280_i2c_write_config(bmp280_config_t cfg)
{
    uint8_t reg = REG_CONFIG;
    uint8_t data[2];
    data[0] = reg;
    data[1] = (cfg.t_sb << 7) | (cfg.filter << 4) | cfg.spi3w_en;
    bmp280_err_t err = bmp280_i2c_hal_write(I2C_ADDRESS_BMP280, data, sizeof(data));
    return err;
}

bmp280_err_t bmp280_i2c_read_config(uint8_t *cfg)
{
    uint8_t reg = REG_CONFIG;
    bmp280_err_t err = bmp280_i2c_hal_read(I2C_ADDRESS_BMP280, &reg, cfg, 1);
    return err;
}

bmp280_err_t bmp280_i2c_reset()
{
    uint8_t reg = REG_RESET;
    bmp280_err_t err = bmp280_i2c_hal_write(I2C_ADDRESS_BMP280, &reg, 1);
    return err;
}

bmp280_err_t bmp280_i2c_read_pressure(uint16_t *dt)
{
    uint8_t reg = REG_PRESS_READ;
    uint8_t data[3];
    bmp280_err_t err = bmp280_i2c_hal_read(I2C_ADDRESS_BMP280, &reg, data, sizeof(data));
    return err;
}

bmp280_err_t bmp280_i2c_read_temperature(int8_t *dt)
{
    uint8_t reg = REG_TEMP_READ;
    uint8_t data[3];
    bmp280_err_t err = bmp280_i2c_hal_read(I2C_ADDRESS_BMP280, &reg, data, sizeof(data));
    return err;
} 

bmp280_err_t bmp280_i2c_read_part_number(uint8_t *dt)
{
    uint8_t reg = REG_ID_PARTNUMBER;
    bmp280_err_t err = bmp280_i2c_hal_read(I2C_ADDRESS_BMP280, &reg, dt, 1);
    return err;
} 