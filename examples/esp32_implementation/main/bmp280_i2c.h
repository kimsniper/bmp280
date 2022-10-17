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

#ifndef MAIN_BMP280_I2C
#define MAIN_BMP280_I2C

#ifdef __cplusplus
extern "C" {
#endif

#include "bmp280_i2c_hal.h" 

typedef struct{
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
} bmp280_calib_t;

typedef struct{
    uint8_t t_sb : 3;
    uint8_t filter : 3;
    uint8_t spi3w_en : 1
} bmp280_config_t;

typedef struct{
    uint8_t measuring : 1;
    uint8_t im_update : 1;
} bmp280_status_t;

typedef struct{
    uint16_t pressure;
    int8_t temperature;
} bmp280_data_t;

/**
 * @brief BMP280 device address.
 * @details BMP280 I2C slave address.
 */
#define I2C_ADDRESS_BMP280              0x76

/**
 * @brief BMP280 command code registers.
 * @details R/W Command registers
 */
#define REG_CALIB                       0x88
#define REG_ID_PARTNUMBER               0xD0
#define REG_RESET                       0xE0
#define REG_STATUS                      0xF3
#define REG_CTRL_MEAS                   0xF4
#define REG_CONFIG                      0xF5
#define REG_PRESS_READ                  0xF7
#define REG_TEMP_READ                   0xFA

/**
 * @brief BMP280 REG_CTRL_MEAS register setting.
 * @details R/W Command values
 */
#define OSRS_x0                         0x00
#define OSRS_x1                         0x01
#define OSRS_x2                         0x02
#define OSRS_x4                         0x03
#define OSRS_x8                         0x04
#define OSRS_x16                        0x05

/**
 * @brief BMP280 REG_CONFIG register T_SB setting 
 * @details R/W time delay values
 */
#define T_SB_0_5                        0x00
#define T_SB_62_5                       0x01
#define T_SB_125                        0x02
#define T_SB_250                        0x03
#define T_SB_500                        0x04
#define T_SB_1000                       0x05
#define T_SB_2000                       0x06
#define T_SB_4000                       0x07

/**
 * @brief BMP280 REG_CONFIG register FILTER setting 
 * @details R/W time delay values
 */
#define FILTER_OFF                      0x01
#define FILTER_2                        0x02
#define FILTER_2                        0x05
#define FILTER_2                        0x03
#define FILTER_2                        0x04

/**
 * @brief BMP280 REG_CONFIG register POWER MODE setting 
 * @details R/W time delay values
 */
#define POWERMODE_SLEEP                 0x00
#define POWERMODE_FORCED                0x01
#define POWERMODE_NORMAL                0x02

/**
 * @brief BMP280 calibration setting.
 * @details Set global parameter calibration values.
 */
bmp280_err_t bmp280_i2c_set_calib();

/**
 * @brief BMP280 configuration setting.
 * @details Set configuration settings.
 */
bmp280_err_t bmp280_i2c_write_config(bmp280_config_t cfg);

/**
 * @brief BMP280 configuration setting.
 * @details Get configuration settings.
 */
bmp280_err_t bmp280_i2c_read_config(uint8_t *cfg);

/**
 * @brief BMP280 calibration setting.
 * @details Get configuration settings.
 */
bmp280_err_t bmp280_i2c_read_calib(bmp280_calib_t *clb);

/**
 * @brief BMP280 status.
 * @details Get device status.
 */
bmp280_err_t bmp280_i2c_read_status(bmp280_status_t *sts);

/**
 * @brief BMP280 setting.
 * @details Reset sensor.
 */
bmp280_err_t bmp280_i2c_reset();

/**
 * @brief BMP280 reading.
 * @details Read pressure sensor.
 */
bmp280_err_t bmp280_i2c_read_pressure_r(uint32_t *dt);

/**
 * @brief BMP280 reading.
 * @details Read temperature sensor.
 */
bmp280_err_t bmp280_i2c_read_temperature_r(uint32_t *dt);

/**
 * @brief BMP280 reading.
 * @details Read temperature sensor.
 */
bmp280_err_t bmp280_i2c_read_data(bmp280_data_t *dt);

/**
 * @brief BMP280 sensor part number.
 * @details Read device part number (Should be 0x58).
 */
bmp280_err_t bmp280_i2c_read_part_number(uint8_t *dt);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_BMP280_I2C */
