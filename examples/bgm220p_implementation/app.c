/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "app_log.h"
#include "bmp280_i2c.h"
#include "bmp280_i2c_hal.h"
#include "app.h"

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  sl_status_t err = BMP280_OK;
  uint8_t id = 0;

  bmp280_i2c_hal_init();

  err = bmp280_i2c_reset();
  if(err != BMP280_OK) app_log_error("Error setting the device!\n\r");

  err += bmp280_i2c_read_part_number(&id);
  if(err == BMP280_OK){
      app_log_info("Part number: 0x%02x\n\r", id);
  }
  else{
      app_log_error("Unable to read part number!\n\r");
  }

  err += bmp280_i2c_set_calib();
  app_log_info("Calibration data setting: %s\n\r", err == BMP280_OK ? "Successful" : "Failed");

  err += bmp280_i2c_write_power_mode(POWERMODE_NORMAL);
  app_log_info("Setting to normal mode: %s\n\r", err == BMP280_OK ? "Successful" : "Failed");

  //Config setting. We'll use suggested settings for elevation detection
  err += bmp280_i2c_write_config_filter(FILTER_4);
  bmp280_ctrl_meas_t ctrl_meas = {
      .osrs_press = OSRS_x4,
      .osrs_tmp = OSRS_x1,
  };
  err += bmp280_i2c_write_osrs(ctrl_meas);

  if (err == BMP280_OK && id == 0x58)
  {
      app_log_info("BMP280 initialization successful\n\r");
      bmp280_data_t bmp280_dt;
      while(1)
      {
          //Reading here
          if(bmp280_i2c_read_data(&bmp280_dt) == BMP280_OK)
          {
              app_log_info("Pressure: %d Pa\n\r", (int)bmp280_dt.pressure/256);
              app_log_info("Temperature: %d °C\n\r", (int)bmp280_dt.temperature/100);
          }
          else{
              app_log_error("Error reading data!\n\r");
          }

          bmp280_i2c_hal_ms_delay(2000);
      }
  }
  else{
      app_log_error("BMP280 initialization failed!\n\r");
  }
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
}
