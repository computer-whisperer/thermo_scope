//
// Created by christian on 6/7/23.
//

#ifndef PICO_TEST_MANAGER_BME280_H
#define PICO_TEST_MANAGER_BME280_H

//
// Created by christian on 6/7/23.
//

#include "bme280.hpp"
#include <cstdint>
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "data_collection.hpp"
#include "i2c_bus_manager.hpp"

class BME280 : public I2CPeripheralDriver
{
  DataChannel* humidity_data_channel;
  DataChannel* temperature_data_channel;
  DataChannel* pressure_data_channel;

  absolute_time_t last_fetch_timestamp = nil_time;

public:
  bool check_device_presence();

  static constexpr uint8_t get_i2c_address(bool addr_select){
    return addr_select? 0x77 : 0x76;
  }

  BME280(i2c_inst_t* i2c_bus_in, bool addr_select_in);

  void initialize_device();

  struct bmp280_calib_param {
    // temperature params
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    // pressure params
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;

    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
  };

  void bme280_read_raw(int32_t* temp, int32_t* pressure, int32_t* humidity);

  void bme280_reset() ;
// intermediate function that calculates the fine resolution temperature
// used for both pressure and temperature conversions
  int32_t bme280_convert(int32_t temp, struct bmp280_calib_param* params);
  int32_t bme280_convert_temp(int32_t temp);

  int32_t bme280_convert_pressure(int32_t pressure, int32_t temp);
  int32_t bme280_convert_humidity(int32_t humidity, int32_t temp);

  void bme280_get_calib_params(struct bmp280_calib_param* params);


  void update();
};




#endif //PICO_TEST_MANAGER_BME280_H
