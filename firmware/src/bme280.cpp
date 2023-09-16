//
// Created by christian on 6/7/23.
//

#include "bme280.hpp"
#include <cstdint>
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#define ADDR _u(0x76)

// hardware registers
#define REG_CONFIG _u(0xF5)
#define REG_CTRL_MEAS _u(0xF4)
#define REG_RESET _u(0xE0)

#define REG_TEMP_XLSB _u(0xFC)
#define REG_TEMP_LSB _u(0xFB)
#define REG_TEMP_MSB _u(0xFA)

#define REG_PRESSURE_XLSB _u(0xF9)
#define REG_PRESSURE_LSB _u(0xF8)
#define REG_PRESSURE_MSB _u(0xF7)

// calibration registers
#define REG_DIG_T1_LSB _u(0x88)
#define REG_DIG_T1_MSB _u(0x89)
#define REG_DIG_T2_LSB _u(0x8A)
#define REG_DIG_T2_MSB _u(0x8B)
#define REG_DIG_T3_LSB _u(0x8C)
#define REG_DIG_T3_MSB _u(0x8D)
#define REG_DIG_P1_LSB _u(0x8E)
#define REG_DIG_P1_MSB _u(0x8F)
#define REG_DIG_P2_LSB _u(0x90)
#define REG_DIG_P2_MSB _u(0x91)
#define REG_DIG_P3_LSB _u(0x92)
#define REG_DIG_P3_MSB _u(0x93)
#define REG_DIG_P4_LSB _u(0x94)
#define REG_DIG_P4_MSB _u(0x95)
#define REG_DIG_P5_LSB _u(0x96)
#define REG_DIG_P5_MSB _u(0x97)
#define REG_DIG_P6_LSB _u(0x98)
#define REG_DIG_P6_MSB _u(0x99)
#define REG_DIG_P7_LSB _u(0x9A)
#define REG_DIG_P7_MSB _u(0x9B)
#define REG_DIG_P8_LSB _u(0x9C)
#define REG_DIG_P8_MSB _u(0x9D)
#define REG_DIG_P9_LSB _u(0x9E)
#define REG_DIG_P9_MSB _u(0x9F)

// number of calibration registers to be read
#define NUM_CALIB_PARAMS 24

static struct BME280::bmp280_calib_param cal_params;

void BME280::bme280_read_raw(int32_t* temp, int32_t* pressure, int32_t* humidity) {
  // BMP280 data registers are auto-incrementing and we have 3 temperature and
  // pressure registers each, so we start at 0xF7 and read 6 bytes to 0xFC
  // note: normal mode does not require further ctrl_meas and config register writes

  uint8_t buf[8];
  uint8_t reg = REG_PRESSURE_MSB;
  i2c_write_blocking(i2c_bus, ADDR, &reg, 1, true);  // true to keep master control of bus
  i2c_read_blocking(i2c_bus, ADDR, buf, 8, false);  // false - finished with bus

  // store the 20 bit read in a 32 bit signed integer for conversion
  *pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
  *temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
  *humidity = buf[6] << 8 | buf[7];
}

void BME280::bme280_reset() {
  // reset the device with the power-on-reset procedure
  uint8_t buf[2] = { REG_RESET, 0xB6 };
  i2c_write_blocking(i2c_bus, ADDR, buf, 2, false);
}

// intermediate function that calculates the fine resolution temperature
// used for both pressure and temperature conversions
int32_t BME280::bme280_convert(int32_t temp, struct bmp280_calib_param* params) {
  // use the 32-bit fixed point compensation implementation given in the
  // datasheet

  int32_t var1, var2;
  var1 = ((((temp >> 3) - ((int32_t)params->dig_t1 << 1))) * ((int32_t)params->dig_t2)) >> 11;
  var2 = (((((temp >> 4) - ((int32_t)params->dig_t1)) * ((temp >> 4) - ((int32_t)params->dig_t1))) >> 12) * ((int32_t)params->dig_t3)) >> 14;
  return var1 + var2;
}

int32_t BME280::bme280_convert_temp(int32_t temp) {
  // uses the BMP280 calibration parameters to compensate the temperature value read from its registers
  int32_t t_fine = bme280_convert(temp, &cal_params);
  return (t_fine * 5 + 128) >> 8;
}

int32_t BME280::bme280_convert_humidity(int32_t humidity, int32_t temp) {
  int32_t t_fine = bme280_convert(temp, &cal_params);

  int32_t v_x1_u32r;
  v_x1_u32r = (t_fine - ((int32_t) 76800));
  v_x1_u32r = (((((humidity << 14) - (((int32_t) cal_params.dig_H4) << 20) - (((int32_t) cal_params.dig_H5) * v_x1_u32r)) +
                 ((int32_t) 16384)) >> 15) * (((((((v_x1_u32r * ((int32_t) cal_params.dig_H6)) >> 10) * (((v_x1_u32r *
                                                                                                ((int32_t) cal_params.dig_H3))
          >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
                                               ((int32_t) cal_params.dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t) cal_params.dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

  return v_x1_u32r >> 12;
}

int32_t BME280::bme280_convert_pressure(int32_t pressure, int32_t temp) {
  // uses the BMP280 calibration parameters to compensate the pressure value read from its registers

  int32_t t_fine = bme280_convert(temp, &cal_params);

  int32_t var1, var2;
  uint32_t converted = 0.0;
  var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)cal_params.dig_p6);
  var2 += ((var1 * ((int32_t)cal_params.dig_p5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)cal_params.dig_p4) << 16);
  var1 = (((cal_params.dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)cal_params.dig_p2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)cal_params.dig_p1)) >> 15);
  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  converted = (((uint32_t)(((int32_t)1048576) - pressure) - (var2 >> 12))) * 3125;
  if (converted < 0x80000000) {
    converted = (converted << 1) / ((uint32_t)var1);
  } else {
    converted = (converted / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)cal_params.dig_p9) * ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(converted >> 2)) * ((int32_t)cal_params.dig_p8)) >> 13;
  converted = (uint32_t)((int32_t)converted + ((var1 + var2 + cal_params.dig_p7) >> 4));
  return converted;
}

void BME280::bme280_get_calib_params(struct bmp280_calib_param* params) {
  // raw temp and pressure values need to be calibrated according to
  // parameters generated during the manufacturing of the sensor
  // there are 3 temperature params, and 9 pressure params, each with a LSB
  // and MSB register, so we read from 24 registers

  uint8_t buf[26] = { 0 };
  uint8_t reg = REG_DIG_T1_LSB;
  i2c_write_blocking(i2c_bus, ADDR, &reg, 1, true);  // true to keep master control of bus
  // read in one go as register addresses auto-increment
  i2c_read_blocking(i2c_bus, ADDR, buf, 26, false);  // false, we're done reading

  // store these in a struct for later use
  params->dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
  params->dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
  params->dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

  params->dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
  params->dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
  params->dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
  params->dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
  params->dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
  params->dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
  params->dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
  params->dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
  params->dig_p9 = (int16_t)(buf[23] << 8) | buf[22];

  params->dig_H1 = buf[25];

  reg = 0xE1;
  i2c_write_blocking(i2c_bus, ADDR, &reg, 1, true);  // true to keep master control of bus
  // read in one go as register addresses auto-increment
  i2c_read_blocking(i2c_bus, ADDR, buf, 8, false);  // false, we're done reading

  params->dig_H2 = (int16_t) (buf[0] | (buf[1] << 8));
  params->dig_H3 = (int8_t) buf[2];
  params->dig_H4 = (int16_t) (buf[3] << 4 | (buf[4] & 0xf));
  params->dig_H5 = (int16_t) ((buf[5] >> 4) | (buf[6] << 4));
  params->dig_H6 = (int8_t) buf[7];
}

BME280::BME280(i2c_inst_t* i2c_bus_in) : i2c_bus(i2c_bus_in) {
  pressure_data_channel = data_collection_create_new_channel("BME280_Pressure");
  temperature_data_channel = data_collection_create_new_channel("BME280_Temperature");
  humidity_data_channel = data_collection_create_new_channel("BME280_Humidity");

  // use the "handheld device dynamic" optimal setting (see datasheet)
  uint8_t buf[2];

  // 500ms sampling time, x16 filter
  const uint8_t reg_config_val = ((0x04 << 5) | (0x05 << 2)) & 0xFC;

  // send register number followed by its corresponding value
  buf[0] = REG_CONFIG;
  buf[1] = reg_config_val;
  i2c_write_blocking(i2c_bus, ADDR, buf, 2, false);

  // osrs_t x1, osrs_p x4, normal mode operation
  const uint8_t reg_ctrl_meas_val = (0x01 << 5) | (0x03 << 2) | (0x03);
  buf[0] = REG_CTRL_MEAS;
  buf[1] = reg_ctrl_meas_val;
  i2c_write_blocking(i2c_bus, ADDR, buf, 2, false);

  bme280_get_calib_params(&cal_params);
}

void BME280::update() {
  if (absolute_time_diff_us(last_fetch_timestamp, get_absolute_time()) > 100000)
  {
    int32_t raw_temperature = 0;
    int32_t raw_pressure = 0;
    int32_t raw_humidity = 0;
    bme280_read_raw(&raw_temperature, &raw_pressure, &raw_humidity);
    double temperature = bme280_convert_temp(raw_temperature) / 100.0;
    double pressure = bme280_convert_pressure(raw_pressure, raw_temperature) / 1000.0;
    double humidity = bme280_convert_humidity(raw_humidity, raw_temperature) / 1024.0;
    temperature_data_channel->push_new_value(temperature);
    pressure_data_channel->push_new_value(pressure);
    humidity_data_channel->push_new_value(humidity);
    last_fetch_timestamp = get_absolute_time();
  }
}
