//
// Created by christian on 9/16/23.
//

#include <cstring>
#include "bmp581.hpp"


BMP581::BMP581(i2c_inst_t* i2c_bus_in, bool addr_select_in) :
        I2CPeripheralDriver(i2c_bus_in, get_i2c_address(addr_select_in))
{
  char buf[50];
  snprintf(buf, sizeof(buf), "BMP581_%d_Pressure", addr_select_in);
  pressure_data_channel = data_collection_create_new_channel(buf);
  snprintf(buf, sizeof(buf), "BMP581_%d_Temperature", addr_select_in);
  temperature_data_channel = data_collection_create_new_channel(buf);
}

bool BMP581::check_device_presence(){
  is_present = this->field_read(FIELD_CHIP_ID) == 0x50;
  return is_present;
}

void BMP581::initialize_device() {

  // Reset unit
  this->field_write(FIELD_CMD, 0xB6);
  sleep_ms(4);

  this->field_write(FIELD_PRESS_EN, 0x01);
  this->set_fifo_mode(true, true);
  start_normal_mode(0x11);
}

//! Configure what measurements are stored in the bmp581 fifo registers
void BMP581::set_fifo_mode(bool temp_in_fifo_in, bool press_in_fifo_in) {
  this->temp_in_fifo = temp_in_fifo_in;
  this->press_in_fifo = press_in_fifo_in;
  // fifo_frame_sel is described in the datasheet. The 0x2 bit indicates pressure, and 0x1 bit indicates temperature.
  this->field_write(FIELD_FIFO_FRAME_SEL, (press_in_fifo_in << 1) | temp_in_fifo_in);
}

void BMP581::update() {
  if (absolute_time_diff_us(last_fetch_timestamp, get_absolute_time()) > 100000)
  {
    pull_from_fifo();
    last_fetch_timestamp = get_absolute_time();
  }
}

void BMP581::register_write(uint8_t addr, uint8_t value) {
  uint8_t buf[2];
  buf[0] = addr;
  buf[1] = value;
  i2c_write_blocking(i2c_bus, i2c_addr, buf, 2, false);
}

uint8_t BMP581::register_read(uint8_t addr) {
  i2c_write_blocking(i2c_bus, i2c_addr, &addr, 1, true);
  uint8_t data = 0;
  i2c_read_blocking(i2c_bus, i2c_addr, &data, 1, false);
  return data;
}

uint32_t BMP581::do_forced_measurement(float *temp_out, float *press_out) {
  this->field_write(FIELD_PWR_MODE, 0x02);
  for (uint8_t i = 0; i < 10; i++) {
    sleep_ms(10);
    if (this->field_read(FIELD_PWR_MODE) == 0x00) {
      // Now in standby mode again, time to read values
      *temp_out = process_temperature_data_C(
              this->field_read(FIELD_TEMP_MSB),
              this->field_read(FIELD_TEMP_LSB),
              this->field_read(FIELD_TEMP_XLSB));
      *press_out = process_pressure_data_kPa(
              this->field_read(FIELD_PRESS_MSB),
              this->field_read(FIELD_PRESS_LSB),
              this->field_read(FIELD_PRESS_XLSB));
      return 0;
    }
  }
  return 1;
}

/*!
 * Start the "normal" mode of the bmp581, where it will automatically sample on its own
 * @param odr_state Sample rate configuration, see datasheet for details
 */
void BMP581::start_normal_mode(int odr_state) {
  // These two cause the FIFO to automatically drop old entries and
  // hold the max number of frames (32)
  this->field_write(FIELD_FIFO_THRESHOLD, 0x00);
  this->field_write(FIELD_FIFO_MODE, 0x00);

  // Normal mode (periodic sample)
  this->field_write(FIELD_PWR_MODE, 0x01);

  // Oversampling configuration (disabled here)
  this->field_write(FIELD_OSR_P, 0);
  this->field_write(FIELD_OSR_T, 0);

  // Sample rate configuration, see datasheet for values
  this->field_write(FIELD_ODR, odr_state);
}

void BMP581::pull_from_fifo() {
  uint8_t available_samples = this->field_read(FIELD_FIFO_COUNT);
  uint8_t bytes_per_sample = 0;
  if (this->temp_in_fifo) {
    bytes_per_sample += 3;
  }
  if (this->press_in_fifo) {
    bytes_per_sample += 3;
  }
  if ((bytes_per_sample > 0) && (available_samples > 0)) {
    // The fifo in the chip can store up to 32 24-bit values.
    uint8_t read_buffer[32 * 3];
    memset(read_buffer, 0, sizeof(read_buffer));
    uint8_t bytes_to_read = available_samples * bytes_per_sample;
    uint8_t addr_value = BMP581::REG_FIFO_DATA;

    i2c_write_blocking(i2c_bus, i2c_addr, &addr_value, 1, true);
    i2c_read_blocking(i2c_bus, i2c_addr, read_buffer, bytes_to_read, false);

    uint64_t temp_data_timestamp = to_us_since_boot(get_absolute_time());
    uint64_t press_data_timestamp = to_us_since_boot(get_absolute_time());

    uint32_t sample_period_us = 1000000/40;

    uint8_t *read_ptr = read_buffer;
    for (uint8_t i = 0; i < available_samples; i++) {
      if (read_ptr[0] == 0x7F) {
        // FIFO is empty or disabled
        // Somehow this state is actually reached sometimes. Nothing should break if this is reached, but it means that somehow
        // the bmp581 told us there were more samples available then are actually readable.
        break;
      }
      if (temp_in_fifo) {
        float value = process_temperature_data_C(read_ptr[2], read_ptr[1], read_ptr[0]);
        temperature_data_channel->push_new_value(value, from_us_since_boot(temp_data_timestamp));
        temp_data_timestamp -= sample_period_us;
        read_ptr += 3;
      }
      if (press_in_fifo) {
        float value = process_pressure_data_kPa(read_ptr[2], read_ptr[1], read_ptr[0]);
        pressure_data_channel->push_new_value(value, from_us_since_boot(press_data_timestamp));
        press_data_timestamp -= sample_period_us;
        read_ptr += 3;
      }
    }
  }
}
