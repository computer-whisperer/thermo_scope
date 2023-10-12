//
// Created by christian on 9/15/23.
//

#include "i2c_bus_manager.hpp"
#include "hardware/gpio.h"

I2CBusManager::I2CBusManager(i2c_inst_t *i2c_bus_in, uint32_t scl_pin, uint32_t sda_pin): i2c_bus(i2c_bus_in) {
  gpio_set_function(sda_pin, GPIO_FUNC_I2C);
  gpio_set_function(scl_pin, GPIO_FUNC_I2C);
  i2c_init(i2c_bus_in, 200000);
}

void I2CBusManager::update() {

  if (absolute_time_diff_us(last_device_scan, get_absolute_time()) > 1000000)
  {
    for (auto driver : peripheral_drivers) {
      bool prev_connected = driver->is_present;
      bool new_connected = driver->check_device_presence();
      if (!prev_connected && new_connected) {
        driver->initialize_device();
      }
    }
    last_device_scan = get_absolute_time();
  }


  for (auto driver : peripheral_drivers) {
    if (driver->is_present) {
      driver->update();
    }
  }
}


