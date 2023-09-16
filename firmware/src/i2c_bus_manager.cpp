//
// Created by christian on 9/15/23.
//

#include "i2c_bus_manager.hpp"
#include "hardware/gpio.h"

I2CBusManager::I2CBusManager(i2c_inst_t *i2c_bus_in, uint32_t scl_pin, uint32_t sda_pin): i2c_bus(i2c_bus_in) {

  i2c_init(i2c_bus_in, 2500000);
  gpio_set_function(sda_pin, GPIO_FUNC_I2C);
  gpio_set_function(scl_pin, GPIO_FUNC_I2C);

  bme280 = new BME280(i2c_bus);
}

void I2CBusManager::update() {
  bme280->update();
}


