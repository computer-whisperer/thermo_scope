//
// Created by christian on 9/15/23.
//

#ifndef THERMO_SCOPE_I2C_BUS_MANAGER_HPP
#define THERMO_SCOPE_I2C_BUS_MANAGER_HPP

#include "hardware/i2c.h"
#include "bme280.hpp"

class I2CBusManager {
  i2c_inst_t* i2c_bus;

  BME280* bme280;

  public:
  I2CBusManager(i2c_inst_t* i2c_bus_in, uint32_t scl_pin, uint32_t sda_pin);
  void update();
};

#endif //THERMO_SCOPE_I2C_BUS_MANAGER_HPP
