//
// Created by christian on 9/15/23.
//

#ifndef THERMO_SCOPE_I2C_BUS_MANAGER_HPP
#define THERMO_SCOPE_I2C_BUS_MANAGER_HPP

#include <vector>
#include "hardware/i2c.h"

class I2CPeripheralDriver {
protected:
  i2c_inst_t* i2c_bus;
  uint8_t i2c_addr;
public:
  bool is_present = false;
  I2CPeripheralDriver(i2c_inst_t* i2c_bus_in, uint8_t i2c_addr_in): i2c_bus(i2c_bus_in), i2c_addr(i2c_addr_in){};
  virtual void initialize_device(){};
  virtual bool check_device_presence(){return false;};
  virtual void update(){};
};


class I2CBusManager {
  i2c_inst_t* i2c_bus;
  absolute_time_t last_device_scan = nil_time;
public:

  std::vector<I2CPeripheralDriver*> peripheral_drivers;
  I2CBusManager(i2c_inst_t* i2c_bus_in, uint32_t scl_pin, uint32_t sda_pin);
  void update();
};

#endif //THERMO_SCOPE_I2C_BUS_MANAGER_HPP
