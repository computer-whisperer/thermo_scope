//
// Created by christian on 9/15/23.
//

#ifndef THERMO_SCOPE_XPT2046_HPP
#define THERMO_SCOPE_XPT2046_HPP

#include "hardware/spi.h"

class XPT2046 {
  spi_inst_t * spi_inst;
  uint32_t cs_gpio;
public:
  XPT2046(spi_inst_t * spi_inst_in, uint32_t cs_gpio_in);
  void update();
};

#endif //THERMO_SCOPE_XPT2046_HPP
