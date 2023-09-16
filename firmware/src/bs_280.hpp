//
// Created by christian on 9/15/23.
//

#ifndef THERMO_SCOPE_BS_280_HPP
#define THERMO_SCOPE_BS_280_HPP
#include <pico/sync.h>
#include "hardware/uart.h"

class BS_280 {
  critical_section_t critical_section;
  uart_inst_t * uart_dev;
  uint32_t tx_gpio;
  uint32_t rx_gpio;
  public:
  BS_280(uart_inst_t * uart_dev_in, uint32_t tx_gpio_in, uint32_t rx_gpio_in);

  void update();
};

#endif //THERMO_SCOPE_BS_280_HPP
