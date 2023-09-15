//
// Created by christian on 9/15/23.
//

#ifndef THERMO_SCOPE_BS_280_HPP
#define THERMO_SCOPE_BS_280_HPP
#include <pico/sync.h>

class BS_280 {
  critical_section_t critical_section;
  public:
  BS_280();

  void update();
};

#endif //THERMO_SCOPE_BS_280_HPP
