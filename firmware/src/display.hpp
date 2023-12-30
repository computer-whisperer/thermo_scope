//
// Created by christian on 9/17/23.
//

#ifndef THERMO_SCOPE_DISPLAY_HPP
#define THERMO_SCOPE_DISPLAY_HPP
#include "lvgl.h"

class Display
{
private:
  lv_obj_t * press_label_obj = nullptr;
  lv_obj_t * temp_label_obj = nullptr;
  lv_obj_t * humid_label_obj = nullptr;
public:
  Display();
  void init();
  void update();
};

#endif //THERMO_SCOPE_DISPLAY_HPP
