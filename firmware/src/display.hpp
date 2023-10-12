//
// Created by christian on 9/17/23.
//

#ifndef THERMO_SCOPE_DISPLAY_HPP
#define THERMO_SCOPE_DISPLAY_HPP

class Display
{
private:
  lv_obj_t * chart_obj = nullptr;
  int16_t chart_data[200];
  lv_obj_t * chart_value_label_obj = nullptr;
  lv_obj_t * wifi_state_label_obj = nullptr;
public:
  Display();
  void init();
  void update();
};

#endif //THERMO_SCOPE_DISPLAY_HPP
