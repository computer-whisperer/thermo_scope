//
// Created by christian on 9/15/23.
//

#ifndef THERMO_SCOPE_SYSTEM_DATA_SOURCES_HPP
#define THERMO_SCOPE_SYSTEM_DATA_SOURCES_HPP

#include "data_collection.hpp"

class SystemDataSources {
  DataChannel* sys_time_channel;
  DataChannel* battery_voltage_channel;
  DataChannel* rp2040_temp_channel;
  DataChannel* heap_space_free;

  public:
  SystemDataSources();

  void update();
};

#endif //THERMO_SCOPE_SYSTEM_DATA_SOURCES_HPP
