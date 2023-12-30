//
// Created by christian on 9/15/23.
//

#ifndef THERMO_SCOPE_SYSTEM_DATA_SOURCES_HPP
#define THERMO_SCOPE_SYSTEM_DATA_SOURCES_HPP
#include <telemetry_manager.hpp>

class SystemDataSources {
  TelemetryManager::Channel sys_time_channel{"system_time_drift", "us"};
  TelemetryManager::Channel battery_voltage_channel{"battery_voltage", "v"};
  TelemetryManager::Channel rp2040_temp_channel{"rp2040_temp", "c"};
  TelemetryManager::Channel heap_space_free{"heap_space_free"};

  public:

  void update();
};

#endif //THERMO_SCOPE_SYSTEM_DATA_SOURCES_HPP
