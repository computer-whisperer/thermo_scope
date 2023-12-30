#include <sys/cdefs.h>
#include <utility>
#include <vector>
#include <cstdlib>
#include <src/widgets/lv_dropdown.h>
//
// Created by christian on 9/6/23.
//

#include "hardware/gpio.h"
#include "pico/time.h"
#include "data_collection.hpp"
#include "data_collection.h"

#include <bmp585.hpp>
#include <sht45.hpp>
#include <telemetry_manager.hpp>
#include <time_manager.hpp>
#include <tsys01.hpp>

#include "system_data_sources.hpp"
#include "i2c_bus_manager.hpp"
#include "ublox_ubx.hpp"
#include "main.hpp"


volatile bool data_collection_ready_for_dormant = true;
volatile bool data_collection_requesting_dormant = false;

TelemetryManager telemetry_manager;

void data_collection()
{
  telemetry_manager.use_influxdb(&influxdb_client, "thermo_scope");

  data_collection_ready_for_dormant = false;

  UBLOX_UBX ublox_ubx{uart1, 8, 9, 10};
  ublox_ubx.initialize_device();

  I2CBusManager i2c1_manager{i2c1, 19, 18};
  i2c1_manager.peripheral_drivers.push_back(new BMP585(i2c1, false));
  i2c1_manager.peripheral_drivers.push_back(new SHT45(i2c1));
  i2c1_manager.peripheral_drivers.push_back(new TSYS01(i2c1, false));
  SystemDataSources system_data_sources;

  absolute_time_t last_system_update = get_absolute_time();

  while (true)
  {
    // Dormant mode concerns
    if (data_collection_requesting_dormant)
    {
      ublox_ubx.on_enter_dormant();
      data_collection_ready_for_dormant = true;
      while (data_collection_requesting_dormant)
      {
        sleep_ms(10);
      }
      ublox_ubx.on_exit_dormant();
      data_collection_ready_for_dormant = false;
    }

    ublox_ubx.update();
    i2c1_manager.update();
    if (absolute_time_diff_us(last_system_update, get_absolute_time()) > 100000)
    {
      system_data_sources.update();
      last_system_update = get_absolute_time();
    }
    sleep_ms(1);
  }
}

void data_collection_core0_process_samples() {
  telemetry_manager.push_data_to_influxdb();
}

