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
#include "max11254.hpp"
#include "data_collection.hpp"
#include "data_collection.h"
#include "influxdb_export.hpp"
#include "system_data_sources.hpp"
#include "i2c_bus_manager.hpp"
#include "xpt2046.hpp"
#include "bme280.hpp"
#include "bmp581.hpp"
#include "ublox_g7.hpp"
#include "rate_meter.hpp"


#define SPI0_SCK 2
#define SPI0_MOSI 3
#define SPI0_MISO 4


std::vector<DataChannel*> data_channels;

static uint32_t data_collection_current_channel_list_rev = 1;
DataChannel* data_collection_create_new_channel(std::string channel_name)
{
  auto* new_channel = new DataChannel(std::move(channel_name));
  data_channels.push_back(new_channel);
  data_collection_current_channel_list_rev++;
  return new_channel;
}

volatile uint64_t data_collection_timestamp_offset_us = 0;
volatile float data_collection_local_clock_correction = 0;
uint32_t data_collection_recent_time_updates = 0;
int32_t data_collection_time_error_since_last_sync_us = 0;

volatile bool data_collection_ready_for_dormant = true;
volatile bool data_collection_requesting_dormant = false;

absolute_time_t data_collection_most_recent_gps_time_sync = nil_time;

struct time_point_t {
  uint64_t gps_time_us;
  uint64_t local_time_us;
};
RingBuffer<struct time_point_t> recent_time_points(10);
uint64_t last_gps_time_point_us = 0;

static uint64_t data_collection_apply_rate_compensation(uint64_t local_time_us)
{
  return (uint64_t)((int64_t)local_time_us + (int64_t)(((float)local_time_us)*data_collection_local_clock_correction));
}

uint64_t data_collection_convert_to_unix_time_us(uint64_t device_clock) {
  if (data_collection_timestamp_offset_us == 0)
  {
    return 0;
  }
  return data_collection_timestamp_offset_us + data_collection_apply_rate_compensation(device_clock);
}

uint64_t data_collection_get_unix_time_us() {
  data_collection_convert_to_unix_time_us(to_us_since_boot(get_absolute_time()));
}

void data_collection_update_gps_time(uint64_t gps_time_us, uint64_t local_time_us) {
  if ((gps_time_us - last_gps_time_point_us) > 60LL*1000000LL) {
    recent_time_points.clear();
    if (data_collection_recent_time_updates != 0)
    {
      data_collection_time_error_since_last_sync_us = (int64_t)data_collection_convert_to_unix_time_us(local_time_us) - (int64_t)gps_time_us;
    }

    data_collection_recent_time_updates = 0;
  }


  struct time_point_t new_time_point = {gps_time_us, local_time_us};
  recent_time_points.push(new_time_point);

  if (recent_time_points.get_num_entries() > 5) {
    // Get approximation of derivative
    struct time_point_t oldest_time_point = recent_time_points.get_value_by_age((int32_t) recent_time_points.get_num_entries() - 1);
    struct time_point_t newest_time_point = recent_time_points.get_value_by_age(0);
    uint64_t gps_time_elapsed_us = newest_time_point.gps_time_us - oldest_time_point.gps_time_us;
    uint64_t local_time_elapsed_us = newest_time_point.local_time_us - oldest_time_point.local_time_us;
    float current_clock_correction = (float) (gps_time_elapsed_us - local_time_elapsed_us) / (float) local_time_elapsed_us;
    if (current_clock_correction == 0)
    {
      data_collection_local_clock_correction = current_clock_correction;
    }
    else
    {
      data_collection_local_clock_correction += (current_clock_correction - data_collection_local_clock_correction)/6;
    }


    data_collection_recent_time_updates++;
  }
  data_collection_most_recent_gps_time_sync = get_absolute_time();


  last_gps_time_point_us = gps_time_us;

  data_collection_timestamp_offset_us = (gps_time_us - data_collection_apply_rate_compensation(local_time_us));
}



void data_collection()
{
  data_collection_ready_for_dormant = false;

  spi_init(spi0, 2500000);
  gpio_set_function(SPI0_MISO, GPIO_FUNC_SPI);
  gpio_set_function(SPI0_SCK, GPIO_FUNC_SPI);
  gpio_set_function(SPI0_MOSI, GPIO_FUNC_SPI);

  XPT2046 xpt2046(spi0, 17);
  MAX11254 max11254_driver(spi0, 1, 0, 6);

  UBLOX_G7 ublox_g7{uart1, 8, 9, 10};
  ublox_g7.initialize_device();

  I2CBusManager i2c0_manager{i2c0, 21, 20};
  i2c0_manager.peripheral_drivers.push_back(new BME280(i2c0, false));
  //I2CBusManager i2c1_manager{i2c1, 19, 18};
  //i2c1_manager.peripheral_drivers.push_back(new BMP581(i2c1, false));
  //i2c1_manager.peripheral_drivers.push_back(new BMP581(i2c1, true));
  SystemDataSources system_data_sources;

  max11254_driver.init_device();
  max11254_driver.set_rate(8);

  absolute_time_t last_system_update = get_absolute_time();

  while (true)
  {
    // Dormant mode concerns
    if (data_collection_requesting_dormant)
    {
      ublox_g7.on_enter_dormant();
      data_collection_ready_for_dormant = true;
      while (data_collection_requesting_dormant)
      {
        sleep_ms(10);
      }
      ublox_g7.on_exit_dormant();
      data_collection_ready_for_dormant = false;
    }

    ublox_g7.update();
    //xpt2046.update();
    max11254_driver.update();
    i2c0_manager.update();
    //i2c1_manager.update();
    if (absolute_time_diff_us(last_system_update, get_absolute_time()) > 100000)
    {
      system_data_sources.update();
      last_system_update = get_absolute_time();
    }
    sleep_ms(1);
  }
}

uint32_t data_collection_update_channel_names(lv_obj_t *dropdown, uint32_t current_rev) {
  if (current_rev != data_collection_current_channel_list_rev)
  {
    lv_dropdown_clear_options(dropdown);
    uint32_t option_id = 0;
    for (DataChannel* channel : data_channels)
    {
      lv_dropdown_add_option(dropdown, channel->channel_name.c_str(), option_id++);
    }
  }
  return data_collection_current_channel_list_rev;
}

void* data_collection_get_channel_pointer(char* name)
{
  for (DataChannel* channel : data_channels)
  {
    if (strcmp(channel->channel_name.c_str(), name) == 0)
    {
      return channel;
    }
  }
  return nullptr;
}

uint32_t data_collection_get_channel_chart_data(void* channel_ptr, double* buffer, uint32_t buffer_size)
{
  if (channel_ptr == nullptr)
    return 0;
  auto channel = (DataChannel*)channel_ptr;
  uint32_t num_samples = channel->graph_downsample_buffer.get_num_entries();
  if (num_samples > buffer_size)
    num_samples = buffer_size;
  for (uint32_t i = 0; i < num_samples; i++)
  {
    buffer[i] = channel->graph_downsample_buffer.get_value_by_age((int32_t)i);
  }
  return num_samples;
}

void *data_collection_get_default_channel_pointer() {
  if (data_channels.empty())
  {
    return nullptr;
  }
  return data_channels[0];
}

void data_collection_core0_process_samples() {
  for (auto channel : data_channels)
  {
    channel->core0_process_data();
  }
  bool data_still_available = true;
  while (data_still_available && influxdb_can_push_point())
  {
    if (data_collection_timestamp_offset_us == 0)
    {
      break;
      // Wait for timestamp offset from GPS
    }
    data_still_available = false;
    for (auto channel : data_channels)
    {
      if (!channel->influx_export_buffer.is_empty)
      {
        auto data = channel->influx_export_buffer.pop_oldest();
        data.timestamp_us = data_collection_convert_to_unix_time_us(data.timestamp_us);
        influxdb_push_point(data, channel);
        if (!influxdb_can_push_point())
        {
          break;
        }
        data_still_available |= (channel->influx_export_buffer.get_num_entries() > 0);
      }
    }
  }
}