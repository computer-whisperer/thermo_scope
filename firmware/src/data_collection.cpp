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
#include "bs_280.hpp"
#include "system_data_sources.hpp"
#include "i2c_bus_manager.hpp"
#include "xpt2046.hpp"

volatile double voltages[6];

#define SPI0_SCK 2
#define SPI0_MOSI 3
#define SPI0_MISO 4

volatile uint32_t max11254_data[6];

std::vector<DataChannel*> data_channels;

static uint32_t data_collection_current_channel_list_rev = 1;
DataChannel* data_collection_create_new_channel(std::string channel_name)
{
  auto* new_channel = new DataChannel(std::move(channel_name));
  data_channels.push_back(new_channel);
  data_collection_current_channel_list_rev++;
  return new_channel;
}

uint64_t data_collection_timestamp_offset = 0;

void data_collection_update_gps_time(uint64_t gps_time_us, uint64_t local_time_us) {
  if (data_collection_timestamp_offset == 0) {
    data_collection_timestamp_offset = (gps_time_us - local_time_us)*1000;
  }
}

void data_collection()
{


  spi_init(spi0, 2500000);
  gpio_set_function(SPI0_MISO, GPIO_FUNC_SPI);
  gpio_set_function(SPI0_SCK, GPIO_FUNC_SPI);
  gpio_set_function(SPI0_MOSI, GPIO_FUNC_SPI);


  XPT2046 xpt2046(spi0, 17);
  MAX11254 max11254_driver(spi0, 1, 0, 6);
  BS_280 bs_280{uart1, 8, 9};
  I2CBusManager i2c0_manager{i2c0, 21, 20};
  SystemDataSources system_data_sources;

  max11254_driver.init_device();
  max11254_driver.set_rate(8);

  absolute_time_t last_system_update = get_absolute_time();

  while (true)
  {
    xpt2046.update();
    max11254_driver.update();
    bs_280.update();
    i2c0_manager.update();
    if (absolute_time_diff_us(last_system_update, get_absolute_time()) > 100000)
    {
      system_data_sources.update();
      last_system_update = get_absolute_time();
    }
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
    data_still_available = false;
    for (auto channel : data_channels)
    {
      if (!channel->influx_export_buffer.is_empty)
      {
        auto data = channel->influx_export_buffer.pop_oldest();
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