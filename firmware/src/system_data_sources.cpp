//
// Created by christian on 9/15/23.
//

#include <malloc.h>
#include "system_data_sources.hpp"
#include "system_data_sources.h"
#include "power_status.h"
#include "hardware/adc.h"

static float last_pushed_vbat = 0;
static volatile float vbat = 0;

static float last_pushed_rp2040_temp = 0;
static volatile float rp2040_temp = 0;

static absolute_time_t last_vbat_read = nil_time;

static int32_t prev_accumulated_error = 0;


void SystemDataSources::update() {
  //sys_time_channel->push_new_value((double)to_us_since_boot(get_absolute_time()));
  if (vbat != last_pushed_vbat) {
    battery_voltage_channel.new_data(vbat);
    last_pushed_vbat = vbat;
  }
  if (rp2040_temp!= last_pushed_rp2040_temp) {
    rp2040_temp_channel.new_data(rp2040_temp);
    last_pushed_rp2040_temp = rp2040_temp;
  }
  auto malloc_info = mallinfo();
  heap_space_free.new_data((double)malloc_info.fordblks);
/*
  if (data_collection_time_error_since_last_sync_us != prev_accumulated_error)
  {
    sys_time_channel.new_data((double)data_collection_time_error_since_last_sync_us);
  }*/
}


/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */
float read_onboard_temperature(const char unit) {

  /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
  const float conversionFactor = 3.3f / (1 << 12);

  adc_select_input(4);
  float adc = (float)adc_read() * conversionFactor;
  float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

  if (unit == 'C') {
    return tempC;
  } else if (unit == 'F') {
    return tempC * 9 / 5 + 32;
  }

  return -1.0f;
}

void system_data_sources_core0_update() {
  if (absolute_time_diff_us(last_vbat_read, get_absolute_time()) > 100000000)
  {
    power_voltage((float*)&vbat);
    rp2040_temp = read_onboard_temperature('C');
    last_vbat_read = get_absolute_time();
  }
}
