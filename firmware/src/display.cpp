//
// Created by christian on 9/17/23.
//
#include <src/core/lv_obj.h>
#include <cstdio>
#include "display.hpp"

#include <st7789v2.hpp>
#include <telemetry_manager.hpp>
#include <time_manager.hpp>

#include "lvgl.h"
#include "data_collection.h"
#include "pico/time.h"
#include "main.hpp"
#include "cyw43_ll.h"
#include "cyw43.h"
#include "cyw43_shim.h"
#include "data_collection.hpp"


static lv_indev_drv_t l_indev_drv;

Display::Display() {
  init();
}

void xpt2046_read_cb(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data) {

}
static ST7789V2* st7789_v2;
void Display::init() {
  lv_init();

  spi_init(spi1,  12*1000 * 1000);
  gpio_set_function(12, GPIO_FUNC_SPI);
  gpio_set_function(11, GPIO_FUNC_SPI);
  gpio_set_function(14, GPIO_FUNC_SPI);
  st7789_v2 = new ST7789V2(spi1, 13, 26, 22, 16);
  st7789_v2->register_with_lvgl();

  lv_indev_drv_init(&l_indev_drv);
  l_indev_drv.type = LV_INDEV_TYPE_POINTER;
  l_indev_drv.read_cb = xpt2046_read_cb;
  lv_indev_drv_register(&l_indev_drv);

  static lv_style_t style_value;
  lv_style_init(&style_value);
  lv_style_set_text_font(&style_value, &lv_font_montserrat_22);

  press_label_obj = lv_label_create(lv_scr_act());
  lv_obj_align(press_label_obj, LV_ALIGN_CENTER, 0, -30);
  lv_obj_add_style(press_label_obj, &style_value, LV_PART_MAIN);

  temp_label_obj = lv_label_create(lv_scr_act());
  lv_obj_align(temp_label_obj, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_style(temp_label_obj, &style_value, LV_PART_MAIN);

  humid_label_obj = lv_label_create(lv_scr_act());
  lv_obj_align(humid_label_obj, LV_ALIGN_CENTER, 0, 30);
  lv_obj_add_style(humid_label_obj, &style_value, LV_PART_MAIN);
}

void Display::update() {

  static absolute_time_t prev_ui_cycle = get_absolute_time();

  if (absolute_time_diff_us(prev_ui_cycle, get_absolute_time()) > 5000)
  {
    lv_timer_handler();
    absolute_time_t ui_cycle = get_absolute_time();
    lv_tick_inc(absolute_time_diff_us(prev_ui_cycle, ui_cycle)/1000);
    prev_ui_cycle = ui_cycle;
  }

  static int prev_influxdb_state = 100;
  static int prev_wifi_state = 100;
  int wifi_state = cyw43_shim_wifi_link_status(CYW43_ITF_STA);
  int influxdb_state = influxdb_client.connected;
  if ((prev_influxdb_state != influxdb_state) || (prev_wifi_state!= wifi_state))
  {
    const char * wifi_state_str;
    const char * influxdb_state_str;
    switch(wifi_state) {
      case CYW43_LINK_DOWN:
        wifi_state_str = "Disconnected";
        break;
      case CYW43_LINK_JOIN:
        wifi_state_str = "Connected";
        break;
      case CYW43_LINK_FAIL:
        wifi_state_str = "Failed";
        break;
      case CYW43_LINK_NONET:
        wifi_state_str = "No Network";
        break;
      case CYW43_LINK_BADAUTH:
        wifi_state_str = "Bad Auth";
        break;
      default:
        break;
    }
    switch(influxdb_state)
    {
      case 0:
        influxdb_state_str = "Disconnected";
        break;
      case 1:
        influxdb_state_str = "Connected";
        break;
    }
    char buff[100];
    snprintf(buff, sizeof(buff), "WiFi: %s\n\rInflux: %s", wifi_state_str, influxdb_state_str);
    //lv_label_set_text(wifi_state_label_obj, buff);
  }
  prev_wifi_state = wifi_state;
  prev_influxdb_state = influxdb_state;

  static absolute_time_t prev_value_update = get_absolute_time();
  if (absolute_time_diff_us(prev_value_update, get_absolute_time()) > 20000)
  {
    prev_value_update = get_absolute_time();
    char buff[100];
    snprintf(buff, sizeof(buff), "%.03f kPa", TelemetryManager::get_best_pressure_kpa());
    lv_label_set_text(press_label_obj, buff);
    snprintf(buff, sizeof(buff), "%.03f C", TelemetryManager::get_best_temperature_c());
    lv_label_set_text(temp_label_obj, buff);
    snprintf(buff, sizeof(buff), "%.03f %%", TelemetryManager::get_best_humidity_rh());
    lv_label_set_text(humid_label_obj, buff);
  }
}

