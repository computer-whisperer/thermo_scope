
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "ili9341.h"
//#include "xpt2046.h"
#include "lvgl.h"
#include "xpt2046.h"
#include "data_collection.h"
#include "influxdb_export.h"
#include "system_data_sources.h"
#include "hardware/adc.h"
#include <pico/cyw43_arch.h>

void lv_example_style_13(void);
static lv_color_t buf1[240 * 320 / 10];
static lv_color_t buf2[240 * 320 / 10];
static lv_disp_draw_buf_t disp_buf;

static lv_disp_drv_t l_disp_drv;

static lv_indev_drv_t l_indev_drv;

int fuck_ctr = 0;
void print_cb(const char * buf)
{
  fuck_ctr++;
  printf("%s", buf);
}

void* channel_ref = NULL;

lv_obj_t * channel_select_dropdown_obj;
static void on_dropdown_select(lv_event_t* e)
{
  char option_name[80];
  lv_dropdown_get_selected_str(channel_select_dropdown_obj, option_name, sizeof(option_name));
  channel_ref = data_collection_get_channel_pointer(option_name);
}

int main()
{
  stdio_init_all();
  adc_init();
  adc_set_temp_sensor_enabled(true);

  cyw43_arch_init_with_country(CYW43_COUNTRY_USA);

  cyw43_arch_enable_sta_mode();
  cyw43_arch_wifi_connect_async("KalogonProdTest", "fwgnjEDjp7jd4pm9", CYW43_AUTH_WPA2_AES_PSK);

  multicore_launch_core1(data_collection);

  sleep_ms(100);

  uint32_t power_button_gpio = 27;

  gpio_init(power_button_gpio);
  gpio_set_dir(power_button_gpio, GPIO_IN);
  gpio_pull_up(power_button_gpio);

  ili9341_Init(LCD_LANDSCAPE);

  lv_init();

  lv_log_register_print_cb(print_cb);

  /*Initialize `disp_buf` with the buffer(s). With only one buffer use NULL instead buf_2 */
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, 240 * 320 / 10);

  lv_disp_drv_init(&l_disp_drv);
  l_disp_drv.draw_buf = &disp_buf;
  l_disp_drv.hor_res = lcd_Get_Width();
  l_disp_drv.ver_res = lcd_Get_Height();
  l_disp_drv.flush_cb = lcd_Flash_CB;
  lv_disp_drv_register(&l_disp_drv);

  lv_indev_drv_init(&l_indev_drv);
  l_indev_drv.type = LV_INDEV_TYPE_POINTER;
  l_indev_drv.read_cb = xpt2046_read_cb;
  lv_indev_drv_register(&l_indev_drv);



  lv_obj_t * chart_obj = NULL;
  int16_t chart_data[200];
  memset(&chart_data, 0, sizeof(chart_data));
  chart_obj = lv_chart_create(lv_scr_act());
  lv_chart_set_type(chart_obj, LV_CHART_TYPE_LINE);
  lv_chart_series_t * main_series = lv_chart_add_series(chart_obj, lv_palette_darken(LV_PALETTE_BLUE, 1), LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_ext_y_array(chart_obj, main_series, chart_data);
  lv_chart_set_update_mode(chart_obj, LV_CHART_UPDATE_MODE_SHIFT);
  lv_chart_set_point_count(chart_obj, 200);
  lv_chart_set_range(chart_obj, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
  //lv_obj_center(chart_obj);
  lv_obj_set_style_size(chart_obj, 0, LV_PART_INDICATOR);
  lv_obj_clear_flag(chart_obj, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_width(chart_obj, 320);
  lv_obj_set_height(chart_obj, 240);
  //lv_obj_set_style_bg_color(chart_obj, lv_palette_darken(LV_PALETTE_GREY, 1), LV_STATE_DEFAULT);

  lv_obj_t * chart_value_label_obj = lv_label_create(chart_obj);
  lv_obj_set_align(chart_value_label_obj, LV_ALIGN_BOTTOM_LEFT);

  lv_obj_t * wifi_state_label_obj = lv_label_create(chart_obj);
  lv_obj_set_align(wifi_state_label_obj, LV_ALIGN_BOTTOM_RIGHT);

  channel_select_dropdown_obj = lv_dropdown_create(chart_obj);
  lv_obj_set_align(channel_select_dropdown_obj, LV_ALIGN_TOP_LEFT);
  lv_obj_set_width(channel_select_dropdown_obj, 200);
  //lv_obj_add_event_cb(channel_select_dropdown_obj, on_dropdown_select, LV_EVENT_VALUE_CHANGED, NULL);

  uint32_t channel_names_rev = 0;

  channel_ref = data_collection_get_default_channel_pointer();
  channel_names_rev = data_collection_update_channel_names(channel_select_dropdown_obj, channel_names_rev);

  double min_val = 0.16575;
  double max_val = 0.16600;

  bool power_button_prev_state = false;
  bool apparent_power_state = true;

  absolute_time_t last_influx_connect_timestamp = nil_time;

  absolute_time_t prev_ui_cycle = get_absolute_time();

  absolute_time_t prev_chart_update = get_absolute_time();

  int prev_influxdb_state = 100;
  int prev_wifi_state = 100;

  while(1)
  {
    sleep_ms(1);

    // Update WIFI state
    {
      int wifi_state = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);

      if (wifi_state == CYW43_LINK_JOIN)
      {
        if (absolute_time_diff_us(last_influx_connect_timestamp, get_absolute_time()) > 10*1000000)
        {
          last_influx_connect_timestamp = get_absolute_time();
          influxdb_try_connect();
        }
      }
      int influxdb_state = influxdb_connection_state;

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
          case InfluxDBConnectionState_INVALID:
            influxdb_state_str = "Invalid";
            break;
          case InfluxDBConnectionState_DISCONNECTED:
            influxdb_state_str = "Disconnected";
            break;
          case InfluxDBConnectionState_CONNECTED:
            influxdb_state_str = "Connected";
            break;
          case InfluxDBConnectionState_CONNECTING:
            influxdb_state_str = "Connecting";
            break;
        }
        char buff[100];
        sprintf(buff, "WiFi: %s\n\rInflux: %s", wifi_state_str, influxdb_state_str);
        lv_label_set_text(wifi_state_label_obj, buff);
      }

      prev_influxdb_state = influxdb_state;
      prev_wifi_state = wifi_state;
    }

    lv_timer_handler();
    absolute_time_t ui_cycle = get_absolute_time();
    lv_tick_inc(absolute_time_diff_us(prev_ui_cycle, ui_cycle)/1000);
    prev_ui_cycle = ui_cycle;

    cyw43_arch_poll();

    data_collection_core0_process_samples();

    bool power_button_state = !gpio_get(power_button_gpio);
    if (!power_button_state && power_button_prev_state)
    {
      apparent_power_state = !apparent_power_state;
      if (apparent_power_state)
      {
        ili9341_SstLED(100);
      }
      else
      {
        ili9341_SstLED(0);
      }
    }
    power_button_prev_state = power_button_state;

    if (absolute_time_diff_us(prev_chart_update, get_absolute_time()) > 1000000)
    {
      if (channel_ref != NULL) {
        double data_buffer[200];
        uint32_t sample_count = data_collection_get_channel_chart_data(channel_ref, data_buffer, 200);

        memset(chart_data, 0, sizeof(chart_data));
        for (uint32_t i = 0; i < sample_count; i++) {
          double latest_value = data_buffer[i];
          double adjusted_value = (latest_value - min_val) / (max_val - min_val) * 1000;
          chart_data[i] = (int16_t)adjusted_value;
        }
        lv_chart_refresh(chart_obj);
        char buf[100];
        sprintf(buf, "%f V", data_buffer[0]);
        lv_label_set_text(chart_value_label_obj, buf);
      }
      prev_chart_update = get_absolute_time();
    }

    system_data_sources_core0_update();
  }
  return 0;
}
