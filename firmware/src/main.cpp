
#include <bmp585.hpp>
#include <cyw43_shim.h>
#include <i2c_bus_manager.hpp>
#include <ili9341.hpp>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include <pico/cyw43_arch.h>
#include <malloc.h>
#include <sht45.hpp>
#include <st7789v2.hpp>
#include <system_data_sources.h>
#include <system_data_sources.hpp>
#include <tsys01.hpp>
#include <ublox_ubx.hpp>
#include <hardware/clocks.h>
#include <hardware/pll.h>
#include <hardware/watchdog.h>
#include <hardware/xosc.h>
#include <hardware/regs/clocks.h>

#include "influxdb_client.hpp"
#include "ntp_client.hpp"
#include "secrets.hpp"

InfluxDBClient influxdb_client{"general_telemetry"};

NTPClient ntp_client;

static lv_indev_drv_t l_indev_drv;



void xpt2046_read_cb(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data) {

}


static ST7789V2* st7789_v2;

class Display
{
private:
  lv_obj_t * press_label_obj = nullptr;
  lv_obj_t * temp_label_obj = nullptr;
  lv_obj_t * humid_label_obj = nullptr;
public:
  Display() {
    init();
  }

  void init() {
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
    lv_style_set_text_font(&style_value, &lv_font_montserrat_32);

    press_label_obj = lv_label_create(lv_scr_act());
    lv_obj_align(press_label_obj, LV_ALIGN_CENTER, 0, -60);
    lv_obj_add_style(press_label_obj, &style_value, LV_PART_MAIN);
    lv_label_set_long_mode(press_label_obj, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(press_label_obj, 200);

    temp_label_obj = lv_label_create(lv_scr_act());
    lv_obj_align(temp_label_obj, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_style(temp_label_obj, &style_value, LV_PART_MAIN);
    lv_label_set_long_mode(temp_label_obj, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(temp_label_obj, 200);

    humid_label_obj = lv_label_create(lv_scr_act());
    lv_obj_align(humid_label_obj, LV_ALIGN_CENTER, 0, 60);
    lv_obj_add_style(humid_label_obj, &style_value, LV_PART_MAIN);
    lv_label_set_long_mode(humid_label_obj, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(humid_label_obj, 200);
  }
  void update() {
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
};

bool power_wifi_power_state = false;

volatile bool data_collection_ready_for_dormant = true;
volatile bool data_collection_requesting_dormant = false;

TelemetryManager telemetry_manager;

absolute_time_t last_data_collection_update = nil_time;

UBLOX_UBX* ublox_ubx;

void data_collection()
{
  telemetry_manager.use_influxdb(&influxdb_client, "thermo_scope");

  data_collection_ready_for_dormant = false;
  ublox_ubx->initialize_device();
  ublox_ubx->use_assistnow_online(Secrets::assistnow_online_token);

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
      ublox_ubx->on_enter_dormant();
      data_collection_ready_for_dormant = true;
      while (data_collection_requesting_dormant)
      {
        sleep_ms(10);
      }
      ublox_ubx->on_exit_dormant();
      data_collection_ready_for_dormant = false;
    }

    ublox_ubx->update();
    i2c1_manager.update();
    if (absolute_time_diff_us(last_system_update, get_absolute_time()) > 100000)
    {
      system_data_sources.update();
      last_system_update = get_absolute_time();
    }
    last_data_collection_update = get_absolute_time();
    sleep_ms(1);
  }
}

void power_up_wifi()
{
  cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, 2000, 1, 1, 10);

  cyw43_arch_enable_sta_mode();
  cyw43_arch_wifi_connect_async(Secrets::kalogon_prod_test_ssid, Secrets::kalogon_prod_test_password, CYW43_AUTH_WPA2_AES_PSK);
  power_wifi_power_state = true;
  influxdb_client.connect();
}

void power_down_wifi()
{
  influxdb_client.disconnect();
  cyw43_arch_disable_sta_mode();
  power_wifi_power_state = false;
}

uint f_pll_sys;
uint f_pll_usb;
uint f_rosc;
uint f_clk_sys;
uint f_clk_peri;
uint f_clk_usb;
uint f_clk_adc;
uint f_clk_rtc;

void measure_clocks()
{
  f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
  f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
  f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
  f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
  f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
  f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
  f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
  f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);
}

uint32_t power_button_gpio = 27;

enum CLOCK_CONFIGURATION {
  CLOCK_CONFIGURATION_DEFAULT = 0,
  CLOCK_CONFIGURATION_HIGH_SPEED = 1,
  CLOCK_CONFIGURATION_DORMANT_READY = 3
};

void configure_clocks(enum CLOCK_CONFIGURATION config)
{
  uint32_t sys_clk_freq = 0;
  uint32_t rosc_speed = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
  switch (config)
  {
    case CLOCK_CONFIGURATION_DORMANT_READY:
      {
        uint32_t target_sys_clk = 2000;

        // switch sys to running directly from xosc
        if (false) {
          clock_configure(clk_sys,
                          CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                          CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_ROSC_CLKSRC,
                          rosc_speed * KHZ,
                          target_sys_clk * KHZ);
          clock_configure(clk_ref,
                          CLOCKS_CLK_REF_CTRL_SRC_VALUE_ROSC_CLKSRC_PH,
                          0,
                          rosc_speed * KHZ,
                          target_sys_clk * KHZ);
          xosc_disable();
        } else {
          clock_configure(clk_sys,
                          CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                          CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
                          XOSC_KHZ * KHZ,
                          target_sys_clk * KHZ);
        }
      }
      //pll_deinit(pll_usb);
      //pll_deinit(pll_sys);

      break;
    case CLOCK_CONFIGURATION_HIGH_SPEED:
      pll_init(pll_sys, 1, (1500 * 1000 * KHZ), 6, 2);
      sys_clk_freq = 125000 * KHZ;

      clock_configure(clk_sys,
                      CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                      CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                      sys_clk_freq,
                      sys_clk_freq);
      break;

  }
  sleep_ms(1);
  measure_clocks();
}


void enter_dormant()
{
  gpio_set_dormant_irq_enabled(power_button_gpio, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS, true);
  configure_clocks(CLOCK_CONFIGURATION_DORMANT_READY);

  while(false)
  {
    tight_loop_contents();
  }
  //rosc_hw->dormant = 0x636f6d61;
  xosc_dormant();

  configure_clocks(CLOCK_CONFIGURATION_HIGH_SPEED);
  gpio_acknowledge_irq(power_button_gpio, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS);
}

int main()
{
  stdio_init_all();

  if (watchdog_enable_caused_reboot()) {
    printf("Watchdog reset!\r\n");
  }

  printf("Booting!\r\n");

  watchdog_enable(4000, true);

  configure_clocks(CLOCK_CONFIGURATION_HIGH_SPEED);

  adc_init();
  adc_set_temp_sensor_enabled(true);


  cyw43_arch_init_with_country(CYW43_COUNTRY_USA);

  ublox_ubx = new UBLOX_UBX(uart1, 8, 9, 10);

  multicore_launch_core1(data_collection);

  bool display_power_state = true;

  bool enter_dormant_when_can = false;

  gpio_init(power_button_gpio);
  gpio_set_dir(power_button_gpio, GPIO_IN);
  gpio_pull_up(power_button_gpio);

  absolute_time_t power_button_time = nil_time;

  sleep_ms(100);

  power_up_wifi();
  display_power_state = true;

  Display display;

  while (true)
  {
    influxdb_client.update();
    ntp_client.update();
    ublox_ubx->lwip_update();

    if (enter_dormant_when_can && data_collection_ready_for_dormant)
    {
      enter_dormant();
      // Just exited dormant!
      power_button_time = get_absolute_time();
      enter_dormant_when_can = false;
      power_up_wifi();
      display_power_state = true;
    }

    data_collection_requesting_dormant = enter_dormant_when_can;


    telemetry_manager.push_data_to_influxdb();

    if (power_wifi_power_state)
    {
      cyw43_arch_poll();
    }

    if (display_power_state)
    {
      //ili9341_SstLED(100);
    }
    else
    {
      //ili9341_SstLED(0);
    }

    display.update();

    if (absolute_time_diff_us(power_button_time, get_absolute_time()) > 1000000)
    {
      if (!gpio_get(power_button_gpio))
      {
        enter_dormant_when_can = true;
        power_button_time = get_absolute_time();
        power_down_wifi();
        display_power_state = false;
      }
    }

    static absolute_time_t last_influx_connect_timestamp = nil_time;
    // Update WIFI state
    if (power_wifi_power_state)
    {

      int wifi_state = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);

      if (wifi_state == CYW43_LINK_JOIN)
      {
        if (absolute_time_diff_us(last_influx_connect_timestamp, get_absolute_time()) > 10*1000000)
        {
          last_influx_connect_timestamp = get_absolute_time();
        }
      }
    }

    system_data_sources_core0_update();

    if (absolute_time_diff_us(last_data_collection_update, get_absolute_time()) < 100000) {
      watchdog_update();
    }

    sleep_ms(10);
  }
};
