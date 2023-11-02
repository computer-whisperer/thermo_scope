//
// Created by christian on 9/17/23.
//

#include <cstdint>
#include "power.h"
#include "hardware/gpio.h"
#include "cyw43_ll.h"
#include "data_collection.h"
#include "pico/cyw43_arch.h"
#include "system_data_sources.h"
#include "ili9341.hpp"
#include "influxdb_export.h"
#include "display.hpp"
#include "hardware/structs/clocks.h"
#include "hardware/clocks.h"
#include "hardware/resets.h"
#include "hardware/pll.h"
#include "pico/multicore.h"
#include "hardware/xosc.h"
#include "data_collection.hpp"
#include "hardware/adc.h"
#include "hardware/structs/rosc.h"
#include "hardware/vreg.h"

bool power_wifi_power_state = false;

void power_up_wifi()
{

  cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, 2000, 1, 1, 10);

  cyw43_arch_enable_sta_mode();
  cyw43_arch_wifi_connect_async("KalogonProdTest", "fwgnjEDjp7jd4pm9", CYW43_AUTH_WPA2_AES_PSK);
  power_wifi_power_state = true;
}

void power_down_wifi()
{
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
  CLOCK_CONFIGURATION_LOW_POWER = 2,
  CLOCK_CONFIGURATION_DORMANT_READY = 3
};

void configure_clocks(enum CLOCK_CONFIGURATION config)
{
  uint32_t sys_clk_freq = 0;
  uint32_t rosc_speed = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
  switch (config)
  {
    case CLOCK_CONFIGURATION_LOW_POWER:
    case CLOCK_CONFIGURATION_DEFAULT:
      pll_init(pll_sys, PLL_COMMON_REFDIV, PLL_SYS_VCO_FREQ_KHZ * KHZ, PLL_SYS_POSTDIV1, PLL_SYS_POSTDIV2);
      sys_clk_freq = SYS_CLK_KHZ * KHZ;

      // Slow down sys
      clock_configure(clk_sys,
                      CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                      CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                      sys_clk_freq,
                      sys_clk_freq);
      clock_configure(clk_adc,
                      0, // No GLMUX
                      CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                      sys_clk_freq,
                      sys_clk_freq);
      clock_configure(clk_peri,
                      0,
                      CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                      sys_clk_freq,
                      sys_clk_freq);
      break;
    case CLOCK_CONFIGURATION_DORMANT_READY:
      {
        uint32_t target_sys_clk = 100;

        // switch sys to running directly from xosc
        if (true)
        {
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
        }
        else
        {
          clock_configure(clk_sys,
                          CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                          CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
                          XOSC_KHZ * KHZ,
                          target_sys_clk * KHZ);
        }
      }
      //clock_stop(clk_gpout0);
      //clock_stop(clk_gpout1);
      //clock_stop(clk_gpout2);
      //clock_stop(clk_gpout3);
      clock_stop(clk_adc);
      clock_stop(clk_peri);
      clock_stop(clk_usb);
      clock_stop(clk_rtc);
      pll_deinit(pll_usb);
      pll_deinit(pll_sys);

      reset_block(
              RESETS_RESET_USBCTRL_BITS |
              RESETS_RESET_UART0_BITS |
              RESETS_RESET_UART1_BITS |
              RESETS_RESET_SPI0_BITS |
              RESETS_RESET_SPI1_BITS |
              RESETS_RESET_RTC_BITS |
              RESETS_RESET_PWM_BITS |
              RESETS_RESET_PLL_USB_BITS |
              RESETS_RESET_PLL_SYS_BITS |
              RESETS_RESET_PIO0_BITS |
              RESETS_RESET_PIO1_BITS |
              RESETS_RESET_ADC_BITS
      );

      break;
    case CLOCK_CONFIGURATION_HIGH_SPEED:
      pll_init(pll_sys, 1, (1500 * KHZ), 6, 2);
      sys_clk_freq = 125000 * KHZ;

      // Slow down sys
      clock_configure(clk_sys,
                      CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                      CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                      sys_clk_freq,
                      sys_clk_freq);
      /*
      clock_configure(clk_adc,
                      0, // No GLMUX
                      CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                      sys_clk_freq,
                      sys_clk_freq/4);*/
      clock_configure(clk_peri,
                      0,
                      CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                      sys_clk_freq,
                      sys_clk_freq);
      break;

  }



  sleep_ms(1);
  measure_clocks();
}


uint32_t debug_gpio = 5;

void enter_dormant()
{
  gpio_set_dormant_irq_enabled(power_button_gpio, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS, true);
  configure_clocks(CLOCK_CONFIGURATION_DORMANT_READY);

  while(false)
  {
    tight_loop_contents();
  }
  gpio_put(debug_gpio, true);
  rosc_hw->dormant = 0x636f6d61;
  //xosc_dormant();

  //configure_clocks(CLOCK_CONFIGURATION_HIGH_SPEED);
  gpio_acknowledge_irq(power_button_gpio, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS);
}



void power_main_loop()
{
  gpio_init(debug_gpio);
  gpio_set_dir(debug_gpio, GPIO_OUT);

  //vreg_set_voltage(VREG_VOLTAGE_0_95);

/*
  reset_block(
          RESETS_RESET_USBCTRL_BITS |
          RESETS_RESET_PWM_BITS |
          RESETS_RESET_RTC_BITS |
          RESETS_RESET_PIO0_BITS |
          RESETS_RESET_UART0_BITS); // hold unneeded peripherals in reset
          */
  //configure_clocks(CLOCK_CONFIGURATION_LOW_POWER);

  adc_init();
  adc_set_temp_sensor_enabled(true);


  cyw43_arch_init_with_country(CYW43_COUNTRY_USA);

  multicore_launch_core1(data_collection);

  bool display_power_state = false;

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

    data_collection_core0_process_samples();

    if (power_wifi_power_state)
    {
      cyw43_arch_poll();
    }

    if (display_power_state)
    {
      ili9341_SstLED(100);
    }
    else
    {
      ili9341_SstLED(0);
    }

    //display.update();

    if (false && absolute_time_diff_us(power_button_time, get_absolute_time()) > 1000000)
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
          influxdb_try_connect();
        }
      }
    }

    system_data_sources_core0_update();

    //sleep_ms(10);
  }
}