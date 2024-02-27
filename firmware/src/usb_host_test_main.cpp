//
// Created by christian on 1/7/24.
//

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
#include <pico/unique_id.h>

#include "influxdb_client.hpp"
#include "ntp_client.hpp"
#include "secrets.hpp"

#include "pio_usb.h"
static usb_device_t *usb_device = NULL;

InfluxDBClient influxdb_client{"general_telemetry"};

NTPClient ntp_client;

bool power_wifi_power_state = false;

volatile bool data_collection_ready_for_dormant = true;
volatile bool data_collection_requesting_dormant = false;

TelemetryManager telemetry_manager;

absolute_time_t last_data_collection_update = nil_time;


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



void core1_main() {
  sleep_ms(10);
  printf("Core 1 out\r\n");

  // To run USB SOF interrupt in core1, create alarm pool in core1.
  static pio_usb_configuration_t config = PIO_USB_DEFAULT_CONFIG;
  config.alarm_pool = (void*)alarm_pool_create(2, 1);
  config.pin_dp = 2;
  config.pinout = PIO_USB_PINOUT_DPDM;
  usb_device = pio_usb_host_init(&config);

  //// Call pio_usb_host_add_port to use multi port
  // const uint8_t pin_dp2 = 8;
  // pio_usb_host_add_port(pin_dp2);

  while (true) {
    pio_usb_host_task();
  }
}

int main()
{
  // default 125MHz is not appropreate. Sysclock should be multiple of 12MHz.
  set_sys_clock_khz(120000, true);

  stdio_init_all();

  if (watchdog_enable_caused_reboot()) {
    printf("Watchdog reset!\r\n");
  }

  printf("Booting!\r\n");

  //watchdog_enable(10000, true);

  telemetry_manager.update_tags(",nick=usb_test_host");


  multicore_reset_core1();
  // all USB task run in core1
  multicore_launch_core1(core1_main);

  adc_init();
  adc_set_temp_sensor_enabled(true);


  cyw43_arch_init_with_country(CYW43_COUNTRY_USA);

  sleep_ms(100);

  power_up_wifi();

  while (true)
  {
    //influxdb_client.update();
    //ntp_client.update();
    //ublox_ubx->lwip_update();

    //telemetry_manager.push_data_to_influxdb();

    if (power_wifi_power_state)
    {
      cyw43_arch_poll();
    }

    //system_data_sources_core0_update();

    watchdog_update();

    if (usb_device != NULL) {
      for (int dev_idx = 0; dev_idx < PIO_USB_DEVICE_CNT; dev_idx++) {
        usb_device_t *device = &usb_device[dev_idx];
        if (!device->connected) {
          continue;
        }

        // Print received packet to EPs
        for (int ep_idx = 0; ep_idx < PIO_USB_DEV_EP_CNT; ep_idx++) {
          endpoint_t *ep = pio_usb_get_endpoint(device, ep_idx);

          if (ep == NULL) {
            break;
          }

          uint8_t temp[64];
          int len = pio_usb_get_in_data(ep, temp, sizeof(temp));

          if (len > 0) {
            printf("%04x:%04x EP 0x%02x:\t", device->vid, device->pid,
                   ep->ep_num);
            for (int i = 0; i < len; i++) {
              printf("%02x ", temp[i]);
            }
            printf("\n");
          }
        }
      }
    }
    stdio_flush();
    sleep_ms(10);
  }
};
