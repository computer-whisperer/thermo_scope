//
// Created by christian on 9/17/23.
//

#ifndef THERMO_SCOPE_UBLOX_G7_HPP
#define THERMO_SCOPE_UBLOX_G7_HPP

#include "hardware/uart.h"
#include "ring_buffer.hpp"
#include <pico/sync.h>
#include <array>

class UBLOX_G7 {
  uart_inst_t * uart_dev;
  uint32_t tx_gpio;
  uint32_t rx_gpio;
  uint32_t pps_gpio;
  absolute_time_t last_sleep_command_time = nil_time;
  absolute_time_t last_wake_command_time = nil_time;
  bool do_power_save = false;
  bool is_going_to_sleep = false;

  uint64_t most_recent_timestamp_seen = 0;
public:
  UBLOX_G7(uart_inst_t * uart_dev_in, uint32_t tx_gpio_in, uint32_t rx_gpio_in, uint32_t pps_gpio_in);

  void send_ubx(uint8_t msg_class, uint8_t msg_id, uint8_t* payload, uint16_t payload_len);
  void send_nmea(std::string body);

  void ubx_csum(uint8_t *data, uint16_t data_len, uint8_t* ck_a_out, uint8_t* ck_b_out);

  uint32_t discover_device_baudrate();

  void initialize_device();

  void update();

  void handle_message(uint8_t* data, uint16_t data_len);
  void handle_ubx_message(uint8_t msg_class, uint8_t msg_id, uint8_t* payload, uint16_t payload_len);

  void start_sleep();
  void start_wake();

  void on_enter_dormant();
  void on_exit_dormant();
};

#endif //THERMO_SCOPE_UBLOX_G7_HPP
