//
// Created by christian on 9/17/23.
//

#include <cstring>
#include <string>
#include "ublox_g7.hpp"
#include "hardware/gpio.h"
#include "data_collection.hpp"

static constexpr uint32_t max_msg_len = 100;


static RingBuffer<std::array<uint8_t, max_msg_len>> latest_full_messages{20};

volatile static char rx_buffer[max_msg_len];
volatile static uint32_t rx_buffer_index = 0;
volatile static uint32_t key_characters_found = 0;

static uart_inst_t * isr_uart_dev = nullptr;

static critical_section_t critical_section;


static uint64_t pps_timestamp = 0;

static void pps_gpio_isr(uint gpio, uint32_t event_mask)
{
  if (event_mask & GPIO_IRQ_EDGE_RISE) {
    pps_timestamp = to_us_since_boot(get_absolute_time());
  }
}

static void clear_message_buffer()
{
  critical_section_enter_blocking(&critical_section);
  latest_full_messages.clear();
  rx_buffer_index = 0;
  key_characters_found = 0;
  critical_section_exit(&critical_section);
}

static void on_uart_rx()
{
  while (uart_is_readable(isr_uart_dev)) {
    auto val = rx_buffer[rx_buffer_index++] = uart_getc(isr_uart_dev);
    if (rx_buffer_index >= sizeof(rx_buffer)) {
      rx_buffer_index = 0;
    }
    else if ((val == '$' || val == 0xB5) && (rx_buffer_index > 1))
    {
      key_characters_found++;
      auto dest = latest_full_messages.push_and_return_ptr();
      for (uint32_t i = 0; i < rx_buffer_index-1; i++)
      {
        dest->data()[i] = rx_buffer[i];
      }
      rx_buffer[0] = rx_buffer[rx_buffer_index-1];
      rx_buffer_index = 1;
    }
  }
}

UBLOX_G7::UBLOX_G7(uart_inst_t *uart_dev_in, uint32_t tx_gpio_in, uint32_t rx_gpio_in, uint32_t pps_gpio_in):
        uart_dev(uart_dev_in),
        tx_gpio(tx_gpio_in),
        rx_gpio(rx_gpio_in),
        pps_gpio(pps_gpio_in)
{
  uart_init(uart_dev, 9600);

  gpio_set_function(tx_gpio, GPIO_FUNC_UART);
  gpio_set_function(rx_gpio, GPIO_FUNC_UART);

  gpio_init(pps_gpio);
  gpio_set_dir(pps_gpio, GPIO_IN);
  gpio_set_irq_enabled_with_callback(pps_gpio, GPIO_IRQ_EDGE_RISE, true, pps_gpio_isr);

  uart_set_hw_flow(uart_dev, false, false);

  uart_set_fifo_enabled(uart_dev, false);

  int UART_IRQ = uart_dev == uart0 ? UART0_IRQ : UART1_IRQ;

  isr_uart_dev = uart_dev;
  irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
  irq_set_enabled(UART_IRQ, true);

  uart_set_irq_enables(uart_dev, true, false);

  critical_section_init(&critical_section);
}

#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_RXM 0x02
#define UBX_CLASS_INF 0x04
#define UBX_CLASS_ACK 0x05
#define UBX_CLASS_CFG 0x06
#define UBX_CLASS_UPD 0x09
#define UBX_CLASS_MON 0x0A
#define UBX_CLASS_AID 0x0B
#define UBX_CLASS_TIM 0x0D
#define UBX_CLASS_ESF 0x10
#define UBX_CLASS_MGA 0x13
#define UBX_CLASS_LOG 0x21
#define UBX_CLASS_SEC 0x27

#define UBX_CFG_PRT 0x00
#define UBX_CFG_PM2 0x3B
#define UBX_CFG_PMS 0x86
#define UBX_CFG_RXM 0x11
#define UBX_CFG_TP5 0x31
#define UBX_CFG_MSG 0x01
#define UBX_TIM_TP 0x01
#define UBX_CFG_GNSS 0x3E
#define UBX_CFG_CFG 0x09




// Apparently we have protocol version 14.00 on hand

void UBLOX_G7::initialize_device() {
  uint32_t current_rate = discover_device_baudrate();

  if (true || current_rate != 115200)
  {
    // Module must be newly configured
    // Set device baudrate to 115200
    uint8_t payload[44];
    memset(payload, 0, sizeof(payload));
    payload[0] = 1; // IO port 1
    *((uint16_t*)&payload[2]) = 0x0000; // txReady
    *((uint32_t*)&payload[4]) = 0x000008C0; // mode
    *((uint32_t*)&payload[8]) = 115200; // baudrate
    *((uint16_t*)&payload[12]) = 0x0001; // inProtoMask
    *((uint16_t*)&payload[14]) = 0x0001; // outProtoMask
    *((uint16_t*)&payload[16]) = 0x0000; // flags
    send_ubx(UBX_CLASS_CFG, UBX_CFG_PRT, payload, 20);

    // Wait for message to send and process
    sleep_ms(50);

    uart_set_baudrate(uart_dev, 115200);
    clear_message_buffer();

    // Configure TP messages
    memset(payload, 0, sizeof(payload));
    payload[0] = 0; // TP0
    *((uint16_t*)&payload[4]) = 50; // antCableDelay
    *((uint16_t*)&payload[6]) = 0; // rfGroupDelay
    *((uint32_t*)&payload[8]) = 1000000; // freqPeriod;
    *((uint32_t*)&payload[12]) = 1000000; // freqPeriodLock;
    *((uint32_t*)&payload[16]) = 0; // pulseLenRatio;
    *((uint32_t*)&payload[20]) = 100000; // pulseLenRatioLock;
    *((int32_t*)&payload[24]) = 0; // userConfigDelay;
    *((uint32_t*)&payload[28]) = 0x00000077; // flags;
    send_ubx(UBX_CLASS_CFG, UBX_CFG_TP5, payload, 32);

    // Enable TIM-TP
    memset(payload, 0, sizeof(payload));
    payload[0] = UBX_CLASS_TIM;
    payload[1] = UBX_TIM_TP;
    payload[3] = 1; // Set rate to 1 on port 1
    send_ubx(UBX_CLASS_CFG, UBX_CFG_MSG, payload, 8);


    // Save settings
    memset(payload, 0, sizeof(payload));
    *((uint32_t*)&payload[4]) = 0x0000061F;
    //*((uint32_t*)&payload[4]) = 0xFFFFFFFF;
    send_ubx(UBX_CLASS_CFG, UBX_CFG_CFG, payload, 12);
  }



}

void UBLOX_G7::send_ubx(uint8_t msg_class, uint8_t msg_id, uint8_t *payload, uint16_t payload_len)
{
  uint32_t total_len = 8+payload_len;
  uint8_t buf[total_len];
  buf[0] = 0xB5;
  buf[1] = 0x62;
  buf[2] = msg_class;
  buf[3] = msg_id;
  buf[4] = payload_len&0xFF;
  buf[5] = payload_len>>8;
  memcpy(&buf[6], payload, payload_len);
  ubx_csum(buf+2, payload_len+4, buf + payload_len + 6, buf + payload_len + 7);
  uart_write_blocking(uart_dev, buf, total_len);
}

void UBLOX_G7::ubx_csum(uint8_t *data, uint16_t data_len, uint8_t* ck_a_out, uint8_t* ck_b_out) {
  *ck_a_out = 0;
  *ck_b_out = 0;
  for (uint16_t i = 0; i < data_len; i++) {
    *ck_a_out += data[i];
    *ck_b_out += *ck_a_out;
  }
}
void UBLOX_G7::handle_message(uint8_t *data, uint16_t data_len) {
  if (data_len < 4)
  {
    return;
  }
  if (data[0] == 0xB5)
  {
    // Validate UBX message
    if ((data[1] != 0x62) || (data_len < 8))
    {
      return;
    }
    uint16_t payload_len = data[4] | (((uint16_t)data[5])<<8);
    if (payload_len+4 > data_len)
    {
      return; // Invalid message
    }
    uint8_t calculated_ck_a, calculated_ck_b;
    ubx_csum(data+2, payload_len+4, &calculated_ck_a, &calculated_ck_b);
    if ((data[payload_len+6] != calculated_ck_a) || (data[payload_len+7]!= calculated_ck_b))
    {
      return;
    }
    handle_ubx_message(data[2], data[3], data+6, payload_len);
  }
  if (data[0] == '$')
  {
    // NMEA message
    // Not implemented ATM
  }
}

void UBLOX_G7::handle_ubx_message(uint8_t msg_class, uint8_t msg_id, uint8_t *payload, uint16_t payload_len) {
  // Align payload
  uint8_t aligned_buf[payload_len];
  memcpy(aligned_buf, payload, payload_len);
  if (msg_class == UBX_CLASS_TIM)
  {
    if (msg_id == UBX_TIM_TP)
    {
      uint32_t towMS = *(uint32_t*)(aligned_buf + 0);
      uint32_t towSubMS = *(uint32_t*)(aligned_buf + 4);
      int32_t qErr = *(int32_t*)(aligned_buf + 8);
      uint16_t week = *(uint16_t*)(aligned_buf + 12);
      uint8_t flags = *(uint8_t*)(aligned_buf + 14);

      uint64_t timestamp_us = 315964766000000 + week*604800000000 + towMS*(uint64_t)1000;
      most_recent_timestamp_seen = timestamp_us;
    }
  }
}


uint32_t UBLOX_G7::discover_device_baudrate() {
  uint32_t baudrates_to_try[] = {115200, 9600};
  uint32_t current_rate;

  for (auto baudrate : baudrates_to_try) {
    current_rate = baudrate;
    uart_set_baudrate(uart_dev, baudrate);
    clear_message_buffer();
    uint8_t payload[1] = {0x01};
    send_ubx(UBX_CLASS_CFG, UBX_CFG_PRT, payload, sizeof(payload));
    bool found = false;
    for (uint32_t i = 0; i < 100; i++) {
      sleep_ms(1);
      if (!latest_full_messages.is_empty) {
        found = true;
        break;
      }
    }
    if (found) {
      // Wait for the rx message to complete
      sleep_ms(100);
      break;
    }
  }
  return current_rate;
}

void UBLOX_G7::send_nmea(std::string body) {
  uint8_t cksum = 0;
  for (auto c : body) {
    cksum ^= c;
  }
  char hex_chars[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  char cksum_buf[3] = {hex_chars[cksum&0x0F], hex_chars[(cksum>>4)&0x0F], 0x00};
  std::string full_msg = "$" + body + "*" + cksum_buf + "\r\n";
  uart_write_blocking(uart_dev, (uint8_t*)full_msg.c_str(), full_msg.length());
}

void UBLOX_G7::update() {
  bool were_there_messages = false;
  uint64_t prev_most_recent_timestamp_seen = most_recent_timestamp_seen;
  critical_section_enter_blocking(&critical_section);
  for (auto msg : latest_full_messages)
  {
    were_there_messages = true;
    handle_message(msg.data(), msg.size());
  }
  latest_full_messages.clear();
  critical_section_exit(&critical_section);
  if ((most_recent_timestamp_seen!= prev_most_recent_timestamp_seen) && (pps_timestamp != 0)) {

    // Sanity check for recent pps timestamp
    if (!is_going_to_sleep && (to_us_since_boot(get_absolute_time()) - pps_timestamp) < 100*1000)
    {
      // New time sync!
      data_collection_update_gps_time(most_recent_timestamp_seen, pps_timestamp);
    }
  }

  // Check if it's time to sleep
  if (do_power_save &&
      were_there_messages &&
      !is_nil_time(data_collection_most_recent_gps_time_sync) &&
      (absolute_time_diff_us(data_collection_most_recent_gps_time_sync, get_absolute_time()) < 60*1000000) &&
      (is_nil_time(last_sleep_command_time) || (absolute_time_diff_us(last_sleep_command_time, get_absolute_time()) > 60*1000000)) &&
      data_collection_recent_time_updates > 20)
  {
    start_sleep();
  }

  // Check if it's time to wake (if it has been an hour since the last time sync)
  if (do_power_save &&
      !were_there_messages &&
      (absolute_time_diff_us(data_collection_most_recent_gps_time_sync, get_absolute_time()) > 60*60*1000000LL) &&
      (absolute_time_diff_us(last_wake_command_time, get_absolute_time()) > 60*1000000LL))
  {
    start_wake();
  }
}

void UBLOX_G7::on_enter_dormant() {
  start_sleep();
}

void UBLOX_G7::on_exit_dormant() {
  start_wake();
}

void UBLOX_G7::start_sleep() {
  uint8_t payload[44];
  // Send sleep config/command
  memset(payload, 0, sizeof(payload));
  payload[0] = 0x01; // Version
  // This defaults to 0x00029000
  *((uint32_t*)&payload[4]) = 0x00000000; // PSM config flags
  *((uint32_t*)&payload[8]) = 0; // Update period
  *((uint32_t*)&payload[12]) = 5000; // Search period
  *((uint32_t*)&payload[16]) = 0; // gridOffset
  *((uint16_t*)&payload[20]) = 0; // onTime
  *((uint16_t*)&payload[22]) = 0; // minAcqTime
  send_ubx(UBX_CLASS_CFG, UBX_CFG_PM2, payload, 44);

  // Enter power save mode
  memset(payload, 0, sizeof(payload));
  payload[0] = 0x08; // Always set for some reason
  payload[1] = 0x01; // Low power mode
  send_ubx(UBX_CLASS_CFG, UBX_CFG_RXM, payload, 2);

  last_sleep_command_time = get_absolute_time();
  is_going_to_sleep = true;
}

void UBLOX_G7::start_wake() {
  uint8_t payload = 0xFF;
  uart_write_blocking(uart_dev, &payload, sizeof(payload));
  is_going_to_sleep = false;

  last_wake_command_time = get_absolute_time();
}

