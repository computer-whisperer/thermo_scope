//
// Created by christian on 9/15/23.
//

#include <cstring>
#include <array>
#include "bs_280.hpp"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "ring_buffer.hpp"
#include "hardware/sync.h"
#include "data_collection.hpp"

uart_inst_t * uart_dev = uart1;

static constexpr uint32_t tx_pin = 8;
static constexpr uint32_t rx_pin = 9;

static constexpr uint32_t max_msg_len = 256;


static RingBuffer<std::array<char, max_msg_len>> latest_full_messages{20};

static char rx_buffer[256];
static uint32_t rx_buffer_index = 0;

static void on_uart_rx()
{
  while (uart_is_readable(uart_dev)) {
    auto val = rx_buffer[rx_buffer_index++] = uart_getc(uart_dev);
    if (val == '\n' || rx_buffer_index == sizeof(rx_buffer))
    {
      auto dest = latest_full_messages.push_and_return_ptr();
      memcpy(dest->data(), &rx_buffer[0], rx_buffer_index);
      rx_buffer_index = 0;
    }
  }
}

BS_280::BS_280()
{
  uart_init(uart_dev, 9600);

  gpio_set_function(tx_pin, GPIO_FUNC_UART);
  gpio_set_function(rx_pin, GPIO_FUNC_UART);

  uart_set_hw_flow(uart_dev, false, false);

  uart_set_fifo_enabled(uart_dev, false);

  int UART_IRQ = uart_dev == uart0 ? UART0_IRQ : UART1_IRQ;

  irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
  irq_set_enabled(UART_IRQ, true);

  uart_set_irq_enables(uart_dev, true, false);

  critical_section_init(&critical_section);
}

int parse_two_digit_num(char* input)
{
  return (input[0] - '0') * 10 + (input[1] - '0');
}

uint64_t latest_gps_time = 0;

void BS_280::update() {
  bool found_msg = false;
  char msg_buffer[max_msg_len];
  critical_section_enter_blocking(&critical_section);
  for (auto message_buffer : latest_full_messages) {
    if (strncmp(message_buffer.data(), "$GPRMC", 6) == 0)
    {
      found_msg = true;
      memcpy(msg_buffer, message_buffer.data(), message_buffer.size());
      break;
    }
  }
  latest_full_messages.clear();
  critical_section_exit(&critical_section);
  if (found_msg)
  {
    // Parse NMEA message
    uint32_t field_num = 0;
    uint32_t msg_ptr = 0;

    bool found_time = false;
    bool found_date = false;

    int h;
    int min;
    int s;
    int ss;
    int d;
    int month;
    int y;

    while (msg_ptr < max_msg_len)
    {
      if (msg_buffer[msg_ptr] == ',')
      {
        // Handle field
        field_num++;
        if (msg_buffer[msg_ptr + 1] != ',')
        {
          // Field is not empty
          if (field_num == 1)
          {
            // Time (hhmmss.ss)
            h = parse_two_digit_num(msg_buffer + msg_ptr + 1);
            min = parse_two_digit_num(msg_buffer + msg_ptr + 3);
            s = parse_two_digit_num(msg_buffer + msg_ptr + 5);
            ss = parse_two_digit_num(msg_buffer + msg_ptr + 8);
            found_time = true;
          }

          if (field_num == 9)
          {
            // Date (ddmmyy)
            d = parse_two_digit_num(msg_buffer + msg_ptr + 1);
            month = parse_two_digit_num(msg_buffer + msg_ptr + 3);
            y = parse_two_digit_num(msg_buffer + msg_ptr + 5);
            found_date = true;
          }
        }
      }
      msg_ptr++;
    }

    if (found_time && found_date)
    {
      // Dirty, should fix in future
      uint64_t total_time_ms = 1672531200000; // Sun Jan 01 2023 00:00:00 GMT+0000
      int days_since_year_start = 0;
      switch (month-1)
      {
        case 12:
          days_since_year_start += 31;
        case 11:
          days_since_year_start += 30;
        case 10:
          days_since_year_start += 31;
        case 9:
          days_since_year_start += 30;
        case 8:
          days_since_year_start += 31;
        case 7:
          days_since_year_start += 31;
        case 6:
          days_since_year_start += 30;
        case 5:
          days_since_year_start += 31;
        case 4:
          days_since_year_start += 30;
        case 3:
          days_since_year_start += 31;
        case 2:
          days_since_year_start += 28;
        case 1:
          days_since_year_start += 31;
      }
      days_since_year_start += d-1;
      total_time_ms += (uint64_t)days_since_year_start * (uint64_t)(24 * 60 * 60 * 1000);
      total_time_ms += ((h*60 + min)*60 + s)*1000 + ss*10;
      latest_gps_time = total_time_ms;
      data_collection_update_gps_time(total_time_ms*1000, to_us_since_boot(get_absolute_time()));
    }
  }
}
