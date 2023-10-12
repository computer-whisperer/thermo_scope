//
// Created by christian on 6/7/23.
//

#include <cstdio>
#include "max11254.hpp"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "data_collection.hpp"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

uint32_t rdy_gpio_ctr = 0;

static absolute_time_t data_rdy_timestamps[6];

static void rdy_gpio_isr(uint gpio, uint32_t event_mask)
{
  if (event_mask & GPIO_IRQ_EDGE_FALL) {
    data_rdy_timestamps[rdy_gpio_ctr] = get_absolute_time();
    rdy_gpio_ctr++;
  }
}

MAX11254 :: MAX11254(spi_inst_t * spi_inst_in, uint32_t cs_gpio_in, uint32_t rst_gpio_in, uint32_t rdy_gpio_in):
spi_inst(spi_inst_in),
cs_gpio(cs_gpio_in),
rst_gpio(rst_gpio_in),
rdy_gpio(rdy_gpio_in)
{
  referenceVoltage = 2.8;

  gpio_init(cs_gpio);
  gpio_set_dir(cs_gpio, GPIO_OUT);
  gpio_put(cs_gpio, true);

  gpio_init(rst_gpio);
  gpio_set_dir(rst_gpio, GPIO_OUT);
  gpio_put(rst_gpio, false);

  gpio_init(rdy_gpio);
  gpio_set_dir(rdy_gpio, GPIO_IN);
  gpio_set_irq_enabled_with_callback(rdy_gpio, GPIO_IRQ_EDGE_FALL, true, rdy_gpio_isr);

  for (uint32_t i = 0; i < 6; i++)
  {
    char name[25];
    snprintf(name, sizeof(name), "MAX11254_Channel_%lu", i);
    dataChannels.push_back(data_collection_create_new_channel(name));
  }
}

// Reset chip to its default settings by power cycling reset pin (necessary at start)
void MAX11254 :: reset() const {
  gpio_put(rst_gpio, false);
  sleep_ms(100);
  gpio_put(rst_gpio, true);
}

/** Write data to specified register
* @param reg registers address
* @param data data to be written
* @param length registers length (eg. 8bit register - length = 8)
*/
void MAX11254 :: write(uint8_t reg, uint32_t data, int length) {
  uint8_t address = MSBS | (reg << 1) | WBIT;

  gpio_put(cs_gpio, false);

  spi_write_blocking(spi_inst, &address, 1);

  for (int i = (length - 8); i >= 0; i -= 8) {
    uint8_t data_to_send = ((data >> i) & 0xFF);
    spi_write_blocking(spi_inst, &data_to_send, 1);
  }

  gpio_put(cs_gpio, true);
}

/** Read data from specified register. Returns 32bit integer.
* @param reg registers address
* @param length registers length / length of data to be read in bits
*/
uint32_t MAX11254 :: read(uint8_t reg, int length) {
  uint32_t data = 0;
  uint8_t address =  MSBS | (reg << 1) | RBIT;

  gpio_put(cs_gpio, false);
  spi_write_blocking(spi_inst, &address, 1);

  for (int i = 0; i < length; i += 8) {
    uint8_t ret = 0;
    spi_read_blocking(spi_inst, 0, &ret, 1);
    data = (data << 8) | ret;
  }

  gpio_put(cs_gpio, true);
  return data;
}

// Blocks thread until data ready pin signals ( 1 -> 0) that data is ready.
// It's required to run this command before reading new values from registers
// (once each 5 registers (or less as specified in configuration))
bool MAX11254 :: dataReady() const {
  while (gpio_get(rdy_gpio));
  return true;
}

bool MAX11254 :: isPositive(uint32_t data, int length)
{
  if(((data >> (length-1)) & 0x1) == 1)
    return 1;
  else
    return 0;
}

/** Sends specified command to MAX11254
* @param cmd command to be sent
*/
void MAX11254 :: command(uint8_t cmd) {
  gpio_put(cs_gpio, false);
  spi_write_blocking(spi_inst, &cmd, 1);
  gpio_put(cs_gpio, true);
}

/** Performs adc calibration
 * @param type provide number corresponding to the type of calibration that should be performed
 * 0 - no calibration (all calibration is disabled),
 * 1 - self-calibration,
 * 2 - full system calibration (self-calibration, offset calibration, gain calibration)
*/
void MAX11254 :: calibration(int type) {
  switch (type)
  {
    case 0:
      write(CTRL3, 0x0F, 8);
      break;
    case 1:
      write(CTRL3, 0x0C, 8);
      write(CTRL1, 0x02, 8);
      command(0xA0);
      sleep_ms(200);
      break;
    case 2:
      write(CTRL3, 0x00, 8);
      read(SOC, 24);
      read(SCGC, 24);

      write(CTRL1, 0x42, 8);
      command(0xA0);
      sleep_ms(200);

      sleep_ms(6);
      write(CTRL1, 0x82, 8);
      command(0xA0);
      sleep_ms(200);

      read(SOC, 24);
      read(SCGC, 24);
      break;
    default:
      break;
  }
}

/** Enables specified operation mode
 * @param type provide number corresponding to the type of operation that sould be used
 * 0 - single cycle,
 * 1 - single cycle continuous,
 * 2 - continuous
*/
void MAX11254 :: mode(int type) {
  uint8_t regVal = read(CTRL1, 8);
  regVal = ((regVal >> 2) << 2);

  switch (type)
  {
    case 0:
      regVal = regVal | 0x02;
      break;
    case 1:
      regVal = regVal | 0x03;
      break;
    case 2:
      regVal = regVal | 0x00;
      break;
    default:
      break;
  }

  write(CTRL1, regVal, 8);
}

/** Starts conversion with specified rate
 * @param rate 4 bit conversion rate value (based on table.1. in max11254 documentation)
*/
void MAX11254 :: conversion(uint8_t rate) {
  uint8_t comm = 0xB0 | rate;
  command(comm);
}

double MAX11254 :: getVoltage(uint32_t raw_value) {

  union s24_to_s32 {
    uint8_t b[4];
    int32_t v;
  } value;

  value.b[0] = ((uint8_t*)&raw_value)[0];
  value.b[1] = ((uint8_t*)&raw_value)[1];
  value.b[2] = ((uint8_t*)&raw_value)[2];
  value.b[3] = (((uint8_t*)&raw_value)[2] & 0x80 ? 0xFF : 0);

  return value.v*((referenceVoltage/(1LL<<23))/16);
}

void MAX11254::write_full(uint8_t reg, uint8_t reg_val_HSB, uint8_t reg_val_MSB, uint8_t reg_val_LSB) {
  uint32_t value = reg_val_HSB;
  value <<= 8;
  value |= reg_val_MSB;
  value <<= 8;
  value |= reg_val_LSB;
  write(reg, value, 24);
}


void MAX11254::init_device() {
  reset();

  gpio_put(cs_gpio, false);                  // select and deselect chip to reset spi controller
  sleep_ms(100);
  gpio_put(cs_gpio, true);


  calibration(2);


  write(SEQ, 0x08, 8);

  write(CTRL1, 0x22, 8);
  write(CTRL2, 0x2C, 8);
  write(CTRL3, 0x5C, 8);

  write(GPO_DIR, 0x03, 8);

  write_full(CHMAP0, 0x02 | (1 << 2), 0x02 | (2 << 2), 0x02 | (3 << 2));
  write_full(CHMAP1, 0x02 | (4 << 2), 0x02 | (5 << 2), 0x02 | (6 << 2));
}

void MAX11254::blocking_update() {
  command(0x0F | 0x30 | 0x80);
  for (uint32_t i = 0; i < 6; i++)
  {
    for(uint32_t j = 0; j < 100; j++)
    {
      if (!gpio_get(rdy_gpio))
      {
        break;
      }
      sleep_us(1);
    }

  }
}


void MAX11254::update()
{
  if (rdy_gpio_ctr == 6 && !needs_cmd)
  {
    for (uint32_t i = 0; i < 6; i++)
    {
      dataChannels[i]->push_new_value(getVoltage(read(DATA0 + i, 24)), data_rdy_timestamps[i]);
    }
    needs_cmd = true;
  }

  // recover after lockup
  if (absolute_time_diff_us(last_update_time, get_absolute_time()) > 2000000)
  {
    needs_cmd = true;
  }

  if (needs_cmd)
  {
    command((rate&0x0F) | 0x30 | 0x80);
    needs_cmd = false;
    rdy_gpio_ctr = 0;
    last_update_time = get_absolute_time();
  }
}

void MAX11254::set_rate(uint8_t rate_in) {
  rate = rate_in;
}
