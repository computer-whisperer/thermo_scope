//
// Created by christian on 6/7/23.
//

#ifndef PICO_TEST_MANAGER_MAX11254_H
#include <cstdint>
#include <vector>
#include "hardware/spi.h"
#include "pico/time.h"


// read / write bits
#define MSBS        0xC0
#define RBIT        1
#define WBIT        0

// status and settings registers
#define STAT        0x00            // R    24 bit
#define CTRL1       0x01            // R/W  8bit
#define CTRL2       0x02            // R/W  8bit
#define CTRL3       0x03            // R/W  8bit
#define GPIO_CTRL   0x04            // R/W  8bit
#define DELAY       0x05            // R/W  16bit
#define CHMAP1      0x06            // R/W  24bit
#define CHMAP0      0x07            // R/W  24bit
#define SEQ         0x08            // R/W  8bit
#define GPO_DIR     0x09            // R/W  8bit
#define SOC         0xA             // R/W  24bit
#define SCGC        0xD             // R/W  24bit

// data registers
#define DATA0       0xE             // R    24bit
#define DATA1       0xF             // R    24bit
#define DATA2       0x10            // R    24bit
#define DATA3       0x11            // R    24bit
#define DATA4       0x12            // R    24bit
#define DATA5       0x13            // R    24bit

class MAX11254
{
private:
  double referenceVoltage;
  void reset() const;
  spi_inst_t * spi_inst;
  std::vector<struct DataChannel*> dataChannels;
  uint32_t cs_gpio;
  uint32_t rst_gpio;
  uint32_t rdy_gpio;

  bool needs_cmd = true;
  int32_t update_phase = 0;
  absolute_time_t last_update_time = nil_time;
public:
  MAX11254(spi_inst_t * spi_inst_in, uint32_t cs_gpio_in, uint32_t rst_gpio_in, uint32_t rdy_gpio_in);

  uint8_t rate = 8;

  void set_rate(uint8_t rate);

  void init_device();

  uint32_t read(uint8_t reg, int length);
  void write(uint8_t reg, uint32_t data, int length);
  void write_full(uint8_t reg, uint8_t reg_val_HSB, uint8_t reg_val_MSB, uint8_t reg_val_LSB);
  void command(uint8_t cmd);
  bool dataReady() const;
  void calibration(int type);
  void mode(int type);
  void conversion(uint8_t rate);
  double getVoltage(uint32_t data);
  bool isPositive(uint32_t data, int length);

  void blocking_update();

  void update();
};

#endif //PICO_TEST_MANAGER_MAX11254_H
