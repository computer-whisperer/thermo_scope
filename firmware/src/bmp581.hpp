//
// Created by christian on 9/16/23.
//

#ifndef THERMO_SCOPE_BMP581_HPP
#define THERMO_SCOPE_BMP581_HPP
#include "i2c_bus_manager.hpp"
#include "data_collection.hpp"

/*!
 * This is a helper class for managing bmp581 register manipulation
 */
class BMP581RegisterField {
public:
  uint8_t reg_addr;
  uint8_t mask;
  uint8_t shift;

  constexpr BMP581RegisterField(uint8_t reg_addr_in, uint8_t mask_in) : reg_addr(reg_addr_in), mask(mask_in), shift(0) {
    uint8_t shift_val = 0;
    for (; shift_val < 8; shift_val++) {
      if ((1 << shift_val) & mask) {
        break;
      }
    }
    shift = shift_val;
  }

  /*!
   * @param register_value the full register value
   * @return The masked and shifted value of a specific field in the register
   */
  constexpr uint8_t read_from_register_value(uint8_t register_value) const {
    return (register_value & mask) >> shift;
  }

  /*!
   * @param register_value the full previous register value
   * @return The new register value incorporating the new value for the specific field
   */
  constexpr uint8_t write_in_register_value(uint8_t register_value, uint8_t field_value) const {
    return ((field_value << shift) & mask) | (register_value & ~mask);
  }
};

class BMP581: public I2CPeripheralDriver {
  DataChannel* pressure_data_channel;
  DataChannel* temperature_data_channel;
  bool press_in_fifo = false;
  bool temp_in_fifo = false;

  absolute_time_t last_fetch_timestamp = nil_time;
public:

  static constexpr uint8_t REG_CMD = 0x7E;
  static constexpr uint8_t REG_OSR_EFF = 0x38;
  static constexpr uint8_t REG_ODR_CONFIG = 0x37;
  static constexpr uint8_t REG_OSR_CONFIG = 0x36;
  static constexpr uint8_t REG_OOR_CONFIG = 0x35;
  static constexpr uint8_t REG_OOR_RANGE = 0x34;
  static constexpr uint8_t REG_OOR_THR_P_MSB = 0x33;
  static constexpr uint8_t REG_OOR_THR_P_LSB = 0x32;
  static constexpr uint8_t REG_DSP_IIR = 0x31;
  static constexpr uint8_t REG_DSP_CONFIG = 0x30;
  static constexpr uint8_t REG_NVM_DATA_MSB = 0x2D;
  static constexpr uint8_t REG_NVM_DATA_LSB = 0x2C;
  static constexpr uint8_t REG_NVM_ADDR = 0x2B;
  static constexpr uint8_t REG_FIFO_DATA = 0x29;
  static constexpr uint8_t REG_STATUS = 0x28;
  static constexpr uint8_t REG_INT_STATUS = 0x27;
  static constexpr uint8_t REG_PRESS_DATA_MSB = 0x22;
  static constexpr uint8_t REG_PRESS_DATA_LSB = 0x21;
  static constexpr uint8_t REG_PRESS_DATA_XLSB = 0x20;
  static constexpr uint8_t REG_TEMP_DATA_MSB = 0x1F;
  static constexpr uint8_t REG_TEMP_DATA_LSB = 0x1E;
  static constexpr uint8_t REG_TEMP_DATA_XLSB = 0x1D;
  static constexpr uint8_t REG_FIFO_SEL = 0x18;
  static constexpr uint8_t REG_FIFO_COUNT = 0x17;
  static constexpr uint8_t REG_FIFO_CONFIG = 0x16;
  static constexpr uint8_t REG_INT_SOURCE = 0x15;
  static constexpr uint8_t REG_INT_CONFIG = 0x14;
  static constexpr uint8_t REG_DRIVE_CONFIG = 0x13;
  static constexpr uint8_t REG_CHIP_STATUS = 0x11;
  static constexpr uint8_t REG_REV_ID = 0x02;
  static constexpr uint8_t REG_CHIP_ID = 0x01;

  static constexpr BMP581RegisterField FIELD_CHIP_ID{REG_CHIP_ID, 0xFF};

  static constexpr BMP581RegisterField FIELD_ASIC_REV_ID{REG_REV_ID, 0xFF};

  static constexpr BMP581RegisterField FIELD_I3C_ERR_3{REG_CHIP_STATUS, 0x08};
  static constexpr BMP581RegisterField FIELD_I3C_ERR_0{REG_CHIP_STATUS, 0x04};
  static constexpr BMP581RegisterField FIELD_HIF_MODE{REG_CHIP_STATUS, 0x03};

  static constexpr BMP581RegisterField FIELD_PAD_IF_DRV{REG_DRIVE_CONFIG, 0xF0};
  static constexpr BMP581RegisterField FIELD_SPI3_EN{REG_DRIVE_CONFIG, 0x02};
  static constexpr BMP581RegisterField FIELD_I2C_CSB_PUP_EN{REG_DRIVE_CONFIG, 0x01};

  static constexpr BMP581RegisterField FIELD_PAD_INT_DRV{REG_INT_CONFIG, 0xF0};
  static constexpr BMP581RegisterField FIELD_INT_EN{REG_INT_CONFIG, 0x08};
  static constexpr BMP581RegisterField FIELD_INT_OD{REG_INT_CONFIG, 0x04};
  static constexpr BMP581RegisterField FIELD_INT_POL{REG_INT_CONFIG, 0x02};
  static constexpr BMP581RegisterField FIELD_INT_MODE{REG_INT_CONFIG, 0x01};

  static constexpr BMP581RegisterField FIELD_OOR_P_EN{REG_INT_SOURCE, 0x08};
  static constexpr BMP581RegisterField FIELD_FIFO_THS_EN{REG_INT_SOURCE, 0x04};
  static constexpr BMP581RegisterField FIELD_FIFO_FULL_EN{REG_INT_SOURCE, 0x02};
  static constexpr BMP581RegisterField FIELD_DRDY_DATA_REG_EN{REG_INT_SOURCE, 0x01};

  static constexpr BMP581RegisterField FIELD_FIFO_MODE{REG_FIFO_CONFIG, 0x20};
  static constexpr BMP581RegisterField FIELD_FIFO_THRESHOLD{REG_FIFO_CONFIG, 0x1F};

  static constexpr BMP581RegisterField FIELD_FIFO_COUNT{REG_FIFO_COUNT, 0x3F};

  static constexpr BMP581RegisterField FIELD_FIFO_DEC_SEL{REG_FIFO_SEL, 0x1C};
  static constexpr BMP581RegisterField FIELD_FIFO_FRAME_SEL{REG_FIFO_SEL, 0x03};

  static constexpr BMP581RegisterField FIELD_TEMP_XLSB{REG_TEMP_DATA_XLSB, 0xFF};
  static constexpr BMP581RegisterField FIELD_TEMP_LSB{REG_TEMP_DATA_LSB, 0xFF};
  static constexpr BMP581RegisterField FIELD_TEMP_MSB{REG_TEMP_DATA_MSB, 0xFF};

  static constexpr BMP581RegisterField FIELD_PRESS_XLSB{REG_PRESS_DATA_XLSB, 0xFF};
  static constexpr BMP581RegisterField FIELD_PRESS_LSB{REG_PRESS_DATA_LSB, 0xFF};
  static constexpr BMP581RegisterField FIELD_PRESS_MSB{REG_PRESS_DATA_MSB, 0xFF};

  static constexpr BMP581RegisterField FIELD_POR{REG_INT_STATUS, 0x10};
  static constexpr BMP581RegisterField FIELD_OOR_P{REG_INT_STATUS, 0x08};
  static constexpr BMP581RegisterField FIELD_FIFO_THS{REG_INT_STATUS, 0x04};
  static constexpr BMP581RegisterField FIELD_FIFO_FULL{REG_INT_STATUS, 0x02};
  static constexpr BMP581RegisterField FIELD_DRDY_DATA_REG{REG_INT_STATUS, 0x01};

  static constexpr BMP581RegisterField FIELD_ST_CRACK_PASS{REG_STATUS, 0x80};
  static constexpr BMP581RegisterField FIELD_STATUS_BOOT_ERR_CORRECTED{REG_STATUS, 0x10};
  static constexpr BMP581RegisterField FIELD_STATUS_NVM_CMD_ERR{REG_STATUS, 0x08};
  static constexpr BMP581RegisterField FIELD_STATUS_NVM_ERR{REG_STATUS, 0x04};
  static constexpr BMP581RegisterField FIELD_STATUS_NVM_RDY{REG_STATUS, 0x02};
  static constexpr BMP581RegisterField FIELD_STATUS_CORE_RDY{REG_STATUS, 0x01};

  static constexpr BMP581RegisterField FIELD_FIFO_DATA{REG_FIFO_DATA, 0xFF};

  static constexpr BMP581RegisterField FIELD_NVM_PROG_EN{REG_NVM_ADDR, 0xFF};
  static constexpr BMP581RegisterField FIELD_NVM_ROW_ADDR{REG_NVM_ADDR, 0xFF};

  static constexpr BMP581RegisterField FIELD_NVM_DATA_LSB{REG_NVM_DATA_LSB, 0xFF};

  static constexpr BMP581RegisterField FIELD_NVM_DATA_MSB{REG_NVM_DATA_MSB, 0xFF};

  static constexpr BMP581RegisterField FIELD_OOR_SEL_IIR_P{REG_DSP_CONFIG, 0x80};
  static constexpr BMP581RegisterField FIELD_FIFO_SEL_IIR_P{REG_DSP_CONFIG, 0x40};
  static constexpr BMP581RegisterField FIELD_SHDW_SEL_IIR_P{REG_DSP_CONFIG, 0x20};
  static constexpr BMP581RegisterField FIELD_FIFO_SEL_IIR_T{REG_DSP_CONFIG, 0x10};
  static constexpr BMP581RegisterField FIELD_SHDW_SEL_IIR_T{REG_DSP_CONFIG, 0x08};
  static constexpr BMP581RegisterField FIELD_IIR_FLUSH_FORCED_EN{REG_DSP_CONFIG, 0x04};

  static constexpr BMP581RegisterField FIELD_SET_IIR_P{REG_DSP_IIR, 0x28};
  static constexpr BMP581RegisterField FIELD_SET_IIR_T{REG_DSP_IIR, 0x07};

  static constexpr BMP581RegisterField FIELD_OOR_THR_P_LSB{REG_OOR_THR_P_LSB, 0xFF};
  static constexpr BMP581RegisterField FIELD_OOR_THR_P_MSB{REG_OOR_THR_P_MSB, 0xFF};

  static constexpr BMP581RegisterField FIELD_OOR_RANGE_P{REG_OOR_RANGE, 0xFF};

  static constexpr BMP581RegisterField FIELD_CNT_LIM{REG_OOR_CONFIG, 0xC0};
  static constexpr BMP581RegisterField FIELD_OOR_THR_P_16{REG_OOR_CONFIG, 0x01};

  static constexpr BMP581RegisterField FIELD_PRESS_EN{REG_OSR_CONFIG, 0x40};
  static constexpr BMP581RegisterField FIELD_OSR_P{REG_OSR_CONFIG, 0x38};
  static constexpr BMP581RegisterField FIELD_OSR_T{REG_OSR_CONFIG, 0x07};

  static constexpr BMP581RegisterField FIELD_DEEP_DIS{REG_ODR_CONFIG, 0x80};
  static constexpr BMP581RegisterField FIELD_ODR{REG_ODR_CONFIG, 0x7C};
  static constexpr BMP581RegisterField FIELD_PWR_MODE{REG_ODR_CONFIG, 0x03};

  static constexpr BMP581RegisterField FIELD_ODR_IS_VALID{REG_OSR_EFF, 0x80};
  static constexpr BMP581RegisterField FIELD_OSR_P_EFF{REG_OSR_EFF, 0x38};
  static constexpr BMP581RegisterField FIELD_OSR_T_EFF{REG_OSR_EFF, 0x07};

  static constexpr BMP581RegisterField FIELD_CMD{REG_CMD, 0xFF};

  uint8_t field_read(BMP581RegisterField field) {
    return field.read_from_register_value(register_read(field.reg_addr));
  }

  void field_write(BMP581RegisterField field, uint8_t value) {
    uint8_t original_value = 0x00;
    // No need to read existing value if the field takes up the full register
    if (field.mask != 0xFF)
      original_value = register_read(field.reg_addr);
    uint8_t new_value = field.write_in_register_value(original_value, value);
    register_write(field.reg_addr, new_value);
  }

  void register_write(uint8_t addr, uint8_t value);

  uint8_t register_read(uint8_t addr);

  static constexpr float process_pressure_data_kPa(uint8_t msb, uint8_t lsb, uint8_t xlsb) {
    uint32_t raw = 0;
    raw |= msb << 16;
    raw |= lsb << 8;
    raw |= xlsb;
    // Page 22 of datasheet gives transfer function
    return ((float) raw / (1 << 6)) / 1000;
  }

  static constexpr float process_temperature_data_C(uint8_t msb, uint8_t lsb, uint8_t xlsb) {
    uint32_t raw = 0;
    raw |= msb << 16;
    raw |= lsb << 8;
    raw |= xlsb;
    // Page 22 of datasheet gives transfer function
    return (float) raw / (1 << 16);
  }

  bool check_device_presence();

  static constexpr uint8_t get_i2c_address(bool addr_select){
    return addr_select? 0x47 : 0x46;
  }

  BMP581(i2c_inst_t* i2c_bus_in, bool addr_select_in);

  void initialize_device();
  void update();

  void set_fifo_mode(bool temp_in_fifo_in, bool press_in_fifo_in);
  uint32_t do_forced_measurement(float *temp_out, float *press_out);
  void start_normal_mode(int odr_state);

  void pull_from_fifo();
};

#endif //THERMO_SCOPE_BMP581_HPP
