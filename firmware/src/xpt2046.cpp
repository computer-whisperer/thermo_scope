//
// Created by christian on 9/15/23.
//

#include "xpt2046.hpp"
#include "xpt2046.h"
#include "hardware/gpio.h"

XPT2046::XPT2046(spi_inst_t * spi_inst_in, uint32_t cs_gpio_in): spi_inst(spi_inst_in), cs_gpio(cs_gpio_in)
{
  gpio_init(cs_gpio);
  gpio_set_function(cs_gpio, GPIO_FUNC_SIO);
  gpio_set_dir(cs_gpio, GPIO_OUT);
  gpio_put(cs_gpio, true);
}

void XPT2046::update() {

};

void xpt2046_read_cb(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{

}