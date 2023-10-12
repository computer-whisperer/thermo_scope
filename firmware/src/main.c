
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "data_collection.h"
#include "hardware/adc.h"
#include "power.h"
#include <pico/cyw43_arch.h>

int main()
{
  power_main_loop();
  return 0;
}
