//
// Created by christian on 9/17/23.
//

#include "cyw43_shim.h"
#include "cyw43.h"

int cyw43_shim_wifi_link_status(int itf)
{
  return cyw43_wifi_link_status(&cyw43_state, itf);
}