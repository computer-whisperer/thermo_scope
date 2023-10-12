//
// Created by christian on 9/17/23.
//

#ifndef THERMO_SCOPE_POWER_H
#define THERMO_SCOPE_POWER_H

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

extern bool power_wifi_power_state;

EXTERNC void power_main_loop();

#endif //THERMO_SCOPE_POWER_H
