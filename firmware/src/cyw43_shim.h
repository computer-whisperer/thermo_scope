//
// Created by christian on 9/17/23.
//

#ifndef THERMO_SCOPE_CYW43_SHIM_H
#define THERMO_SCOPE_CYW43_SHIM_H

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC int cyw43_shim_wifi_link_status(int itf);

#endif //THERMO_SCOPE_CYW43_SHIM_H
