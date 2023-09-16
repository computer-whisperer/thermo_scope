//
// Created by christian on 9/15/23.
//

#ifndef THERMO_SCOPE_XPT2046_H
#define THERMO_SCOPE_XPT2046_H

#include <src/hal/lv_hal.h>

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC void xpt2046_read_cb(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data);

#endif //THERMO_SCOPE_XPT2046_H
