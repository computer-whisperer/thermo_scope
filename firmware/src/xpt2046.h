#ifndef XPT2046_H
#define XPT2046_H
#include "lvgl.h"
#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif


#define TP_PORTRAIT 0
#define TP_LANDSCAPE 1
#define TP_INV_PORTRAIT 2
#define TP_INV_LANDSCAPE 3

void xpt2046_Init(uint rot, spi_inst_t * spi_inst);
int xpt2046_raw_pos(uint16_t * x, uint16_t * y);
int xpt2046_Pos(uint16_t * posX, uint16_t * posY);
void xpt2046_read_cb(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
void xpt2046_poll();

#ifdef __cplusplus
}
#endif

#endif