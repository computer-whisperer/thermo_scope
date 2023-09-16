#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "xpt2046_old.h"
#include "lvgl.h"
#include "hardware/spi.h"

#define XPT2046_CS 17

#define XPT2046_BITS 12
#define XPT2046_X_MIN  100
#define XPT2046_X_MAX 1900
#define XPT2046_Y_MIN  200
#define XPT2046_Y_MAX 1950

#define TP_PORTRAIT 0
#define TP_LANDSCAPE 1
#define TP_INV_PORTRAIT 2
#define TP_INV_LANDSCAPE 3
#define CHAN_X      0b01010000
#define CHAN_Y      0b00010000
#define CHAN_Z1     0b00110000
#define CHAN_Z2     0b01000000
#define CHAN_T0     0b00000000
#define CHAN_T1     0b01110000
#define CHAN_BAT    0b00100000
#define CHAN_AUX    0b01100000
#define CONV_8_BIT  0b00001000
#define CONV_12_BIT 0b00000000
#define START_BIT   0b10000000

uint8_t spiSendBuf[3];
uint8_t spiRecvBuf[3];


uint8_t conv = CONV_12_BIT;
struct  {
  uint xMin;
  uint xMax;
  uint yMin;
  uint yMax;
} range;

struct {
  uint width;
  uint height;
} dim;
uint8_t rotation;
struct {
  float x;
  float y;
} scale;
struct {
  uint x;
  uint y;
} origin;

static spi_inst_t * priv_spi_inst;

void xpt2046_Init(uint rot, spi_inst_t * spi_inst)
{
  priv_spi_inst = spi_inst;

  gpio_init(XPT2046_CS);
  gpio_set_function(XPT2046_CS, GPIO_FUNC_SIO);
  gpio_set_dir(XPT2046_CS, GPIO_OUT);
  gpio_put(XPT2046_CS, 1);

  range.xMin = 100;
  range.xMax = 1900;
  range.yMin = 200;
  range.yMax = 1950;
  dim.width = 240;
  dim.height = 320;
  rotation = rot;
  scale.x = (float)dim.width / (range.xMax - range.xMin);
  scale.y = (float)dim.height / (range.yMax - range.yMin);
  origin.x = range.xMin;
  origin.y = range.yMin;
}

#define READ_X 0x90
#define READ_Y 0xD0

#define ILI9341_TOUCH_SCALE_X 320
#define ILI9341_TOUCH_SCALE_Y 240

#define ILI9341_TOUCH_MIN_RAW_X 1500
#define ILI9341_TOUCH_MAX_RAW_X 31000
#define ILI9341_TOUCH_MIN_RAW_Y 3276
#define ILI9341_TOUCH_MAX_RAW_Y 30110


#define MAX_SAMPLES 3

static void inline send_bytes(uint8_t * data, uint8_t len) {
  spi_write_blocking(priv_spi_inst, data, len);
}

bool ILI9341_T_TouchGetCoordinates(uint16_t* x, uint16_t* y) {

  static const uint8_t cmd_read_x[] = { READ_X };
  static const uint8_t cmd_read_y[] = { READ_Y };
  uint8_t zeroes_tx[] = { 0x00, 0x00 };

  gpio_put(XPT2046_CS, 0);

  uint32_t avg_x = 0;
  uint32_t avg_y = 0;
  uint8_t nsamples = 0;
  for(uint8_t i = 0; i < MAX_SAMPLES; i++) {

    nsamples++;

    send_bytes( (uint8_t*)cmd_read_y, 1);
    uint8_t y_raw[2] = {0, 0};
    spi_read_blocking(priv_spi_inst, 0, y_raw, 2);

    send_bytes( (uint8_t*)cmd_read_x, 1);
    uint8_t x_raw[2] = {0, 0};
    spi_read_blocking(priv_spi_inst, 0, x_raw, 2);

    avg_x += (((uint16_t)x_raw[0]) << 8) | ((uint16_t)x_raw[1]);
    avg_y += (((uint16_t)y_raw[0]) << 8) | ((uint16_t)y_raw[1]);
  }

  gpio_put(XPT2046_CS, 1);

  if(nsamples < MAX_SAMPLES)
    return false;

  uint32_t raw_x = (avg_x / MAX_SAMPLES);
  if(raw_x < ILI9341_TOUCH_MIN_RAW_X) raw_x = ILI9341_TOUCH_MIN_RAW_X;
  if(raw_x > ILI9341_TOUCH_MAX_RAW_X) raw_x = ILI9341_TOUCH_MAX_RAW_X;

  uint32_t raw_y = (avg_y / MAX_SAMPLES);
  if(raw_y < ILI9341_TOUCH_MIN_RAW_X) raw_y = ILI9341_TOUCH_MIN_RAW_Y;
  if(raw_y > ILI9341_TOUCH_MAX_RAW_Y) raw_y = ILI9341_TOUCH_MAX_RAW_Y;

  *x = (raw_x - ILI9341_TOUCH_MIN_RAW_X) * ILI9341_TOUCH_SCALE_X / (ILI9341_TOUCH_MAX_RAW_X - ILI9341_TOUCH_MIN_RAW_X);
  *y = (raw_y - ILI9341_TOUCH_MIN_RAW_Y) * ILI9341_TOUCH_SCALE_Y / (ILI9341_TOUCH_MAX_RAW_Y - ILI9341_TOUCH_MIN_RAW_Y);

  if (*y > ILI9341_TOUCH_SCALE_Y)
    return false;
  if (*x > ILI9341_TOUCH_SCALE_X)
    return false;
  return true;
}

int xpt2046_raw_pos(uint16_t * x, uint16_t * y)
{
  uint16_t xVal;
  uint16_t yVal;
  spiSendBuf[0] = (START_BIT | CONV_12_BIT | CHAN_X);
  spiSendBuf[1] = 0;
  spiSendBuf[2] = 0;
  gpio_put(XPT2046_CS, 0);
  spi_write_read_blocking(priv_spi_inst, spiSendBuf, spiRecvBuf, 3);
  gpio_put(XPT2046_CS, 1);
  if(conv == CONV_8_BIT)
  {
    xVal = (uint)spiRecvBuf[1];
  }
  else
  {
    xVal = (uint)((spiRecvBuf[1] << 4) | (spiRecvBuf[2] >> 4));
  }

  spiSendBuf[0] = (START_BIT | conv | CHAN_Y);

  gpio_put(XPT2046_CS, 0);
  spi_write_read_blocking(priv_spi_inst, spiSendBuf, spiRecvBuf, 3);

  gpio_put(XPT2046_CS, 1);
  if(conv == CONV_8_BIT)
  {
    yVal = (uint)spiRecvBuf[1];
  }
  else
  {
    yVal = (uint)((spiRecvBuf[1] << 4) | (spiRecvBuf[2] >> 4));
  }

  if((xVal < range.xMin) | (xVal > range.xMax))
  {
    return(-1);
  }

  if((yVal < range.yMin) | (yVal > range.yMax))
  {
    return(-1);
  }
  *x = xVal;
  *y = yVal;
  return(0);
}

int xpt2046_Pos(uint16_t * posX, uint16_t * posY)
{
  int N = 10;
  int attempts = 1;
  uint16_t xx = 0;
  uint16_t yy = 0;
  uint done = 0;
  float mx = 0;
  float my = 0;
  uint16_t pixX = 0;
  uint16_t pixY = 0;
  uint16_t px = 0;
  uint16_t py = 0;;
  for(uint x = 0; x < attempts; x++)
  {
    int ret;
    ret = xpt2046_raw_pos(&px, &py);
    if(ret == -1)
    {
      continue;
    }
    xx += px;
    yy += py;
    done += 1;
    if(done == N)
    {
      break;
    }
  }
  if(done < N)
  {
    return(-1);
  }
  mx = (float)xx * 1.0 / N;
  my = (float)yy * 1.0 / N;
  pixX = (uint16_t)(scale.x * (mx - origin.x));
  pixY = (uint16_t)(scale.y * (my - origin.y));
  if(rotation == TP_PORTRAIT)
  {
    *posX = pixX;
    *posY = dim.height - pixY;
  }
  else if(rotation == TP_LANDSCAPE)
  {
    *posX = dim.height - pixY;
    *posY = dim.width - pixX;
  }
  else if(rotation == TP_INV_PORTRAIT)
  {
    *posX = dim.width - pixX;
    *posY = pixY;
  }
  else if(rotation == TP_INV_LANDSCAPE)
  {
    *posX = pixY;
    *posY = pixX;
  }
  else
  {
    *posX = 0;
    *posY = 0;
    return(-1);
  }
  return(0);
}

static bool is_pressed = false;
static uint16_t posX;
static uint16_t posY;

void xpt2046_poll()
{
  is_pressed = ILI9341_T_TouchGetCoordinates(&posX, &posY);
}

void xpt2046_read_cb(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
  uint32_t margin = 15;
  if ((posX < margin)||(posX > ILI9341_TOUCH_SCALE_X - margin))
  {
    is_pressed = false;
  }
  if ((posY < margin)||(posY > ILI9341_TOUCH_SCALE_Y - margin))
  {
    is_pressed = false;
  }

  if(!is_pressed)
  {
    data->point.x = 0;
    data->point.y = 0;
    data->state = 0;
  }
  else
  {
    data->point.x = 320-posX;
    data->point.y = posY;
    data->state = 1;
  }
}