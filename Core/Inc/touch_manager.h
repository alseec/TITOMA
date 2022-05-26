#ifndef _TESTING_TOUCH_
#define _TESTING_TOUCH_

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c3;

#define I2C_TOUCH_ADDR              0x82

#define TOUCH_CHIP_ID               0x00
#define TOUCH_ID_VER                0x02
#define TOUCH_SYS_CTRL1             0x03
#define TOUCH_SYS_CTRL2             0x04
#define TOUCH_TSC_CTRL              0x40
#define TOUCH_TSC_CFG               0x41
#define TOUCH_FIFO_TH               0x4A
#define TOUCH_FIFO_STA              0x4B
#define TOUCH_FIFO_SIZE             0x4C
#define TOUCH_TSC_DATA_X            0x4D
#define TOUCH_TSC_DATA_Y            0x4F
#define TOUCH_TSC_DATA_Z            0x51
#define TS_I2C_ADDRESS              0x82
#define TOUCH_TSC_STAT              0x80

#define TOUCH_X_BOUND               240
#define TOUCH_Y_BOUND               320

extern volatile int touch_x;
extern volatile int touch_y;

int i2c_touch_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t *buff, uint8_t buff_len);
void i2c_touch_init(void);
int i2c_touch_detect(void);
void i2c_touch_update(void);
void i2c_touch_handler(void);

#endif /* _TESTING_TOUCH_ */