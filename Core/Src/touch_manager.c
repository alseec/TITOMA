#include <string.h>
#include "touch_manager.h"
#include "usbd_cdc_if.h"

//I2C_HandleTypeDef hi2c3;

volatile int touch_x = 0;
volatile int touch_y = 0;
volatile int touch_z = 0;

int i2c_touch_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t *buff, uint8_t buff_len)
{
  uint8_t reg_addr_aux = reg_addr;
  if (HAL_I2C_Master_Transmit(&hi2c3, slave_addr, &reg_addr_aux, 1, 100) != HAL_OK)
  {
    printf("[I2C]TX_error\r\n");
    return -1;
  }
  else
  {
    if (HAL_I2C_Master_Receive(&hi2c3, slave_addr, buff, buff_len, 100) != HAL_OK)
    {
      printf("[I2C]RX_error\r\n");
      return -1;
    }
  }

  return 0;
}

int i2c_touch_write(uint8_t slave_addr, uint8_t *buff, uint8_t buff_len)
{
  if (HAL_I2C_Master_Transmit(&hi2c3, slave_addr, buff, buff_len, 100) != HAL_OK)
  {
    printf("[I2C]TX_error\r\n");
    return -1;
  }

  return 0;
}

void i2c_touch_init(void)
{
  /* TODO: 
   * Improve the initialization routine
   */

  uint8_t rx_data[2] = {0};
  uint8_t tx_data[2];

  i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_CHIP_ID, rx_data, 2);
  printf("CHIP ID: %X %X\n", rx_data[0], rx_data[1]);

  i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_ID_VER, rx_data, 1);
  printf("ID VER: %X\n", rx_data[0]);

  i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_SYS_CTRL2, rx_data, 1);
  printf("SYS CTRL2: %X\n", rx_data[0]);

  printf("Enable clocks for everything\n");
  tx_data[0] = TOUCH_SYS_CTRL2;
  tx_data[1] = 0x00; //Turn off TSC and ADC clocks
  i2c_touch_write(I2C_TOUCH_ADDR, tx_data, 2);
  i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_SYS_CTRL2, rx_data, 1);
  printf("SYS CTRL2: %X\n", rx_data[0]);

  printf("Configure TSC, disable TSC\n");
  i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_TSC_CTRL, rx_data, 1);
  printf("TSC CTRL: %X\n", rx_data[0]);
  tx_data[0] = TOUCH_TSC_CTRL;
  tx_data[1] = 0x11; //Turn on TSC, 4 tracking index, TSC disable
  i2c_touch_write(I2C_TOUCH_ADDR, tx_data, 2);
  i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_TSC_CTRL, rx_data, 1);
  printf("TSC CTRL: %X\n", rx_data[0]);

  printf("Configure CFG, 4 samples, delay 500ms, settling 500us\n");
  tx_data[0] = TOUCH_TSC_CFG;
  tx_data[1] = 0x9A; 
  i2c_touch_write(I2C_TOUCH_ADDR, tx_data, 2);
  i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_TSC_CFG, rx_data, 1);
  printf("TSC CFG: %X\n", rx_data[0]);

  printf("Configure FIFO TH, 0xFF\n");
  i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_FIFO_TH, rx_data, 1);
  printf("FIFO TH: %X\n", rx_data[0]);
  tx_data[0] = TOUCH_FIFO_TH;
  tx_data[1] = 0x1; //Not zero!
  i2c_touch_write(I2C_TOUCH_ADDR, tx_data, 2);
  i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_FIFO_TH, rx_data, 1);
  printf("FIFO TH: %X\n", rx_data[0]);

  i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_SYS_CTRL2, rx_data, 1);
  printf("SYS CTRL2: %X\n", rx_data[0]);
}
  
int i2c_touch_detect(void)
{
  uint8_t cmd_data[2];
  uint8_t rx_data;

  i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_TSC_CTRL, &rx_data, 1);

	uint8_t state = ((rx_data & (uint8_t) TOUCH_TSC_STAT) == (uint8_t) 0x80);

	if (state > 0)
	{
    uint8_t fifo_state = 0;
    i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_FIFO_SIZE, &fifo_state, 1);

		if (fifo_state > 0)
			return 0;
	}
	else
	{
		cmd_data[0] = TOUCH_FIFO_STA;

		/* Reset FIFO */
		cmd_data[1] = 0x01;
    i2c_touch_write(I2C_TOUCH_ADDR, cmd_data, 2);

		/* Enable the FIFO again */
		cmd_data[1] = 0;
    i2c_touch_write(I2C_TOUCH_ADDR, cmd_data, 2);

    return -1;
	}

  return -1;
}

void i2c_touch_update(void)
{
  if (i2c_touch_detect() == 0)
  {    
    uint8_t rx_data[2];
    i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_TSC_DATA_X, rx_data, 2);
    uint16_t x = rx_data[1] | (rx_data[0] << 8);

    i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_TSC_DATA_Y, rx_data, 2);
    uint16_t y = rx_data[1] | (rx_data[0] << 8);

    i2c_touch_read(I2C_TOUCH_ADDR, TOUCH_TSC_DATA_Z, rx_data, 2);
    uint16_t z = rx_data[1] | (rx_data[0] << 8);
    
  /* TODO: 
   * Improve the correction routine
   */
#if 1
    y -= 360;               /* Y value first correction */
    uint16_t yr = y / 11;   /* Y value second correction */

    /* Return y position value */
    if(yr <= 0)
      yr = 0;
    else if (yr > TOUCH_Y_BOUND)
      yr = TOUCH_Y_BOUND - 1;
    else;

    y = yr;

    /* X value first correction */
    if (x <= 3000)
      x = 3870 - x;
    else
      x = 3800 - x;

    /* X value second correction */
    uint16_t xr = x / 15;

    /* Return X position value */
    if (xr <= 0)
      xr = 0;
    else if (xr > TOUCH_X_BOUND)
        xr = TOUCH_X_BOUND - 1;
    else;

    x = xr;
 #endif

    touch_x = x;  /* Update the X position */
    touch_y = y;  /* Update the Y position */
    touch_z = z;  /* Update the Z position */

    // printf("[Touched!] X:[%d] Y:[%d] Z:[%d]\r\n", touch_x, touch_y, touch_z);
    char buff[50];
    sprintf(buff, "[Touched!] X:[%d] Y:[%d] Z:[%d]\r\n", touch_x, touch_y, touch_z);
    CDC_Transmit_HS((uint8_t *) buff, (uint16_t) strlen(buff));
  }
}

void i2c_touch_handler(void)
{
	i2c_touch_update();
}
