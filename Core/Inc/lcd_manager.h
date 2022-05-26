#ifndef _LCD_MANAGER_
#define _LCD_MANAGER_

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "lcd_gfx.h"
#include "lcd_image.h"

/* Regulative[sic] Command Set */
#define ILI_NOP                     0x00
#define ILI_RESET                   0x01
#define ILI_RD_DID                  0x04
#define ILI_RD_STS                  0x09
#define ILI_RD_PWR_MODE             0x0a
#define ILI_RD_MADCTL               0x0b
#define ILI_RD_PXL_FMT              0x0c
#define ILI_PD_IMG_FMT              0x0d
#define ILI_RD_SIG_MODE             0x0e
#define ILI_RD_DIAG_RSLT            0x0f
#define ILI_ENTER_SLEEP             0x10
#define ILI_SLEEP_OUT               0x11
#define ILI_PARTIAL_ON              0x12
#define ILI_NORMAL_MODE_ON          0x13
#define ILI_INVERSE_ON              0x20
#define ILI_INVERSE_OFF             0x21
#define ILI_GAMMA_SET               0x26
#define ILI_DISP_OFF                0x28
#define ILI_DISP_ON                 0x29
#define ILI_CAS                     0x2a
#define ILI_PAS                     0x2b
#define ILI_MEM_WRITE               0x2c
#define ILI_COLOR_SET               0x2d
#define ILI_MEM_READ                0x2e
#define ILI_PARTIAL_AREA            0x30
#define ILI_VERT_SCROLL_DEF         0x33
#define ILI_TEAR_EFF_OFF            0x34
#define ILI_TEAR_EFF_ON             0x35
#define ILI_MEM_ACC_CTL             0x36
#define ILI_V_SCROLL_START          0x37
#define ILI_IDLE_OFF                0x38
#define ILI_IDLE_ON                 0x39
#define ILI_PIX_FMT_SET             0x3a
#define ILI_WR_MEM_CONT             0x3c
#define ILI_RD_MEM_CONT             0x3e
#define ILI_SET_TEAR_LINE           0x44
#define ILI_GET_SCANLINE            0x45
#define ILI_WR_BRIGHTNESS           0x51
#define ILI_RD_BRIGHTNESS           0x52
#define ILI_WR_CTRL                 0x53
#define ILI_RD_CTRL                 0x54
#define ILI_WR_CABC                 0x55
#define ILI_RD_CABC                 0x56
#define ILI_WR_CABC_MIN             0x5e
#define ILI_RD_CABC_MAX             0x5f
#define ILI_RD_ID1                  0xda
#define ILI_RD_ID2                  0xdb
#define ILI_RD_ID3                  0xdc

/* Extended Command Set */
#define ILI_RGB_IFC_CTL             0xb0
#define ILI_FRM_CTL_NORM            0xb1
#define ILI_FRM_CTL_IDLE            0xb2
#define ILI_FRM_CTL_PART            0xb3
#define ILI_INVERSE_CTL             0xb4
#define ILI_PORCH_CTL               0xb5
#define ILI_FUNC_CTL                0xb6
#define ILI_ENTRY_MODE_SET          0xb7
#define ILI_BL_CTL_1                0xb8
#define ILI_BL_CTL_2                0xb9
#define ILI_BL_CTL_3                0xba
#define ILI_BL_CTL_4                0xbb
#define ILI_BL_CTL_5                0xbc
#define ILI_BL_CTL_7                0xbe
#define ILI_BL_CTL_8                0xbf
#define ILI_PWR_CTL_1               0xc0
#define ILI_PWR_CTL_2               0xc1
#define ILI_VCOM_CTL_1              0xc5
#define ILI_VCOM_CTL_2              0xc7
#define ILI_NV_MEM_WR               0xd0
#define ILI_NV_MEM_PROT_KEY         0xd1
#define ILI_NV_MEM_STATUS_RD        0xd2
#define ILI_RD_ID4                  0xd3
#define ILI_POS_GAMMA               0xe0
#define ILI_NEG_GAMMA               0xe1
#define ILI_GAMMA_CTL_1             0xe2
#define ILI_GAMMA_CTL_2             0xe3
#define ILI_IFC_CTL                 0xf6

#define LCD_WIDTH                   240
#define LCD_HEIGHT                  320
#define LCD_LAYER1_PIXEL_SIZE       (sizeof(uint16_t))
#define LCD_LAYER1_WIDTH            LCD_WIDTH
#define LCD_LAYER1_HEIGHT           LCD_HEIGHT
#define LCD_LAYER2_WIDTH            LCD_WIDTH
#define LCD_LAYER2_HEIGHT           LCD_HEIGHT
#define LCD_LAYER1_PIXELS           (LCD_LAYER1_WIDTH * LCD_LAYER1_HEIGHT)
#define LCD_LAYER1_BYTES            (LCD_LAYER1_PIXELS * LCD_LAYER1_PIXEL_SIZE)

#define FRAME_SIZE                  (LCD_WIDTH * LCD_HEIGHT)
#define FRAME_SIZE_BYTES            (FRAME_SIZE * 2)

/* Color definitions */
#define	LCD_BLACK   				0x0000
#define	LCD_BLUE    				0x1F00
#define	LCD_RED     				0x00F8
#define	LCD_GREEN   				0xE007
#define LCD_CYAN    				0xFF07
#define LCD_MAGENTA 				0x1FF8
#define LCD_YELLOW  				0xE0FF
#define LCD_WHITE   				0xFFFF
#define LCD_GREY    				0xC339

extern uint16_t *cur_frame;
extern uint16_t *display_frame;

extern SPI_HandleTypeDef hspi5;

/*
 * ILI9341 datasheet, pp 46-49:
 *
 *     RCM[1:0} = 0b10    command 0xb0
 *     DPI[2:0] = 0b110   command 0x3a
 *     RIM      = 0       command 0xf6
 *     PCDIV    = ????    command 0xB6
 *
 * Pp 239-240:
 *     external fosc = DOTCLK / (2 * (PCDIV + 1))
 *
 * ("Cube" is how the STM32F4Cube demo software sets the register.
 *  "Chuck" is ChuckM's lcd-serial demo, first revision.)
 *
 * Command 0x3A: COLMOD: Pixel Format Set  LCD_PIXEL_FORMAT
 *                Reset              Cube   Chuck
 *      DPI[2:0]   110 (18 bit/pix)   110    101 (16 bit/pix)
 *      DBI[2:0]   110 (18 bit/pix)   110    101 (16 bit/pix)
 *
 * Command 0xB0: RGB Interface Signal      LCD_RGB_INTERFACE
 *                Reset              Cube
 *      Bypass:      0 (direct)        1 (memory)
 *      RCM[1:0]    10                10
 *      VSPL         0 (low)           0
 *      HSPL         0 (low)           0
 *      DPL          0 (rising)        1 (falling)
 *      EPL          1 (low)           0 (high)
 *
 * Command 0xB6: Display Function Control  LCD_DFC
 *                Reset              Cube 0A A7 27 04
 *      PTG[1:0]    10                10
 *      PT[1:0]     10                10
 *      REV          1                 1
 *      GS           0                 0
 *      SS           0 (S1->S720)      1 (S720->S1)
 *      SM           0                 0
 *      ISC[3:0]  0010 (5 frames)   0111 (15 frames)
 *      NL[5:0]   100111          100111
 *      PCDIV[5:0]   ?            000100
 *   S720->S1 moves the origin from the lower left corner to lower right
 *   (viewing the board so the silkscreen is upright)
 *
 * Command 0xF6: Interface Control         LCD_INTERFACE
 *               Reset              Cube  01 00 06
 *      MY_EOR       0                 0
 *      MX_EOR       0                 0
 *      MV_EOR       0                 0
 *      BGR_EOR      0                 0
 *      WEMODE       1 (wrap)          1
 *      EPF[1:0]    00                00
 *      MDT[1:0]    00                00
 *      ENDIAN       0 (MSB first)     0
 *      DM[1:0]     00 (int clk)      01 (RGB ifc)
 *      RM           0 (sys ifc)       1 (RGB ifc)
 *      RIM          0 (1 xfr/pix)     0
 */

/*
* This structure defines the sequence of commands to send
* to the Display in order to initialize it. The AdaFruit
* folks do something similar, it helps when debugging the
* initialization sequence for the display.
*/
typedef struct {
	uint16_t delay;		/* If you need a delay after */
	uint8_t cmd;		/* Command to send */
	uint8_t args_len;		/* How many arguments it has */
	uint8_t *cmd_data;	/* Command data*/
} lcd_cmds_t;

void lcd_command(uint8_t cmd, int delay, int args_len, uint8_t *args);
void lcd_draw_pixel(int x, int y, uint16_t color);
void lcd_test_image(void);
void lcd_show_frame(void);
void lcd_start(const lcd_cmds_t cmds[]);
void lcd_spi_init(void);
void lcd_handler_run(void);

#endif /* _LCD_MANAGER_ */