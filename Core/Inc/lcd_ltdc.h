#ifndef _LCD_LTDC_
#define _LCD_LTDC_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "sdram_manager.h"
#include "lcd_manager.h"
#include "touch_manager.h"

#include "stm32f429xx.h"
#include "stm32f4xx_hal.h"

#define LCD_LAYER1_PIXFORMAT            LTDC_LxPFCR_ARGB4444
#define LCD_LAYER1_PIXEL_SIZE_LTDC      (sizeof(layer1_pixel))
#define LCD_LAYER1_WIDTH_LTDC           LCD_WIDTH
#define LCD_LAYER1_HEIGHT               LCD_HEIGHT
#define LCD_LAYER1_PIXELS_LTDC          (LCD_LAYER1_WIDTH_LTDC * LCD_LAYER1_HEIGHT)
#define LCD_LAYER1_BYTES_LTDC           (LCD_LAYER1_PIXELS_LTDC * LCD_LAYER1_PIXEL_SIZE_LTDC)

#define LCD_LAYER2_PIXFORMAT            LTDC_LxPFCR_ARGB4444
#define LCD_LAYER2_PIXEL_SIZE           (sizeof(layer2_pixel))

#define LCD_LAYER2_WIDTH_LTDC            32
#define LCD_LAYER2_HEIGHT_LTDC           32

#define LCD_LAYER2_PIXELS               (LCD_LAYER2_WIDTH_LTDC * LCD_LAYER2_HEIGH)
#define LCD_LAYER2_BYTES                (LCD_LAYER2_PIXELS * LCD_LAYER2_PIXEL_SIZE)

#define swap_ltdc(a, b)                 { int16_t t = a; a = b; b = t; }
#define d2r(d)                          ((d) * 6.2831853 / 360.0)

/* Layer 1 (bottom layer) is ARGB8888 format, full screen. */
typedef uint16_t layer1_pixel;

/* Layer 2 (top layer) is ARGB4444, a 128x128 square. */
typedef uint16_t layer2_pixel;

typedef enum
{
    LCD_LAYER_1,
    LCD_LAYER_2
} layer_t;

#define MAX_INLINE_ARGS (sizeof(uint8_t *))

typedef struct {
	uint16_t delay;		/* If you need a delay after */
	uint8_t cmd;		/* command to send */
	uint8_t n_args;		/* How many arguments it has */
	union {
		uint8_t args[MAX_INLINE_ARGS]; /* The first four arguments */
		uint8_t *aptr; /* More than four arguemnts */
	};
} tft_ltdc_cmd_t;

extern LTDC_HandleTypeDef hltdc;

void lcd_ltdc_drawPixel(layer_t layer, int x, int y, uint16_t color);
void lcd_ltdc_drawLine(layer_t layer, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void lcd_ltdc_drawFastVLine(layer_t layer, int16_t x, int16_t y, int16_t h, uint16_t color);

void lcd_ltdc_drawRect(layer_t layer, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void lcd_ltdc_fillRect(layer_t layer, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void lcd_ltdc_fillCircleHelper(layer_t layer, int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
void lcd_ltdc_fillCircle(layer_t layer, int16_t x0, int16_t y0, int16_t r, uint16_t color);

void lcd_ltdc_mutate_background_color(void);
void lcd_ltdc_move_sprite_ly1(void);
void lcd_ltdc_move_sprite_ly2(void);
void lcd_ltdc_draw_layer_1(void);
void lcd_ltdc_draw_layer_2(void);

void lcd_fill_buffers(void);
void lcd_ltdc_init(void);

#endif /* _LCD_LTDC_ */