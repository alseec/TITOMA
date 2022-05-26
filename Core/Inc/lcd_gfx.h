#ifndef _LCD_GFX_
#define _LCD_GFX_

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define GFX_WIDTH   320
#define GFX_HEIGHT  240

#define GFX_COLOR_WHITE          0xFFFF
#define GFX_COLOR_BLACK          0x0000
#define GFX_COLOR_GREY           0xF7DE
#define GFX_COLOR_BLUE           0x001F
#define GFX_COLOR_BLUE2          0x051F
#define GFX_COLOR_RED            0xF800
#define GFX_COLOR_MAGENTA        0xF81F
#define GFX_COLOR_GREEN          0x07E0
#define GFX_COLOR_CYAN           0x7FFF
#define GFX_COLOR_YELLOW         0xFFE0

#define swap(a, b)              {int16_t t = a; a = b; b = t;}
#define d2r(d) 					((d) * 6.2831853 / 360.0)

typedef void (*pixel_func_ptr)(int, int, uint16_t);

struct gfx_handle_t {
	int16_t width;
	int16_t height;
	pixel_func_ptr drawpixel;
};

void gfx_draw_pixel(int x, int y, uint16_t color);

void gfx_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void gfx_draw_fast_vline(int16_t x, int16_t y, int16_t h, uint16_t color);
void gfx_draw_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void gfx_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void gfx_fill_screen(uint16_t color);
void gfx_fill_circle_helper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
void gfx_fill_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

void gfx_init(pixel_func_ptr, int width, int height);

#endif /* _LCD_GFX_ */