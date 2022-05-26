#include "lcd_gfx.h"

struct gfx_handle_t gfx_handle;

void gfx_draw_pixel(int x, int y, uint16_t color)
{
	if ((x < 0) || (x >= gfx_handle.width) || (y < 0) || (y >= gfx_handle.height))
		return;

	gfx_handle.drawpixel(x, y, color);
}

/* Bresenham's algorithm - thx wikpedia */
void gfx_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep)
    {
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1)
    {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1)
		ystep = 1;
	else
		ystep = -1;

	for (; x0 <= x1; x0++)
    {
		if (steep)
			gfx_draw_pixel(y0, x0, color);
		else
			gfx_draw_pixel(x0, y0, color);

		err -= dy;
		if (err < 0)
        {
			y0 += ystep;
			err += dx;
		}
	}
}

void gfx_draw_fast_vline(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	/* Update in subclasses if desired! */
	gfx_draw_line(x, y, x, y + h - 1, color);
}

void gfx_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	/* Update in subclasses if desired! */
	int16_t i;
	for (i = x; i < x + w; i++) {
		gfx_draw_fast_vline(i, y, h, color);
	}
}

void gfx_fill_screen(uint16_t color)
{
	gfx_fill_rect(0, 0, gfx_handle.width, gfx_handle.height, color);
}

/* Draw a circle outline */
void gfx_draw_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	gfx_draw_pixel(x0  , y0+r, color);
	gfx_draw_pixel(x0  , y0-r, color);
	gfx_draw_pixel(x0+r, y0  , color);
	gfx_draw_pixel(x0-r, y0  , color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		gfx_draw_pixel(x0 + x, y0 + y, color);
		gfx_draw_pixel(x0 - x, y0 + y, color);
		gfx_draw_pixel(x0 + x, y0 - y, color);
		gfx_draw_pixel(x0 - x, y0 - y, color);
		gfx_draw_pixel(x0 + y, y0 + x, color);
		gfx_draw_pixel(x0 - y, y0 + x, color);
		gfx_draw_pixel(x0 + y, y0 - x, color);
		gfx_draw_pixel(x0 - y, y0 - x, color);
	}
}

/* Used to do circles and roundrects */
void gfx_fill_circle_helper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color)
{
	int16_t f     = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x     = 0;
	int16_t y     = r;

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f     += ddF_y;
		}
		x++;
		ddF_x += 2;
		f     += ddF_x;

		if (cornername & 0x1) {
			gfx_draw_fast_vline(x0+x, y0-y, 2*y+1+delta, color);
			gfx_draw_fast_vline(x0+y, y0-x, 2*x+1+delta, color);
		}
		if (cornername & 0x2) {
			gfx_draw_fast_vline(x0-x, y0-y, 2*y+1+delta, color);
			gfx_draw_fast_vline(x0-y, y0-x, 2*x+1+delta, color);
		}
	}
}

void gfx_fill_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	gfx_draw_fast_vline(x0, y0 - r, 2 * r + 1, color);
	gfx_fill_circle_helper(x0, y0, r, 3, 0, color);
}

void gfx_init(pixel_func_ptr pixel_func, int width, int height)
{
	gfx_handle.drawpixel = pixel_func;
	gfx_handle.width    = width;
	gfx_handle.height   = height;
}
