/*
 * The datasheet says (Figure 16, page 151):
 *     The LCD-TFT clock comes from PLLSAI.
 *     PLLSRC selects either HSI or HSE.
 *     PLLSAI's input clock is either HSI or HSE divided by PLLM.
 *     PLLSAI's PLLLCDCLK output is the input * PLLSAIN / PLLSAIR.
 *     LCD-TFT clock is PLLLCDCLK divided by PLLSAIDIVR.
 *
 * PLLSRC and PLLM are in the RCC_PLLCFGR register.
 * PLLSAIN and PLLSAIR are in RCC_PLLSAICFGR.
 * PLLSAIDIVR is in RCC_DCKCFGR;
 *
 * In our case,
 * PLLSRC already selected HSE, which is 8 MHz.
 * PLLM is already set to 8.  8 MHz / 4 = 2 MHz.
 * We set PLLSAIN = 60 and PLLSAIR = 2.  2 MHz * 60 / 5 = 24 MHz.
 * We set PLLSAIDIVR to 4.  24 MHz / 4 = 6 MHz.
 * So the LCD-TFT pixel clock is 6 MHz.
 *
 * The number of clocks per frame is
 * (VSYNC + VBP + LCD_HEIGHT + VFP) * (HSYNC + HBP + LCD_WIDTH + HFP) =
 * (2 + 2 + 320 + 4) * (10 + 20 + 240 + 10) = 91840.
 * (2 + 4 + 320 + 4) * (100 + 24 + 240 + 16) = 125400.
 *
 * So the refresh frequency is 6 MHz / 91840 ~= 65.6 Hz.
 * So the refresh frequency is 6 MHz / 125400 ~= 47.8 Hz.
 */

#include "lcd_ltdc.h"

//LTDC_HandleTypeDef hltdc;

/* Layer 1 (bottom layer) is ARGB4444 format */
layer1_pixel * const lcd_layer1_frame_buffer = SDRAM_BASE_ADDRESS;

/* Layer 2 (top layer) is ARGB4444 format */
layer2_pixel * const lcd_layer2_frame_buffer = SDRAM_BASE_ADDRESS + LCD_LAYER1_BYTES_LTDC;

static const uint8_t pos_gamma_args_ltdc[] = { 0x0F, 0x29, 0x24, 0x0C, 0x0E,
												0x09, 0x4E, 0x78, 0x3C, 0x09,
												0x13, 0x05, 0x17, 0x11, 0x00 };
	
static const uint8_t neg_gamma_args_ltdc[] = { 0x00, 0x16, 0x1B, 0x04, 0x11,
												0x07, 0x31, 0x33, 0x42, 0x05,
												0x0C, 0x0A, 0x28, 0x2F, 0x0F };

const tft_ltdc_cmd_t ltdc_init_cmd[] = {
	{  0, ILI_MEM_ACC_CTL,      1, .args = { 0x08 } },
	{  0, ILI_RGB_IFC_CTL,      1, .args = { 0xc0 } },
	{  0, ILI_IFC_CTL,          3, .args = { 0x01, 0x00, 0x06 } },
	{  0, ILI_GAMMA_SET,        1, .args = { 0x01 } },
	{  0, ILI_POS_GAMMA,       15, .aptr = pos_gamma_args_ltdc },
	{  0, ILI_NEG_GAMMA,       15, .aptr = neg_gamma_args_ltdc },
	{ +5, ILI_SLEEP_OUT,        0, .args = {} },
	{  0, ILI_DISP_ON,          0, .args = {} },
};

void lcd_ltdc_drawPixel(layer_t layer, int x, int y, uint16_t color)
{
	if (layer == LCD_LAYER_1)
	{
		*(lcd_layer1_frame_buffer + x + y * LCD_LAYER1_WIDTH_LTDC) = color;
	}
	else
	{
		*(lcd_layer2_frame_buffer + x + y * LCD_LAYER2_WIDTH_LTDC) = color;
	}
}

void lcd_ltdc_drawLine(layer_t layer, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);

	if (steep)
	{
		swap_ltdc(x0, y0);
		swap_ltdc(x1, y1);
	}

	if (x0 > x1)
	{
		swap_ltdc(x0, x1);
		swap_ltdc(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1)
	{
		ystep = 1;
	}
	else
	{
		ystep = -1;
	}

	for (; x0 <= x1; x0++)
	{
		if (steep)
		{
			lcd_ltdc_drawPixel(layer, y0, x0, color);
		}
		else
		{
			lcd_ltdc_drawPixel(layer, x0, y0, color);
		}

		err -= dy;

		if (err < 0)
		{
			y0 += ystep;
			err += dx;
		}
	}
}

void lcd_ltdc_drawFastVLine(layer_t layer, int16_t x, int16_t y, int16_t h, uint16_t color)
{
	/* Update in subclasses if desired! */
	lcd_ltdc_drawLine(layer, x, y, x, y + h - 1, color);
}

void lcd_ltdc_drawRect(layer_t layer, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	lcd_ltdc_drawFastVLine(layer, x, y, w, color);
	lcd_ltdc_drawFastVLine(layer, x, y + h - 1, w, color);
	lcd_ltdc_drawFastVLine(layer, x, y, h, color);
	lcd_ltdc_drawFastVLine(layer, x + w - 1, y, h, color);
}

void lcd_ltdc_fillRect(layer_t layer, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	/* Update in subclasses if desired! */
	int16_t i;

	for (i = x; i < x + w; i++)
		lcd_ltdc_drawFastVLine(layer, i, y, h, color);
}

void lcd_ltdc_fillCircleHelper(layer_t layer, int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color)
{
	int16_t f     = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x     = 0;
	int16_t y     = r;

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}

		x++;
		ddF_x += 2;
		f += ddF_x;

		if (cornername & 0x1)
		{
			lcd_ltdc_drawFastVLine(layer, x0+x, y0-y, 2*y+1+delta, color);
			lcd_ltdc_drawFastVLine(layer, x0+y, y0-x, 2*x+1+delta, color);
		}
		if (cornername & 0x2)
		{
			lcd_ltdc_drawFastVLine(layer, x0-x, y0-y, 2*y+1+delta, color);
			lcd_ltdc_drawFastVLine(layer, x0-y, y0-x, 2*x+1+delta, color);
		}
	}
}

void lcd_ltdc_fillCircle(layer_t layer, int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	lcd_ltdc_drawFastVLine(layer, x0, y0 - r, 2 * r + 1, color);
	lcd_ltdc_fillCircleHelper(layer, x0, y0, r, 3, 0, color);
}

void lcd_ltdc_mutate_background_color(void)
{
	static uint32_t ints;
	ints += 10;
	uint32_t shift = ints >> 9;

	if (shift >= 3) 
	{
		ints = shift = 0;
	}
	
	uint32_t component = ints & 0xFF;

	if (ints & 0x100)
	{
		component = 0xff - component;
	}

	hltdc.Instance->BCCR = component << 8 * shift;
}

void lcd_ltdc_move_sprite_ly2(void)
{
	static int8_t dx = 1, dy = 1;
	static int16_t x, y;
	static int16_t age;

	x += dx;
	y += dy;

	if (x < 0) 
	{
		dy = rand() % 7 - 3;
		dx = -dx;
		x = 0;
		age = 0;
		// lcd_ltdc_fillRect(LCD_LAYER_2, 0, 0, LCD_LAYER2_WIDTH_LTDC, LCD_LAYER2_WIDTH_LTDC, 0xFFF0);
		lcd_ltdc_fillCircle(LCD_LAYER_2, LCD_LAYER2_WIDTH_LTDC / 2, LCD_LAYER2_HEIGHT_LTDC / 2, LCD_LAYER2_WIDTH_LTDC / 2 - 5, 0xF0F0);
	}
	else if (x >= LCD_WIDTH - LCD_LAYER2_WIDTH_LTDC)
	{
		dy = rand() % 7 - 3;
		dx = -dx;
		x = LCD_WIDTH - LCD_LAYER2_WIDTH_LTDC - 1;
		age = 0;
		// lcd_ltdc_fillRect(LCD_LAYER_2, 0, 0, LCD_LAYER2_WIDTH_LTDC, LCD_LAYER2_WIDTH_LTDC, 0xFFF0);
		lcd_ltdc_fillCircle(LCD_LAYER_2, LCD_LAYER2_WIDTH_LTDC / 2, LCD_LAYER2_HEIGHT_LTDC / 2, LCD_LAYER2_WIDTH_LTDC / 2 - 5, 0xF00F);
	}
	if (y < 0)
	{
		dx = rand() % 7 - 3;
		dy = -dy;
		y = 0;
		age = 0;
		// lcd_ltdc_fillRect(LCD_LAYER_2, 0, 0, LCD_LAYER2_WIDTH_LTDC, LCD_LAYER2_WIDTH_LTDC, 0xFFF0);
		lcd_ltdc_fillCircle(LCD_LAYER_2, LCD_LAYER2_WIDTH_LTDC / 2, LCD_LAYER2_HEIGHT_LTDC / 2, LCD_LAYER2_WIDTH_LTDC / 2 - 5, 0xF0FF);
	}
	else if (y >= LCD_HEIGHT - LCD_LAYER2_HEIGHT_LTDC)
	{
		dx = rand() % 7 - 3;
		dy = -dy;
		y = LCD_HEIGHT - LCD_LAYER2_HEIGHT_LTDC - 1;
		age = 0;
		// lcd_ltdc_fillRect(LCD_LAYER_2, 0, 0, LCD_LAYER2_WIDTH_LTDC, LCD_LAYER2_WIDTH_LTDC, 0xFFF0);
		lcd_ltdc_fillCircle(LCD_LAYER_2, LCD_LAYER2_WIDTH_LTDC / 2, LCD_LAYER2_HEIGHT_LTDC / 2, LCD_LAYER2_WIDTH_LTDC / 2 - 5, 0xFF00);
	}
	if (dy == 0 && dx == 0) 
	{
		dy = y ? -1 : +1;
	}

	HAL_LTDC_SetWindowPosition(&hltdc, x, y, LCD_LAYER_2);

	/* The sprite fades away as it ages. */
	age += 1;

	if (age > 0xFF)
	{
		age = 0xFF;
	}

// #define LTDC_LAYER(__HANDLE__, __LAYER__)              ((LTDC_Layer_TypeDef *)((uint32_t)(((uint32_t)((__HANDLE__).Instance)) + 0x84U + (0x80U*(__LAYER__)))))
	// LTDC_L2CACR = 0x000000FF - age;
	// LTDC_LAYER(hltdc, LCD_LAYER_2)->CACR = 0x000000FF - age;
}

void lcd_ltdc_move_sprite_ly1(void)
{
	lcd_ltdc_fillRect(LCD_LAYER_1, touch_x, (LCD_LAYER1_HEIGHT - touch_y), 10, 10, 0xFF0F);
}

void lcd_ltdc_draw_layer_1(void)
{
	int row = 0, col = 0;

	for (row = 0; row < LCD_LAYER1_HEIGHT; row++)
	{
		for (col = 0; col < LCD_LAYER1_WIDTH_LTDC; col++)
		{
			size_t i = row * LCD_LAYER1_WIDTH_LTDC + col;

			uint8_t a = 0xF;
			uint8_t r = 0xF;
			uint8_t g = 0xF;
			uint8_t b = 0xF;

			r = rgb444_image_1[i] & 0xF;
			g = (rgb444_image_1[i] >> 4) & 0xF;
			b = (rgb444_image_1[i] >> 8) & 0xF;

			layer1_pixel pix = a << 12 | r << 8 | g << 4 | b << 0;
			lcd_layer1_frame_buffer[i] = pix;
		}
	}
}

void lcd_ltdc_draw_layer_2(void)
{
	int row, col;

	for (row = 0; row < LCD_LAYER2_HEIGHT_LTDC; row++)
	{
		for (col = 0; col < LCD_LAYER2_WIDTH_LTDC; col++)
		{
			size_t i = row * LCD_LAYER2_WIDTH_LTDC + col;
			uint8_t a = 0x0;

			layer2_pixel pix = a << 12;
			lcd_layer2_frame_buffer[i] = pix;
		}
	}

	lcd_ltdc_fillRect(LCD_LAYER_2, 0, 0, LCD_LAYER2_WIDTH_LTDC, LCD_LAYER2_WIDTH_LTDC, 0xFFF0);
	lcd_ltdc_fillCircle(LCD_LAYER_2, LCD_LAYER2_WIDTH_LTDC / 2, LCD_LAYER2_HEIGHT_LTDC / 2, LCD_LAYER2_WIDTH_LTDC / 2 - 5, 0xFF00);
}

void lcd_fill_buffers(void)
{
	/* load image in RAM */
	lcd_ltdc_draw_layer_1();
	lcd_ltdc_draw_layer_2();
}

static void lcd_init(const tft_ltdc_cmd_t cmds[], size_t cmd_count)
{
	size_t i;

	for (i = 0; i < cmd_count; i++)
	{
		uint8_t arg_count = cmds[i].n_args;
		uint8_t *args = cmds[i].args;
		
		if (arg_count > MAX_INLINE_ARGS) 
		{
			args = cmds[i].aptr;
		}

		lcd_command(cmds[i].cmd, cmds[i].delay, arg_count, args);
	}
}

void lcd_ltdc_init(void)
{
	/* Set up the display */
	lcd_init(ltdc_init_cmd, sizeof(ltdc_init_cmd) / sizeof(ltdc_init_cmd[0]));
}
