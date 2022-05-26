#include "lcd_manager.h"
#include "sdram_manager.h"
#include "touch_manager.h"

//SPI_HandleTypeDef hspi5;

/*
* Simple double buffering, one frame is displayed, the
* other being built.
*/
uint16_t *cur_frame;
uint16_t *display_frame;

/*
* This creates a 'script' of commands that can be played
* to the LCD controller to initialize it.
* One array holds the 'argument' bytes, the other
* the commands.
* Keeping them in sync is essential
*/
const uint8_t frame_control_arg[] 			= {0x00, 0x1B};
const uint8_t disp_func_ctrl_arg[] 			= {0x0A, 0xA2}; //TODO
const uint8_t power_ctrl_1_arg[] 			= {0x10}; //TODO
const uint8_t power_ctrl_2_arg[] 			= {0x10}; //TODO
const uint8_t vcom_ctrl_1_arg[] 			= {0x45, 0x15};//TODO
const uint8_t vcom_ctrl_2_arg[] 			= {0x90}; //TODO
const uint8_t memory_access_ctrl_arg[] 		= {0x08};
const uint8_t rgb_intrf_signal_crtl_arg[] 	= {0xC2}; //TODO
const uint8_t pixel_format_set_arg[] 		= {0x55};
const uint8_t column_addr_set_arg[] 		= {0x00, 0x00, 0x00, 0xEF};
const uint8_t page_addr_set_arg[] 			= {0x00, 0x00, 0x01, 0x3F};
const uint8_t intrf_ctrl_arg[] 				= {0x01, 0x00, 0x00};
const uint8_t gamma_set_arg[] 				= {0x01};

const uint8_t pos_gamma_corr_arg[] 			= {0x0F, 0x29, 0x24, 0x0C, 0x0E, 0x09, 0x4E, 0x78, 0x3C, 0x09, 0x13, 0x05, 0x17, 0x11, 0x00};
// const uint8_t pos_gamma_corr_arg[] 			= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// const uint8_t pos_gamma_corr_arg[] 			= {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const uint8_t neg_gamma_corr_arg[] 			= {0x00, 0x16, 0x1B, 0x04, 0x11, 0x07, 0x31, 0x33, 0x42, 0x05, 0x0C, 0x0A, 0x28, 0x2F, 0x0F};
// const uint8_t neg_gamma_corr_arg[] 			= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
// const uint8_t neg_gamma_corr_arg[] 			= {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

const uint8_t cmd_args[] = {
	0x00, 0x1B,						/* Frame Control (In Normal Mode) */
	0x0A, 0xA2,						/* Display Function Control	 	  */

	0x10,							/* Power Control 1				  */
	0x10,							/* Power Control 2		 		  */

	0x45, 0x15,						/* VCOM Control 1				  */

	0x90,							/* VCOM Control 2 				  */
	0x08,							/* Memory Access Control		  */
	0xC2,							/* RGB Interface Signal Control   */
	0x55,							/* Pixel Format Set			 	  */

	0x0A, 0xA7, 0x27, 0x04,			/* Display Function Control	 	  */
	0x00, 0x00, 0x00, 0xEF,			/* Column Address Set		 	  */
	0x00, 0x00, 0x01, 0x3F,			/* Page Address Set			 	  */

	0x01, 0x00, 0x00,               /* Interface Control		 	  */

	0x01,							/* Gamma Set				 	  */

	0x0F, 0x29, 0x24, 0x0C, 0x0E,	/* Positive Gamma Correction 	  */
	0x09, 0x4E, 0x78, 0x3C, 0x09,
	0x13, 0x05, 0x17, 0x11, 0x00,

	0x00, 0x16, 0x1B, 0x04, 0x11,	/* Negative Gamma Correction 	  */
	0x07, 0x31, 0x33, 0x42, 0x05,
	0x0C, 0x0A, 0x28, 0x2F, 0x0F,
};

/*
* These are the commands we're going to send to the
* display to initialize it. We send them all, in sequence
* with occasional delays. Commands that require data bytes
* as arguments, indicate how many bytes to pull out the
* above array to include.
*
* The sequence was pieced together from the ST Micro demo
* code, the data sheet, and other sources on the web.
*/
const lcd_cmds_t init_cmd[] = {
	{.delay = 0, 	.cmd = 0xB1, 	.args_len = 2,		.cmd_data = frame_control_arg},
	// {.delay = 0, 	.cmd = 0xB6, 	.args_len = 2,		.cmd_data = disp_func_ctrl_arg},
	// {.delay = 0, 	.cmd = 0xC0, 	.args_len = 1,		.cmd_data = power_ctrl_1_arg},
	// {.delay = 0, 	.cmd = 0xC1, 	.args_len = 1,		.cmd_data = power_ctrl_2_arg},
	// {.delay = 0, 	.cmd = 0xC5, 	.args_len = 2,		.cmd_data = vcom_ctrl_1_arg},
	// {.delay = 0, 	.cmd = 0xC7, 	.args_len = 1,		.cmd_data = vcom_ctrl_2_arg},
	{.delay = 0, 	.cmd = 0x36, 	.args_len = 1,		.cmd_data = memory_access_ctrl_arg},
	// {.delay = 0, 	.cmd = 0xB0, 	.args_len = 1,		.cmd_data = rgb_intrf_signal_crtl_arg},
	{.delay = 0, 	.cmd = 0x3A, 	.args_len = 1,		.cmd_data = pixel_format_set_arg},
	// {.delay = 0, 	.cmd = 0x2A, 	.args_len = 4,		.cmd_data = column_addr_set_arg},
	// {.delay = 0, 	.cmd = 0x2B, 	.args_len = 4,		.cmd_data = page_addr_set_arg},
	// {.delay = 0, 	.cmd = 0xF6, 	.args_len = 3,		.cmd_data = intrf_ctrl_arg},

	// {.delay = 200, 	.cmd = 0x2C, 	.args_len = 0, 		.cmd_data = NULL},

	{.delay = 0, 	.cmd = 0x26, 	.args_len = 1, 		.cmd_data = gamma_set_arg},
	{.delay = 0, 	.cmd = 0xE0, 	.args_len = 15, 	.cmd_data = pos_gamma_corr_arg},
	{.delay = 0, 	.cmd = 0xE1, 	.args_len = 15, 	.cmd_data = neg_gamma_corr_arg},

	{.delay = 200, 	.cmd = 0x11, 	.args_len = 0, 		.cmd_data = NULL},
	{.delay = 0, 	.cmd = 0x29, 	.args_len = 0, 		.cmd_data = NULL},
	{.delay = 0, 	.cmd = 0x00, 	.args_len = 0, 		.cmd_data = NULL},
};

/*
* void lcd_command(cmd, delay, args, arg_ptr)
*
* All singing all dancing 'do a command' feature. Basically it
* sends a command, and if args are present it sets 'data' and
* sends those along too.
*/
void lcd_command(uint8_t cmd, int delay, int args_len, uint8_t *args)
{
	int i;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); 	/* Select the LCD */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); 	/* Reset the D/CX pin for commands */

	if (HAL_SPI_Transmit(&hspi5, &cmd, 1, 100) != HAL_OK)
		printf("[SPI]TX_error\r\n");

	if (args_len)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); /* Set the D/CX pin for data */

		for (i = 0; i < args_len; i++)
		{
			if (HAL_SPI_Transmit(&hspi5, (args + i), 1, 100) != HAL_OK)
				printf("[SPI]TX_error\r\n");
		}
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); 	/* Turn off chip select */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); 	/* Always reset D/CX */

	if (delay)
		HAL_Delay(delay);		/* wait, if called for */
}

/*
* Drawing a pixel consists of storing a 16 bit value in the
* memory used to hold the frame. This code computes the address
* of the word to store, and puts in the value we pass to it.
*/
void lcd_draw_pixel(int x, int y, uint16_t color)
{
	*(cur_frame + x + y * LCD_WIDTH) = color;
}

/*
* Interesting questions:
*   - How quickly can I write a full frame?
*      * Take the bits sent (16 * width * height)
*        and divide by the  baud rate (10.25Mhz)
*      * Tests in main.c show that yes, it taks 74ms.
*
* Create non-random data in the frame buffer. In our case
* a black background and a grid 16 pixels x 16 pixels of
* white lines. No line on the right edge and bottom of screen.
*/
void lcd_test_image(void)
{
	int x, y;
	uint16_t pixel;

	for (x = 0; x < LCD_WIDTH; x++)
	{
		for (y = 0; y < LCD_HEIGHT; y++)
		{
#if 0
			// pixel = 0;			/* all black */
			/* Take care the order of the bytes sent */
			// pixel = 0xF800;			/* LCD_BLUE 0b11111000 00000000 0xF800 */
			// pixel = 0xE007;			/* LCD_GREEN 0b11100000 00000111 0xE007 */
			// pixel = 0x00F8;			/* LCD_RED 0b00000000 11111000 0x00F8 */
			if ((x % 16) == 0)
				pixel = 0xFFFF;		/* all white */

			if ((y % 16) == 0)
				pixel = 0xFFFF;		/* all white */
#else
			pixel = elnic_map[x + y * LCD_WIDTH];
#endif

			lcd_draw_pixel(x, y, pixel);
		}
	}
}

/*
* void lcd_show_frame(void)
*
* Dump an entire frame to the LCD all at once. In theory you
* could call this with DMA but that is made more difficult by
* the implementation of SPI and the modules interpretation of
* D/CX line.
*/
void lcd_show_frame(void)
{
	uint8_t size[4];

	// uint16_t *aux_buff = display_frame;
	display_frame = cur_frame;
	// cur_frame = aux_buff;

	size[0] = 0; 						/* SC */
	size[1] = 0; 						/* SC */
	size[2] = (LCD_WIDTH >> 8) & 0xFF; 	/* EC */
	size[3] = (LCD_WIDTH) & 0xFF; 		/* EC */
	uint8_t column_addr_set_arg = 0x2A;
	lcd_command(column_addr_set_arg, 0, 4, &size[0]);

	size[0] = 0; 						/* EP */
	size[1] = 0; 						/* EP */
	size[2] = (LCD_HEIGHT >> 8) & 0xFF; /* EP */
	size[3] = LCD_HEIGHT & 0xFF; 		/* EP */
	uint8_t page_addr_set_arg = 0x2B;
	lcd_command(page_addr_set_arg, 0, 4, &size[0]);

	uint8_t memory_write_arg = 0x2C;
	lcd_command(memory_write_arg, 0, FRAME_SIZE_BYTES, (uint8_t *) display_frame); /* 320 * 240 * 2 (16 bits color) */
}

/*
* void lcd_start(struct cmds[])
*
* This is the function that sends the entire list. It also puts
* the commands it is sending to the console.
*/
void lcd_start(const lcd_cmds_t cmds[])
{
	int com_idx = 0;

	while (cmds[com_idx].cmd)
	{
		printf("CMD: %02X, ", cmds[com_idx].cmd);

		if (cmds[com_idx].args_len)
		{
			printf("ARGS: ");

			uint8_t j;
			for (j = 0; j < cmds[com_idx].args_len; j++)
				printf("%02X, ", cmds[com_idx].cmd_data[j]);
		}

		printf("\r\nDELAY: %d ms", cmds[com_idx].delay);

		lcd_command(cmds[com_idx].cmd, cmds[com_idx].delay, cmds[com_idx].args_len, cmds[com_idx].cmd_data);
		com_idx++;
	}

	printf("Done\r\n");
}

void lcd_spi_init(void)
{
	cur_frame = (uint16_t *) (SDRAM_BASE_ADDRESS);
	display_frame = cur_frame + (LCD_WIDTH * LCD_HEIGHT);

	lcd_start(init_cmd);
	lcd_test_image();
	// gfx_fill_screen(LCD_CYAN);
	lcd_show_frame();
}

void lcd_handler_run(void)
{
	static int p1 = 0;
	static int p2 = 45;
	static int p3 = 90;

	gfx_fill_screen(LCD_BLACK);
	// gfx_setCursor(15, 30);

	// gfx_setTextColor(LCD_YELLOW, LCD_BLACK);
	// gfx_setTextSize(3);
	// gfx_puts("PLANETS!");

	gfx_fill_circle(LCD_WIDTH / 2, LCD_HEIGHT / 2, 40, LCD_YELLOW);
	gfx_draw_circle(LCD_WIDTH / 2, LCD_HEIGHT / 2, 55, LCD_GREY);
	gfx_draw_circle(LCD_WIDTH / 2, LCD_HEIGHT / 2, 75, LCD_GREY);
	gfx_draw_circle(LCD_WIDTH / 2, LCD_HEIGHT / 2, 100, LCD_GREY);

	gfx_fill_circle(120 + (sin(d2r(p1)) * 55),  160 + (cos(d2r(p1)) * 55), 5, LCD_RED);
	gfx_fill_circle(120 + (sin(d2r(p2)) * 75),  160 + (cos(d2r(p2)) * 75), 10, LCD_WHITE);
	gfx_fill_circle(120 + (sin(d2r(p3)) * 100), 160 + (cos(d2r(p3)) * 100), 8, LCD_BLUE);

	p1 = (p1 + 3 * 10) % 360;
	p2 = (p2 + 2 * 10) % 360;
	p3 = (p3 + 1 * 10) % 360;

	lcd_show_frame();
}
