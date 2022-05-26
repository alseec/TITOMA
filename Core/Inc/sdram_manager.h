#ifndef _SDRAM_MANAGER_
#define _SDRAM_MANAGER_

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define SDRAM_BASE_ADDRESS          (void *) (0xD0000000)

extern SDRAM_HandleTypeDef hsdram1;

void sdram_print_line(uint8_t *addr);
void sdram_print_page(uint8_t *addr);
void sdram_print_screen(uint8_t *addr);
void sdram_handler(void);
void sdram_init_sequence(void);

#endif /* _SDRAM_MANAGER_ */