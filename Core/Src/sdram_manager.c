#include <string.h>
#include "sdram_manager.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

//SDRAM_HandleTypeDef hsdram1;

void sdram_print_line(uint8_t *addr)
{
  uint8_t idx;
	for (idx = 0; idx < 16; idx++)
	{
//		printf("%02X ", *(addr + idx));
		char aux_buff[10];
		memset(aux_buff, 0, sizeof(aux_buff));
		sprintf(aux_buff, "%02X ", *(addr + idx));
		CDC_Transmit_HS((uint8_t *) aux_buff, (uint16_t) strlen(aux_buff));
	}

  printf(" | ");

	for (idx = 0; idx < 16; idx++)
	{
		uint8_t b = *addr++;
//		printf("%c", ((b > 126) || (b < 32)) ? '.' : (char) b);

		char aux_buff[10];
		memset(aux_buff, 0, sizeof(aux_buff));
		sprintf(aux_buff, "%c", ((b > 126) || (b < 32)) ? '.' : (char) b);
		CDC_Transmit_HS((uint8_t *) aux_buff, (uint16_t) strlen(aux_buff));
	}

	printf("\r\n");
}

void sdram_print_page(uint8_t *addr)
{
	uint8_t idx;
	for (idx = 0; idx < 16; idx++)
		sdram_print_line(addr);
}

void sdram_handler(void)
{      
	uint8_t* addr = SDRAM_BASE_ADDRESS;
	sdram_print_page(addr);
}

void sdram_init_sequence(void)
{
	  FMC_SDRAM_CommandTypeDef SDRAM_CommandStructure;

#if 1
	/* Configure a clock configuration enable command */
	SDRAM_CommandStructure.CommandMode              = FMC_SDRAM_CMD_CLK_ENABLE;
	SDRAM_CommandStructure.CommandTarget            = FMC_SDRAM_CMD_TARGET_BANK2;
	SDRAM_CommandStructure.AutoRefreshNumber        = 1;
	SDRAM_CommandStructure.ModeRegisterDefinition   = 0;
	HAL_SDRAM_SendCommand(&hsdram1, &SDRAM_CommandStructure, 100);
#endif

#if 1
	/* Configure a PALL (precharge all) command */ 
	SDRAM_CommandStructure.CommandMode              = FMC_SDRAM_CMD_PALL;
	SDRAM_CommandStructure.CommandTarget            = FMC_SDRAM_CMD_TARGET_BANK2;
	SDRAM_CommandStructure.AutoRefreshNumber        = 1;
	SDRAM_CommandStructure.ModeRegisterDefinition   = 0;
	HAL_SDRAM_SendCommand(&hsdram1, &SDRAM_CommandStructure, 100);
#endif

#if 1
	/* Configure a Auto-Refresh command */ 
	SDRAM_CommandStructure.CommandMode              = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	SDRAM_CommandStructure.CommandTarget            = FMC_SDRAM_CMD_TARGET_BANK2;
	SDRAM_CommandStructure.AutoRefreshNumber        = 1;
	SDRAM_CommandStructure.ModeRegisterDefinition   = 0;
	HAL_SDRAM_SendCommand(&hsdram1, &SDRAM_CommandStructure, 100);
#endif

#if 1
	/* Configure a load Mode register command */
	SDRAM_CommandStructure.CommandMode              = FMC_SDRAM_CMD_LOAD_MODE;
	SDRAM_CommandStructure.CommandTarget            = FMC_SDRAM_CMD_TARGET_BANK2;
	SDRAM_CommandStructure.AutoRefreshNumber        = 1;
	SDRAM_CommandStructure.ModeRegisterDefinition   = 0;
	HAL_SDRAM_SendCommand(&hsdram1, &SDRAM_CommandStructure, 100);
#endif

  /*
    Set the refresh rate counter:
    SDRAM refresh rate      = 60ms
    refresh rate per ROW    = 60ms / 2^12 = 15.625us

	If main clock is 180MHz:
		Refresh count           = 15.625us * 90MHz = 1406
		Applying safe margin    = 1406 - 20 = 1386

	If main clock is 120MHz:
		Refresh count           = 15.625us * 60MHz = 937.5
		Applying safe margin    = 937.5 - 20 = 917.5
   */
#define SDRAM_REFRESH_COUNT   917
  HAL_SDRAM_SetAutoRefreshNumber(&hsdram1, SDRAM_REFRESH_COUNT);
}
