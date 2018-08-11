//#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "nofrendo.h"
#include "menu.h"

int romPartition;

char *osd_getromdata() {
	//printf("choosen: %d\n",romPartition);
	char* romdata;
	const esp_partition_t* part;
	spi_flash_mmap_handle_t hrom;
	esp_err_t err;
	nvs_flash_init();
	
	
	part=esp_partition_find_first(0x41+romPartition, 1, NULL);
	if (part==0) printf("Couldn't find rom part!\n");
	
	int partSize;
	switch(romPartition) {
		case 0: case 5: case 6: case 7: 	partSize = 100; break;
		case 1: case 4: case 11: case 12:	partSize = 260; break;
		case 2:								partSize = 388; break;
		case 3: case 13:					partSize = 132; break;
		case 8:								partSize = 772; break;
		case 9:								partSize = 516; break;
		case 10:							partSize = 296; break;
		default:							partSize = 0; break;
	}
	err=esp_partition_mmap(part, 0, partSize*1024, SPI_FLASH_MMAP_DATA, (const void**)&romdata, &hrom);
	if (err!=ESP_OK) printf("Couldn't map rom part!\n");
	printf("Initialized. ROM@%p\n", romdata);
    return (char*)romdata;
}

void esp_wake_deep_sleep(){
	esp_restart();
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

int app_main(void)
{
	romPartition=runMenu();
	printf("NoFrendo start!\n");
	nofrendo_main(0, NULL);
	printf("NoFrendo died? WtF?\n");
	asm("break.n 1");
    return 0;
}

