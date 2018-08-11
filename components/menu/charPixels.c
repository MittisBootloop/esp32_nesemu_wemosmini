#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "charData.c"
#include <string.h>
#include "iconData.c"
#include "pretty_effect.h"

bool endOfLine;
bool endOfFile;
int yLineEnded;
int linesRomList;
int charOff;
bool endCharLine;
int change;
int lineCounter;
				 
char *lines;//[1000];

bool cpGetPixel(char cpChar, int cp1, int cp2){
	return getPixel(cpChar,cp1,cp2);
}

//Load Rom list from flash partition to char array(lines), init some variables for printing rom list
void initRomList() {
	char* romdata;
	const esp_partition_t* part;
	spi_flash_mmap_handle_t hrom;
	esp_err_t err;
	nvs_flash_init();
	part=esp_partition_find_first(0x40, 1, NULL);
	if(part==0) strcpy(lines, "No Rom List Found\n*");
	err=esp_partition_mmap(part, 0, /*3*1024*1024*/4*1024, SPI_FLASH_MMAP_DATA, (const void**)&romdata, &hrom);
	if (err!=ESP_OK) {
		//printf("No Rom list found\n");
		strcpy(lines, "No Rom List Found\n*");
	}
	for(int i=0; i < 1000; i++){
		if(romdata[i] == '*'){
			lines=calloc(i+5, sizeof(char));
			break;
		}
	}
	
	for(int i=0; i < 1000; i++){
		lines[i]=romdata[i];
		if(romdata[i]=='\n') lineCounter+=1;
		if(romdata[i] == '*'){
			break;
		}
	}
	setLineMax(lineCounter-2);
	printf("lineMax = %d\n",lineCounter);
	
	endOfFile=0;
	endOfLine=0;
	charOff=0;
	change=0;
}
					 
//get depending on x/y value the actual char from lines array
//depending on x/y/actChar return color/black if char value is true/false(charData.c)
//depending on x/y/actChar read and return icon pixel color(iconData.c)
int getCharPixel(int x, int y, int change, int choosen){
	int line 	= ((y-3)/18)+1;
	int charNo 	= (x-26)/16;
	int lineLength;
	char actChar;
	int page = choosen/13;
	lineCounter=0;

	
	if(x==7)endOfLine=0;
	if(!endOfLine){
		for(lineLength = 0; lineLength <1000; lineLength++){
			if(lines[lineLength]=='\n') lineCounter+=1;
			if (lines[lineLength]=='.' && lineCounter==line+13*page){
					break;
			}
			if(lines[lineLength]=='*')return 0x0000;
		}
		if(x>=7 && x<=22){
			if(line-1==choosen%13)return getIconPixel(lines[lineLength+2],x-7,(y-5)%18, change);
			else return 0x0000;
		}
		actChar=lines[lineLength+2+2+charNo];
		if(actChar=='\n'){
			endOfLine=1;
			return 0x0000;
		}
		if(actChar=='\r')return 0x0000;
	}
	else return 0x0000;
	
	if(getPixel(actChar,(x-26)%16, (y-3)%18)==1)return 0x001F;
	return 0x0000;
}

void freeRL(){
	free(lines);
}