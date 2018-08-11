/*
   This code generates an effect that should pass the 'fancy graphics' qualification
   as set in the comment in the spi_master code.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_system.h>
#include <math.h>
#include "pretty_effect.h"
#include "decode_image.h"
#include "driver/gpio.h"
#include "charPixels.c"
#include "esp_deep_sleep.h"
#include "menu.h"

uint16_t **pixels;
int yOff;
int newX;
int bootTV;
int whiteN;
int slow;
int page;
int test;
int change;
int choosen;
int inputDelay;
int lineMax;
int selRom;

bool xStretch;
bool yStretch;

void setBright(int bright){
	setBr(bright);
}
	
bool peGetPixel(char peChar, int pe1, int pe2){
	return cpGetPixel(peChar,pe1,pe2);
}

bool getYStretch(){
	return yStretch;
}

bool getXStretch(){
	return xStretch;
}

void setXStretch(bool str){
	xStretch=str;
}

void setYStretch(bool str){
	yStretch=str;
}
void setLineMax(int lineM){
	lineMax = lineM;
}

void setSelRom(int selR){
	selRom=selR;
}

int getSelRom(){
	return selRom;
}
//!!! Colors repeat after 3Bit(example: 001 = light green, 111 = max green -> 1000 = again light green),
//		 so all values over (dec) 7 start to repeat the color, but they are stored in 5bits!!!  
//returns a 16bit rgb Color (1Bit + 15Bit bgr), values for each Color from 0-31
//(MSB=? + 5Bits blue + 5Bits red + 5Bits green)
int rgbColor(int red, int green, int blue){
	return 0x8000+blue*1024+red*32+green;
}

int getNoise(){
	whiteN=rand()%8;
	return rgbColor(whiteN,whiteN,whiteN);
}

//Grab a rgb16 pixel from the esp32_tiles image, scroll part of it in
static inline uint16_t bootScreen(int x, int y, int yOff, int bootTV){
	if(gpio_get_level(14)==1)test=0;
	if(bootTV<slow*251 && bootTV>slow*150) return getNoise();
	else if(bootTV>0){
		if(x>250 && x<284 && y>210 && y<228){
			int xAct = ((x-250)/2)+96;
			int yAct = ((y-210)/2)+210;
			if(pixels[yAct+30][xAct]<0x8000+1000)return 0x8000+31;
			else return getNoise();
		}
		return getNoise();
	}
	if(x>=105 && x <=216){
	if(y<65 && pixels[y][x-104]!=0x0000 ){
		return pixels[y][x-104];
	}
	else y=y-yOff/8;
	
	if(y<65 || pixels[y][x-104]==0x0000){		
		return getNoise();
	}

    return pixels[y][x-104];}
	else return getNoise();
}

//run "boot screen" (intro) and later menu to choose a rom
static inline uint16_t get_bgnd_pixel(int x, int y, int yOff, int bootTV, int choosen1)
{
	page=0;
	if(test>=0){
		if(y==0)test--;
		return bootScreen(x,y,yOff,bootTV);
	}
	
	if(x<=6 || x>=313 || y<3 || y>236 || (x>=23 && x<=25)) return 0x0000;
	if((y-3)%18<2 || (y-3)%18>=16) return 0x0000;

	return getCharPixel(x, y, change, choosen1);
}


//This variable is used to detect the next frame.
static int prev_frame=-1;

//Instead of calculating the offsets for each pixel we grab, we pre-calculate the valueswhenever a frame changes, then re-use
//these as we go through all the pixels in the frame. This is much, much faster.
/*static int8_t xofs[320], yofs[240];
static int8_t xcomp[320], ycomp[240];*/

//Calculate the pixel data for a set of lines (with implied line size of 320). Pixels go in dest, line is the Y-coordinate of the
//first line to be calculated, linect is the amount of lines to calculate. Frame increases by one every time the entire image
//is displayed; this is used to go to the next frame of animation.

void pretty_effect_calc_lines(uint16_t *dest, int line, int frame, int linect)
{
	if(bootTV>0)bootTV--;
    if(yOff>0 && bootTV==0)yOff--;
	
	if(inputDelay>0)inputDelay-=1;
	if(gpio_get_level(34)==1 && inputDelay==0 && choosen>0){
		choosen-=1;
		inputDelay=200;
	}
	if(gpio_get_level(33)==1 && inputDelay==0 && choosen<lineMax){
		choosen+=1;
		inputDelay=200;
	}
	if(gpio_get_level(13)==1) selRom=choosen;
	if(gpio_get_level(12)==1){
		//gpio_set_level(5, 0);
		esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
		gpio_pullup_dis(12);
		gpio_pulldown_en(12);
		esp_deep_sleep_enable_ext0_wakeup(12, 1);
		
		vTaskDelay(1000);
		esp_deep_sleep_start();
	}

	if (frame!=prev_frame) {
		//variable for blinking icons - very ugly solution, i know
		change+=1;
		if(change == 30)change = 0;
        prev_frame=frame;
    }
	
    for (int y=line; y<line+linect; y++) {
		for (int x=0; x<320; x++){
			*dest++=get_bgnd_pixel(x, y, yOff, bootTV, choosen);
        }
    }
}

void initGPIO(int gpioNo){
	gpio_set_direction(gpioNo, GPIO_MODE_INPUT);
	gpio_pulldown_en(gpioNo);
}

void freeMem(){
	for (int i=0; i<256; i++) {
            free((pixels)[i]);
        }
    free(pixels);
	freeRL();
}

//initialize varibles for "timers" and input, gpios and load picture
esp_err_t pretty_effect_init() 
{
	slow=4;
    yOff=slow*880;
	bootTV=slow*250;
	test=slow*6000;
	choosen=0;
	inputDelay=0;
	lineMax = 0;
	yStretch=0;
	xStretch=0;
	initRomList();
	initGPIO(34);
	initGPIO(35);
	initGPIO(32);
	initGPIO(39);
	initGPIO(17);
	initGPIO(14);
	initGPIO(12);
	initGPIO(13);
	initGPIO(33);
	initGPIO(16);
	return decode_image(&pixels);
}
