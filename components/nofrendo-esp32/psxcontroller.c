// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"


#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "psxcontroller.h"
#include "sdkconfig.h"
#include "pretty_effect.h"
#include "esp_deep_sleep.h"

#define PSX_CLK CONFIG_HW_PSX_CLK
#define PSX_DAT CONFIG_HW_PSX_DAT
#define PSX_ATT CONFIG_HW_PSX_ATT
#define PSX_CMD CONFIG_HW_PSX_CMD

#define DELAY() asm("nop; nop; nop; nop;nop; nop; nop; nop;nop; nop; nop; nop;nop; nop; nop; nop;")

#if CONFIG_HW_PSX_ENA

int volume,bright;
int inpDelay;
bool shutdown;

/* Sends and receives a byte from/to the PSX controller using SPI */
static int psxSendRecv(int send) {
	int x;
	int ret=0;
	volatile int delay;
	
#if 0
	while(1) {
		GPIO.out_w1ts=(1<<PSX_CMD);
		GPIO.out_w1ts=(1<<PSX_CLK);
		GPIO.out_w1tc=(1<<PSX_CMD);
		GPIO.out_w1tc=(1<<PSX_CLK);
	}
#endif

	GPIO.out_w1tc=(1<<PSX_ATT);
	for (delay=0; delay<100; delay++);
	for (x=0; x<8; x++) {
		if (send&1) {
			GPIO.out_w1ts=(1<<PSX_CMD);
		} else {
			GPIO.out_w1tc=(1<<PSX_CMD);
		}
		DELAY();
		for (delay=0; delay<100; delay++);
		GPIO.out_w1tc=(1<<PSX_CLK);
		for (delay=0; delay<100; delay++);
		GPIO.out_w1ts=(1<<PSX_CLK);
		ret>>=1;
		send>>=1;
		if (GPIO.in&(1<<PSX_DAT)) ret|=128;
	}
	return ret;
}

bool showMenu;

static void psxDone() {
	DELAY();
	GPIO_REG_WRITE(GPIO_OUT_W1TS_REG, (1<<PSX_ATT));
}

bool getShowMenu(){
	return showMenu;
}


int psxReadInput() {
	/*int b1, b2;

	psxSendRecv(0x01); //wake up
	psxSendRecv(0x42); //get data
	psxSendRecv(0xff); //should return 0x5a
	b1=psxSendRecv(0xff); //buttons byte 1
	b2=psxSendRecv(0xff); //buttons byte 2*/
	int b2b1 = 65535;
	if(inpDelay>0)inpDelay--;
	
	if(showMenu){
		if(gpio_get_level(34)==1 && inpDelay==0){
			if(volume<4)volume+=1;
			inpDelay=15;
		}
		if(gpio_get_level(33)==1 && inpDelay==0){
			if(volume>0)volume-=1;
			inpDelay=15;
		}
		if(gpio_get_level(32)==1 && inpDelay==0){
			if(bright<4)bright+=1;
			inpDelay=15;
		}
		if(gpio_get_level(39)==1 && inpDelay==0){
			if(bright>0)bright-=1;
			inpDelay=15;
		}
		if(gpio_get_level(35)==1 && inpDelay==0){
			getYStretch() ? setYStretch(0) : setYStretch(1);
			inpDelay=15;
		}
		if(gpio_get_level(13)==1&& inpDelay==0){
			getXStretch() ? setXStretch(0) : setXStretch(1);
			inpDelay=15;
		}
	}
	//Bit0 Bit1 Bit2 Bit3 Bit4 Bit5 Bit6 Bit7
    //SLCT           STRT UP   RGHT DOWN LEFT
    //L2   R2    L1  R1   /\   O    X    |_|
	else{
		if(gpio_get_level(34)==1)b2b1-=16;//up
		if(gpio_get_level(33)==1)b2b1-=64;//down
		if(gpio_get_level(32)==1)b2b1-=32;//right
		if(gpio_get_level(39)==1)b2b1-=128;//left
		if(gpio_get_level(17)==1)b2b1-=1;//select
		if(gpio_get_level(14)==1)b2b1-=8;//start
		if(gpio_get_level(35)==1)b2b1-=8192*2;//4096;//B
		if(gpio_get_level(13)==1)b2b1-=8192;//A
	}
	//Button2
	if(gpio_get_level(16)==1 && inpDelay==0){
		showMenu=showMenu ? 0 : 1;
		inpDelay=15;
	}
	//Button1
	if(gpio_get_level(12)==1){
		bright = -1;
		inpDelay+=2;
		printf("delay %d\n",inpDelay);
	}
	
	if(bright==-1 && inpDelay==0){
		esp_sleep_enable_timer_wakeup(1000*100);
		vTaskDelay(100);
		esp_deep_sleep_start();
	}
	
	if(bright==-1 && inpDelay>100){
		bright=-2;
		shutdown=1;
		esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
		gpio_pullup_dis(12);
		gpio_pulldown_en(12);
		esp_deep_sleep_enable_ext0_wakeup(12, 1);
		
		vTaskDelay(1500);
		esp_deep_sleep_start();
	}
	
	return b2b1;
	//psxDone();
	//return (b2<<8)|b1;

}

int getBright(){
	return bright;
}

int getVolume(){
	return volume;
}

bool getShutdown(){
	return shutdown;
}

void psxcontrollerInit() {
	volatile int delay;
	int t;
	showMenu=0;
	shutdown=0;
	/*gpio_config_t gpioconf[2]={
		{
			.pin_bit_mask=(1<<PSX_CLK)|(1<<PSX_CMD)|(1<<PSX_ATT), 
			.mode=GPIO_MODE_OUTPUT, 
			.pull_up_en=GPIO_PULLUP_DISABLE, 
			.pull_down_en=GPIO_PULLDOWN_DISABLE, 
			.intr_type=GPIO_PIN_INTR_DISABLE
		},{
			.pin_bit_mask=(1<<PSX_DAT), 
			.mode=GPIO_MODE_INPUT, 
			.pull_up_en=GPIO_PULLUP_ENABLE, 
			.pull_down_en=GPIO_PULLDOWN_DISABLE, 
			.intr_type=GPIO_PIN_INTR_DISABLE
		}
	};
	gpio_config(&gpioconf[0]);
	gpio_config(&gpioconf[1]);*/
	
	//Send a few dummy bytes to clean the pipes.
	psxSendRecv(0);
	psxDone();
	for (delay=0; delay<500; delay++) DELAY();
	psxSendRecv(0);
	psxDone();
	for (delay=0; delay<500; delay++) DELAY();
	//Try and detect the type of controller, so we can give the user some diagnostics.
	psxSendRecv(0x01);
	t=psxSendRecv(0x00);
	psxDone();
	if (t==0 || t==0xff) {
		printf("No PSX/PS2 controller detected (0x%X). You will not be able to control the game.\n", t);
	} else {
		printf("PSX controller type 0x%X\n", t);
	}
	inpDelay=0;
	volume=0;
	bright=2;
}


#else

int psxReadInput() {
	return 0xFFFF;
}


void psxcontrollerInit() {
	printf("PSX controller disabled in menuconfig; no input enabled.\n");
}

#endif