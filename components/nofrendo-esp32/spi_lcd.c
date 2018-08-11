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

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/periph_ctrl.h"
#include "spi_lcd.h"
#include "psxcontroller.h"
#include "driver/ledc.h"
#include "pretty_effect.h"

#define PIN_NUM_MISO CONFIG_HW_LCD_MISO_GPIO
#define PIN_NUM_MOSI CONFIG_HW_LCD_MOSI_GPIO
#define PIN_NUM_CLK  CONFIG_HW_LCD_CLK_GPIO
#define PIN_NUM_CS   CONFIG_HW_LCD_CS_GPIO
#define PIN_NUM_DC   CONFIG_HW_LCD_DC_GPIO
#define PIN_NUM_RST  CONFIG_HW_LCD_RESET_GPIO
//#define PIN_NUM_BCKL CONFIG_HW_LCD_BL_GPIO
#define LCD_SEL_CMD()   GPIO.out_w1tc = (1 << PIN_NUM_DC) // Low to send command 
#define LCD_SEL_DATA()  GPIO.out_w1ts = (1 << PIN_NUM_DC) // High to send data
#define LCD_RST_SET()   GPIO.out_w1ts = (1 << PIN_NUM_RST) 
#define LCD_RST_CLR()   GPIO.out_w1tc = (1 << PIN_NUM_RST)

#define LEDC_LS_TIMER          	LEDC_TIMER_1
#define LEDC_LS_MODE           	LEDC_LOW_SPEED_MODE
#define PIN_NUM_BCKL 			27//CONFIG_HW_LCD_BL_GPIO
#define LEDC_LS_CH3_CHANNEL    	LEDC_CHANNEL_3

#if CONFIG_HW_INV_BL
#define LCD_BKG_ON()    GPIO.out_w1tc = (1 << PIN_NUM_BCKL) // Backlight ON
#define LCD_BKG_OFF()   GPIO.out_w1ts = (1 << PIN_NUM_BCKL) //Backlight OFF
#else
#define LCD_BKG_ON()    GPIO.out_w1ts = (1 << PIN_NUM_BCKL) // Backlight ON
#define LCD_BKG_OFF()   GPIO.out_w1tc = (1 << PIN_NUM_BCKL) //Backlight OFF
#endif

#define SPI_NUM  0x3

#define LCD_TYPE_ILI 0
#define LCD_TYPE_ST 1

ledc_channel_config_t ledc_channel;

/*void initBCKL(){
	ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 500,
        .speed_mode = LEDC_LS_MODE,
		.timer_num = LEDC_LS_TIMER           
    };
    
    ledc_timer_config(&ledc_timer);
	
	ledc_channel.channel    = LEDC_LS_CH3_CHANNEL;
	ledc_channel.duty       = 500;
	ledc_channel.gpio_num   = PIN_NUM_BCKL;
	ledc_channel.speed_mode = LEDC_LS_MODE;
	ledc_channel.timer_sel  = LEDC_LS_TIMER;
								
	ledc_channel_config(&ledc_channel);
}*/

void setBrightness(int bright){
	/*int duty=1000;
	if(bright == -2)duty=0;
	if(bright == 0)duty=1000;
	if(bright == 1)duty=2050;
	if(bright == 2)duty=4100;
	if(bright == 3)duty=6150;
	if(bright == 4)duty=8190;
	ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);*/
	setBright(bright);
}

static void spi_write_byte(const uint8_t data){
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 0x7, SPI_USR_MOSI_DBITLEN_S);
    WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), data);
    SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
}

static void LCD_WriteCommand(const uint8_t cmd)
{
    LCD_SEL_CMD();
    spi_write_byte(cmd);
}

static void LCD_WriteData(const uint8_t data)
{
    LCD_SEL_DATA();
    spi_write_byte(data);
}

static void  ILI9341_INITIAL ()
{
    LCD_BKG_ON();
    //------------------------------------Reset Sequence-----------------------------------------//

    LCD_RST_SET();
    ets_delay_us(100000);                                                              

    LCD_RST_CLR();
    ets_delay_us(200000);                                                              

    LCD_RST_SET();
    ets_delay_us(200000);                                                             


#if (CONFIG_HW_LCD_TYPE == LCD_TYPE_ILI)
    //************* Start Initial Sequence **********//
    LCD_WriteCommand(0xCF);
    LCD_WriteData(0x00);
    LCD_WriteData(0x83);
    LCD_WriteData(0X30);

    LCD_WriteCommand(0xED);
    LCD_WriteData(0x64);
    LCD_WriteData(0x03);
    LCD_WriteData(0X12);
    LCD_WriteData(0X81);

    LCD_WriteCommand(0xE8);
    LCD_WriteData(0x85);
    LCD_WriteData(0x01); //i
    LCD_WriteData(0x79); //i

    LCD_WriteCommand(0xCB);
    LCD_WriteData(0x39);
    LCD_WriteData(0x2C);
    LCD_WriteData(0x00);
    LCD_WriteData(0x34);
    LCD_WriteData(0x02);

    LCD_WriteCommand(0xF7);
    LCD_WriteData(0x20);

    LCD_WriteCommand(0xEA);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);

    LCD_WriteCommand(0xC0);    //Power control
    LCD_WriteData(0x26); //i  //VRH[5:0]

    LCD_WriteCommand(0xC1);    //Power control
    LCD_WriteData(0x11);   //i //SAP[2:0];BT[3:0]

    LCD_WriteCommand(0xC5);    //VCM control
    LCD_WriteData(0x35); //i
    LCD_WriteData(0x3E); //i

    LCD_WriteCommand(0xC7);    //VCM control2
    LCD_WriteData(0xBE); //i   //»òÕß B1h

    LCD_WriteCommand(0x36);    // Memory Access Control
    LCD_WriteData(0x28); //i //was 0x48

    LCD_WriteCommand(0x3A);
    LCD_WriteData(0x55);

    LCD_WriteCommand(0xB1);
    LCD_WriteData(0x00);
    LCD_WriteData(0x1B); //18
    
    LCD_WriteCommand(0xF2);    // 3Gamma Function Disable
    LCD_WriteData(0x08);

    LCD_WriteCommand(0x26);    //Gamma curve selected
    LCD_WriteData(0x01);
        
    LCD_WriteCommand(0xE0);    //Set Gamma
    LCD_WriteData(0x1F);
    LCD_WriteData(0x1A);
    LCD_WriteData(0x18);
    LCD_WriteData(0x0A);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x06);
    LCD_WriteData(0x45);
    LCD_WriteData(0X87);
    LCD_WriteData(0x32);
    LCD_WriteData(0x0A);
    LCD_WriteData(0x07);
    LCD_WriteData(0x02);
    LCD_WriteData(0x07);
    LCD_WriteData(0x05);
    LCD_WriteData(0x00);
 
    LCD_WriteCommand(0XE1);    //Set Gamma
    LCD_WriteData(0x00);
    LCD_WriteData(0x25);
    LCD_WriteData(0x27);
    LCD_WriteData(0x05);
    LCD_WriteData(0x10);
    LCD_WriteData(0x09);
    LCD_WriteData(0x3A);
    LCD_WriteData(0x78);
    LCD_WriteData(0x4D);
    LCD_WriteData(0x05);
    LCD_WriteData(0x18);
    LCD_WriteData(0x0D);
    LCD_WriteData(0x38);
    LCD_WriteData(0x3A);
    LCD_WriteData(0x1F);

    LCD_WriteCommand(0x2A);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0xEF);

    LCD_WriteCommand(0x2B);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x01);
    LCD_WriteData(0x3f);
    LCD_WriteCommand(0x2C);
    
    LCD_WriteCommand(0xB7); 
    LCD_WriteData(0x07); 
    
    LCD_WriteCommand(0xB6);    // Display Function Control
    LCD_WriteData(0x0A); //8 82 27
    LCD_WriteData(0x82);
    LCD_WriteData(0x27);
    LCD_WriteData(0x00);

    //LCD_WriteCommand(0xF6); //not there
    //LCD_WriteData(0x01);
    //LCD_WriteData(0x30);

#endif
#if (CONFIG_HW_LCD_TYPE == LCD_TYPE_ST)

//212
//122
    LCD_WriteCommand(0x36);
    LCD_WriteData((1<<5)|(1<<6)); //MV 1, MX 1

    LCD_WriteCommand(0x3A);
    LCD_WriteData(0x55);

    LCD_WriteCommand(0xB2);
    LCD_WriteData(0x0c);
    LCD_WriteData(0x0c);
    LCD_WriteData(0x00);
    LCD_WriteData(0x33);
    LCD_WriteData(0x33);

    LCD_WriteCommand(0xB7);
    LCD_WriteData(0x35);

    LCD_WriteCommand(0xBB);
    LCD_WriteData(0x2B);

    LCD_WriteCommand(0xC0);
    LCD_WriteData(0x2C);

    LCD_WriteCommand(0xC2);
    LCD_WriteData(0x01);
    LCD_WriteData(0xFF);

    LCD_WriteCommand(0xC3);
    LCD_WriteData(0x11);

    LCD_WriteCommand(0xC4);
    LCD_WriteData(0x20);

    LCD_WriteCommand(0xC6);
    LCD_WriteData(0x0f);

    LCD_WriteCommand(0xD0);
    LCD_WriteData(0xA4);
    LCD_WriteData(0xA1);

    LCD_WriteCommand(0xE0);
    LCD_WriteData(0xD0);
    LCD_WriteData(0x00);
    LCD_WriteData(0x05);
    LCD_WriteData(0x0E);
    LCD_WriteData(0x15);
    LCD_WriteData(0x0D);
    LCD_WriteData(0x37);
    LCD_WriteData(0x43);
    LCD_WriteData(0x47);
    LCD_WriteData(0x09);
    LCD_WriteData(0x15);
    LCD_WriteData(0x12);
    LCD_WriteData(0x16);
    LCD_WriteData(0x19);

    LCD_WriteCommand(0xE1);
    LCD_WriteData(0xD0);
    LCD_WriteData(0x00);
    LCD_WriteData(0x05);
    LCD_WriteData(0x0D);
    LCD_WriteData(0x0C);
    LCD_WriteData(0x06);
    LCD_WriteData(0x2D);
    LCD_WriteData(0x44);
    LCD_WriteData(0x40);
    LCD_WriteData(0x0E);
    LCD_WriteData(0x1C);
    LCD_WriteData(0x18);
    LCD_WriteData(0x16);
    LCD_WriteData(0x19);

#endif


    LCD_WriteCommand(0x11);    //Exit Sleep
    ets_delay_us(100000);
    LCD_WriteCommand(0x29);    //Display on
    ets_delay_us(100000);


}
//.............LCD API END----------

static void ili_gpio_init()
{
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO21_U,2);   //DC PIN
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO18_U,2);   //RESET PIN
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,2);    //BKL PIN
    WRITE_PERI_REG(GPIO_ENABLE_W1TS_REG, BIT21|BIT18|BIT5);
}

static void spi_master_init()
{
    periph_module_enable(PERIPH_VSPI_MODULE);
    periph_module_enable(PERIPH_SPI_DMA_MODULE);

    ets_printf("lcd spi pin mux init ...\r\n");
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO19_U,2);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO23_U,2);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO22_U,2);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO25_U,2);
    WRITE_PERI_REG(GPIO_ENABLE_W1TS_REG, BIT19|BIT23|BIT22);

    ets_printf("lcd spi signal init\r\n");
    gpio_matrix_in(PIN_NUM_MISO, VSPIQ_IN_IDX,0);
    gpio_matrix_out(PIN_NUM_MOSI, VSPID_OUT_IDX,0,0);
    gpio_matrix_out(PIN_NUM_CLK, VSPICLK_OUT_IDX,0,0);
    gpio_matrix_out(PIN_NUM_CS, VSPICS0_OUT_IDX,0,0);
    ets_printf("Hspi config\r\n");

    CLEAR_PERI_REG_MASK(SPI_SLAVE_REG(SPI_NUM), SPI_TRANS_DONE << 5);
    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_CS_SETUP);
    CLEAR_PERI_REG_MASK(SPI_PIN_REG(SPI_NUM), SPI_CK_IDLE_EDGE);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM),  SPI_CK_OUT_EDGE);
    CLEAR_PERI_REG_MASK(SPI_CTRL_REG(SPI_NUM), SPI_WR_BIT_ORDER);
    CLEAR_PERI_REG_MASK(SPI_CTRL_REG(SPI_NUM), SPI_RD_BIT_ORDER);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_DOUTDIN);
    WRITE_PERI_REG(SPI_USER1_REG(SPI_NUM), 0);
    SET_PERI_REG_BITS(SPI_CTRL2_REG(SPI_NUM), SPI_MISO_DELAY_MODE, 0, SPI_MISO_DELAY_MODE_S);
    CLEAR_PERI_REG_MASK(SPI_SLAVE_REG(SPI_NUM), SPI_SLAVE_MODE);
    
    WRITE_PERI_REG(SPI_CLOCK_REG(SPI_NUM), (1 << SPI_CLKCNT_N_S) | (1 << SPI_CLKCNT_L_S));//40MHz
    //WRITE_PERI_REG(SPI_CLOCK_REG(SPI_NUM), SPI_CLK_EQU_SYSCLK); // 80Mhz
    
    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_CS_SETUP | SPI_CS_HOLD | SPI_USR_MOSI);
    SET_PERI_REG_MASK(SPI_CTRL2_REG(SPI_NUM), ((0x4 & SPI_MISO_DELAY_NUM) << SPI_MISO_DELAY_NUM_S));
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_COMMAND);
    SET_PERI_REG_BITS(SPI_USER2_REG(SPI_NUM), SPI_USR_COMMAND_BITLEN, 0, SPI_USR_COMMAND_BITLEN_S);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_ADDR);
    SET_PERI_REG_BITS(SPI_USER1_REG(SPI_NUM), SPI_USR_ADDR_BITLEN, 0, SPI_USR_ADDR_BITLEN_S);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_MISO);
    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_MOSI);
    char i;
    for (i = 0; i < 16; ++i) {
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)), 0);
    }
}

#define U16x2toU32(m,l) ((((uint32_t)(l>>8|(l&0xFF)<<8))<<16)|(m>>8|(m&0xFF)<<8))

extern uint16_t myPalette[];

char *menuText[10] = {"brightness46  0.","volume82      9."," .","hor stretch1  5.","vert stretch3 7."," .","  stretch can.", " cause graphic.", "   problems!.","*"};
bool arrow[9][9] = {{0,0,0,0,0,0,0,0,0},
					{0,0,0,0,1,0,0,0,0},
					{0,0,0,1,1,1,0,0,0},
					{0,0,1,1,1,1,1,0,0},
					{0,1,1,1,1,1,1,1,0},
					{0,0,0,1,1,1,0,0,0},
					{0,0,0,1,1,1,0,0,0},
					{0,0,0,1,1,1,0,0,0},
					{0,0,0,0,0,0,0,0,0}};
bool buttonA[9][9]={{0,0,1,1,1,1,0,0,0},
					{0,1,1,0,0,1,1,0,0},
					{1,1,0,1,1,0,1,1,0},
					{1,1,0,1,1,0,1,1,0},
					{1,1,0,0,0,0,1,1,0},
					{1,1,0,1,1,0,1,1,0},
					{0,1,0,1,1,0,1,0,0},
					{0,0,1,1,1,1,0,0,0},
					{0,0,0,0,0,0,0,0,0}};
bool buttonB[9][9]={{0,0,1,1,1,1,0,0,0},
					{0,1,0,0,0,1,1,0,0},
					{1,1,0,1,1,0,1,1,0},
					{1,1,0,0,0,1,1,1,0},
					{1,1,0,1,1,0,1,1,0},
					{1,1,0,1,1,0,1,1,0},
					{0,1,0,0,0,1,1,0,0},
					{0,0,1,1,1,1,0,0,0},
					{0,0,0,0,0,0,0,0,0}};
bool disabled[9][9]={{0,0,0,0,0,0,0,0,0},
					{0,0,1,1,1,1,0,0,0},
					{0,1,0,0,0,0,1,0,0},
					{1,0,0,0,0,1,0,1,0},
					{1,0,0,0,1,0,0,1,0},
					{1,0,0,1,0,0,0,1,0},
					{1,0,1,0,0,0,0,1,0},
					{0,1,0,0,0,0,1,0,0},
					{0,0,1,1,1,1,0,0,0}};
bool enabled[9][9]={{0,0,0,0,0,0,0,0,1},
					{0,0,0,0,0,0,0,1,1},
					{0,0,0,0,0,0,1,1,0},
					{0,0,0,0,0,1,1,0,0},
					{1,0,0,0,1,1,0,0,0},
					{1,1,0,1,1,0,0,0,0},
					{0,1,1,1,0,0,0,0,0},
					{0,0,1,0,0,0,0,0,0},
					{0,0,0,0,0,0,0,0,0}};
bool scale[9][9]=  {{0,0,0,0,0,0,0,0,0},
					{0,0,0,0,0,0,1,1,0},
					{0,0,0,0,0,0,1,1,0},
					{0,0,0,0,1,1,1,1,0},
					{0,0,0,0,1,1,1,1,0},
					{0,0,1,1,1,1,1,1,0},
					{0,0,1,1,1,1,1,1,0},
					{1,1,1,1,1,1,1,1,0},
					{1,1,1,1,1,1,1,1,0}};
bool lineEnd;
bool textEnd;

void ili9341_write_frame(const uint16_t xs, const uint16_t ys, const uint16_t width, const uint16_t height, const uint8_t * data[],
							bool xStr, bool yStr){
    int x, y;
    int i;
    uint16_t x1, y1;
    uint32_t xv, yv, dc;
    uint32_t temp[16];
    dc = (1 << PIN_NUM_DC);
    
    for (y=0; y<height; y++) {
        //start line
        x1 = xs+(width-1);
        y1 = ys+y+(height-1);
        xv = U16x2toU32(xs,x1);
        yv = U16x2toU32((ys+y),y1);
        
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1tc = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2A);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1ts = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 31, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), xv);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1tc = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2B);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1ts = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 31, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), yv);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1tc = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2C);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        
		if(getBright()==-1)LCD_BKG_OFF();
		
        x = 0;
        GPIO.out_w1ts = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 511, SPI_USR_MOSI_DBITLEN_S);
        while (x<width) {
			for (i=0; i<16; i++) {
                if(data == NULL){
                    temp[i] = 0;
                    x += 2;
                    continue;
                }
				int newX=x;
				int newy=y;
					//temp[i]==0x0F;
				if(xStr)newX=newX*0.8;
				if(yStr)newy=newy*0.94;
				if(newX>=32&&!xStr)newX=newX-32;
				x1 = myPalette[(unsigned char)(data[newy][newX])]; 
				x++;
				newX++;
                //if(xStr)newX=newX*0.8;
				//if(yStr)newy=newy*0.94;
				y1 = myPalette[(unsigned char)(data[newy][newX])]; 
				x++;
				newX++;
				if(!xStr && (x<=32||x>=288))x1=y1=0x00;
				//"ambilight"
				/*if(!xStr && x<=32)x1 = myPalette[(unsigned char)(data[newy][0])];
				if(!xStr && x<=32)y1 = myPalette[(unsigned char)(data[newy][0])];
				if(!xStr && x>=288)x1 = myPalette[(unsigned char)(data[newy][250])];
				if(!xStr && x>=288)y1 = myPalette[(unsigned char)(data[newy][250])];*/
				if(!yStr && y>=224)x1=y1=0x00;
                if(getShowMenu()){
					char actChar=' ';
					if(y==38)textEnd=0;
					if(x==40)lineEnd=0;
					int line =(y-38)/18;
					int charNo=(x-40)/16;
					if(x<32 || x>286 || y<34 || y>206);
					else if(x<40 || x>280 || y<38 || y>202)x1=y1=0x0F;
					else{
						if(!lineEnd && !textEnd){ 
							x1=y1=0x00;
							actChar=menuText[line][charNo];
							//printf("char %c, x = %d, y = %d{\n",actChar,x,y);
							//color c = [b](0to31)*1 + [g](0to31)*31 + [r] (0to31)*1024 +0x8000 --> x1=y1=c; !? 
							if(actChar=='2' && arrow[8-((y-38)%18)/2][((x-40)%16)/2])x1=y1=0xDDDD;
							else if(actChar=='4' && arrow[((x-40)%16)/2][((y-38)%18)/2])x1=y1=0xDDDD;
							else if(actChar=='6' && arrow[8-((x-40)%16)/2][8-((y-38)%18)/2])x1=y1=0xDDDD;
							else if(actChar=='8' && arrow[((y-38)%18)/2][((x-40)%16)/2])x1=y1=0xDDDD;
							else if(actChar=='1' && buttonA[((y-38)%18)/2][((x-40)%16)/2])x1=y1=0xDDDD;
							else if(actChar=='3' && buttonB[((y-38)%18)/2][((x-40)%16)/2])x1=y1=0xDDDD;
							else if(actChar=='5'){
								if(xStr && enabled[((y-38)%18)/2][((x-40)%16)/2])x1=y1=31*31+0x8000;
								else if(!xStr && disabled[((y-38)%18)/2][((x-40)%16)/2])x1=y1=31*1024+0x8000;
								else x1=y1=0x0F;;
							}
							else if(actChar=='7'){ 
								if(yStr && enabled[((y-38)%18)/2][((x-40)%16)/2])x1=y1=31*31+0x8000;
								else if(!yStr && disabled[((y-38)%18)/2][((x-40)%16)/2])x1=y1=31*1024+0x8000;
								else x1=y1=0x0F;
							}
							else if(actChar=='0'){
								if(scale[((y-38)%18)/2][((x-40)%16)/2] && (((x-40)%16)/2)< getBright()*2)x1=y1=0xFFFF;
								else if (scale[((y-38)%18)/2][((x-40)%16)/2] && (((x-40)%16)/2)>= getBright()*2)x1=y1=0xDDDD;
								else x1=y1=0x0F;
								setBrightness(getBright());
							}
							else if(actChar=='9'){
								if(getVolume()==0 && disabled[((y-38)%18)/2][((x-40)%16)/2])x1=y1=31*1024+0x8000;
								else x1=y1=0x0F;;
									
								if(getVolume()>0){
									if(scale[((y-38)%18)/2][((x-40)%16)/2] && (((x-40)%16)/2)< getVolume()*2)x1=y1=0xFFFF;
									else if (scale[((y-38)%18)/2][((x-40)%16)/2] && (((x-40)%16)/2)>= getVolume()*2)x1=y1=0xDDDD;
									else x1=y1=0x0F;
								}
							}
							else if((actChar<47 || actChar>57) && peGetPixel(actChar,(x-40)%16,(y-38)%18))x1=y1=0xFFFF;//0x55;
							else x1=y1=0x0F;
							if(actChar=='.'){lineEnd=1;x1=y1=0x0F;}
							if(actChar=='*'){textEnd=1;x1=y1=0x0F;}
						}
						else x1=y1=0x0F;
					}
				}
				temp[i] = U16x2toU32(x1,y1);
				if(getShutdown())setBrightness(getBright());
            }
            while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
            for (i=0; i<16; i++) {
                WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)), temp[i]);
            }
            SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        }
    }
    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
}

void ili9341_init()
{
    lineEnd=textEnd=0;
	spi_master_init();
    ili_gpio_init();
    ILI9341_INITIAL ();
	//LCD_BKG_ON();
	//initBCKL();
}





