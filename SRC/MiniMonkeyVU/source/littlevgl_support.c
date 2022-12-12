/*
 * Copyright 2019-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "littlevgl_support.h"
#include "lvgl.h"
#if defined(FSL_RTOS_FREE_RTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

#include "board.h"
#include "littlevgl_support.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include "LPC55S69_cm33_core0.h"
#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_spi.h"


/*
 * Note:
 *
 * This was adapted from the Buy Display example code
 *
 */


void ST7789V_Initial(void);
void Write_Cmd_Data(unsigned char);
void Write_Cmd(unsigned char);
void Write_Data(unsigned char DH,unsigned char DL);
void delayms(unsigned int tt);
void Write_Data_U16(unsigned int y);
void LCD_SetPos(unsigned int Xstart,unsigned int Ystart,unsigned int Xend,unsigned int Yend);
void ClearScreen(unsigned int bColor);
void InitScreenSPI();


#define DIRECTION 0    //0  vertical screen      2 is a horizontal screen


SDK_ALIGN(static uint8_t s_frameBuffer[2][LCD_VIRTUAL_BUF_SIZE * LCD_FB_BYTE_PER_PIXEL], 4);



void lv_port_pre_init(void)
{
}


#define ST7789V_CS_HIGH GPIO->SET[BOARD_INITPINS_SPI_SSEL_PORT] = 1<<BOARD_INITPINS_SPI_SSEL_PIN
#define ST7789V_CS_LOW	GPIO->CLR[BOARD_INITPINS_SPI_SSEL_PORT] = 1<<BOARD_INITPINS_SPI_SSEL_PIN


#define ST7789V_DC_HIGH GPIO->SET[BOARD_INITPINS_LCD_CMD_DATA_PORT] = 1<<BOARD_INITPINS_LCD_CMD_DATA_PIN
#define ST7789V_DC_LOW  GPIO->CLR[BOARD_INITPINS_LCD_CMD_DATA_PORT] = 1<<BOARD_INITPINS_LCD_CMD_DATA_PIN

#define ST7789V_SCL_HIGH GPIO->SET[BOARD_INITPINS_SPI_SCK_PORT] = 1<<BOARD_INITPINS_SPI_SCK_PIN
#define ST7789V_SCL_LOW  GPIO->CLR[BOARD_INITPINS_SPI_SCK_PORT] = 1<<BOARD_INITPINS_SPI_SCK_PIN

#define ST7789V_SDI_HIGH GPIO->SET[BOARD_INITPINS_SPI_MOSI_PORT] = 1<<BOARD_INITPINS_SPI_MOSI_PIN
#define ST7789V_SDI_LOW GPIO->CLR[BOARD_INITPINS_SPI_MOSI_PORT] = 1<<BOARD_INITPINS_SPI_MOSI_PIN

#define ST7789V_RESET_HIGH GPIO->SET[BOARD_INITPINS_LCD_RST_PORT] = 1<<BOARD_INITPINS_LCD_RST_PIN
#define ST7789V_RESET_LOW  GPIO->CLR[BOARD_INITPINS_LCD_RST_PORT] = 1<<BOARD_INITPINS_LCD_RST_PIN


#define LCD_BACKLIGHT_ON	GPIO->CLR[BOARD_INITPINS_LCD_BL_PORT] = 1<<BOARD_INITPINS_LCD_BL_PIN
#define LCD_BACKLIGHT_OFF	GPIO->SET[BOARD_INITPINS_LCD_BL_PORT] = 1<<BOARD_INITPINS_LCD_BL_PIN

static void DEMO_FlushDisplay(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    lv_coord_t x1 = area->x1;
    lv_coord_t y1 = area->y1;
    lv_coord_t x2 = area->x2;
    lv_coord_t y2 = area->y2;

    uint32_t send_size   = (x2 - x1 + 1) * (y2 - y1 + 1);


     LCD_SetPos(x1,y1,x2,y2);

      ST7789V_CS_LOW;
      ST7789V_DC_HIGH;

      //ToDo  Make this better!
      //
      //This is a minimal implementation to get the screen to work.  Need to reimplement with DMA
      //
      //Blast out (Tx only) the SPI Data.
      //
      //Optimizations ToDo
      //
      //Only need to write the upper 16 SPI config bits once
      //Move to DMA

   	for(int i=0;i<(send_size);i++)
   	{
   		/* wait for the response*/
   		while((SPI8->FIFOSTAT & 1<<5) == 0)
   		{
   		}

   		(SPI8->FIFOWR) = ((uint16_t *)color_p)[i] | SPI_FIFOWR_LEN(16-1) | (1<<SPI_FIFOWR_RXIGNORE_SHIFT);
   	}

      	ST7789V_CS_HIGH;



    lv_disp_flush_ready(disp_drv);
}

void lv_port_disp_init(void)
{

	 InitScreenSPI();

    static lv_disp_buf_t disp_buf;

    memset(s_frameBuffer, 0, sizeof(s_frameBuffer));
    lv_disp_buf_init(&disp_buf, s_frameBuffer[0], s_frameBuffer[1], LCD_VIRTUAL_BUF_SIZE);


    /*-----------------------------------
     * Register the display in LittlevGL
     *----------------------------------*/

    lv_disp_drv_t disp_drv;      /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv); /*Basic initialization*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = DEMO_FlushDisplay;

    /*Set a display buffer*/
    disp_drv.buffer = &disp_buf;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}




/* Will be called by the library to read the touchpad */
static bool DEMO_ReadTouch(lv_indev_drv_t *drv, lv_indev_data_t *data)
{

    /*Set the last pressed coordinates*/
    //data->point.x = LCD_WIDTH - touch_y;
    //data->point.y = touch_x;

    /*Return `false` because we are not buffering and no more data to read*/
    return false;
}


void lv_port_indev_init(void)
{
    lv_indev_drv_t indev_drv;

    /*------------------
     * Touchpad
     * -----------------*/

    ///*Initialize your touchpad */
    //DEMO_InitTouch();

    /*Register a touchpad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = DEMO_ReadTouch;
    lv_indev_drv_register(&indev_drv);
}




//Synchronously TX/RX a Byte
uint8_t SPI_TX_RX(uint8_t Data)
{
	SPI8->FIFOWR = Data | SPI_FIFOWR_LEN(8-1);
	/* wait for the response*/
	while((SPI8->FIFOSTAT & 1<<6) == 0)
	{
		/*   Wait for something to pop into the Queue*/
	}

	return SPI8->FIFORD;
}


//===============================================================
//write parameter
void  Write_Cmd_Data (unsigned char CMDP)
{

    ST7789V_CS_LOW;

    ST7789V_DC_HIGH;

  	SPI_TX_RX(CMDP);

   	ST7789V_CS_HIGH;

}


//=============================================================
//write command

void Write_Cmd(unsigned char CMD)
{


    ST7789V_CS_LOW;
   	ST7789V_DC_LOW;

   	SPI_TX_RX(CMD);

   	ST7789V_CS_HIGH;
}


//===================================================================
//write data byte

void Write_Data(unsigned char DH,unsigned char DL)
{
    ST7789V_CS_LOW;

    ST7789V_DC_HIGH;

	SPI_TX_RX(DH);

	SPI_TX_RX(DL);

   	ST7789V_CS_HIGH;

}



//============================================================
//delay
void delayms(unsigned int count)
{
    int i,j;
    for(i=0;i<count;i++)
       {
	     for(j=0;j<1000;j++);
       }
}

//=============================================================
//LCD Initial

void ST7789V_Initial(void)
{

	delayms(5);
	ST7789V_RESET_LOW;
	delayms(10);
	ST7789V_RESET_HIGH;
	delayms(120);

	 //************* Start Initial Sequence **********//
	Write_Cmd(0x36);

	Write_Cmd_Data(0x00);

	if(DIRECTION==0)Write_Cmd_Data(0x00);
	else if(DIRECTION==2)Write_Cmd_Data(0x70);

	Write_Cmd(0x3A);
	Write_Cmd_Data(0x05);

	Write_Cmd(0xB2);
	Write_Cmd_Data(0x0C);
	Write_Cmd_Data(0x0C);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x33);
	Write_Cmd_Data(0x33);

	Write_Cmd(0xB7);
	Write_Cmd_Data(0x35);

	Write_Cmd(0xBB);
	Write_Cmd_Data(0x19);

	Write_Cmd(0xC0);
	Write_Cmd_Data(0x2C);

	Write_Cmd(0xC2);
	Write_Cmd_Data(0x01);

	Write_Cmd(0xC3);
	Write_Cmd_Data(0x12);

	Write_Cmd(0xC4);
	Write_Cmd_Data(0x20);

	Write_Cmd(0xC6);
	Write_Cmd_Data(0x0F);

	Write_Cmd(0xD0);
	Write_Cmd_Data(0xA4);
	Write_Cmd_Data(0xA1);

	Write_Cmd(0xE0);
	Write_Cmd_Data(0xD0);
	Write_Cmd_Data(0x04);
	Write_Cmd_Data(0x0D);
	Write_Cmd_Data(0x11);
	Write_Cmd_Data(0x13);
	Write_Cmd_Data(0x2B);
	Write_Cmd_Data(0x3F);
	Write_Cmd_Data(0x54);
	Write_Cmd_Data(0x4C);
	Write_Cmd_Data(0x18);
	Write_Cmd_Data(0x0D);
	Write_Cmd_Data(0x0B);
	Write_Cmd_Data(0x1F);
	Write_Cmd_Data(0x23);

	Write_Cmd(0xE1);
	Write_Cmd_Data(0xD0);
	Write_Cmd_Data(0x04);
	Write_Cmd_Data(0x0C);
	Write_Cmd_Data(0x11);
	Write_Cmd_Data(0x13);
	Write_Cmd_Data(0x2C);
	Write_Cmd_Data(0x3F);
	Write_Cmd_Data(0x44);
	Write_Cmd_Data(0x51);
	Write_Cmd_Data(0x2F);
	Write_Cmd_Data(0x1F);
	Write_Cmd_Data(0x1F);
	Write_Cmd_Data(0x20);
	Write_Cmd_Data(0x23);

	Write_Cmd(0x21);

	Write_Cmd(0x11);
	delayms(120);

	Write_Cmd(0x29);


}


//===============================================================
 void LCD_SetPos(unsigned int Xstart,unsigned int Ystart,unsigned int Xend,unsigned int Yend)
{
	Write_Cmd(0x2a);
	Write_Cmd_Data(Xstart>>8);
	Write_Cmd_Data(Xstart);
 	Write_Cmd_Data(Xend>>8);
	Write_Cmd_Data(Xend);

	Write_Cmd(0x2b);
	Write_Cmd_Data(Ystart>>8);
	Write_Cmd_Data(Ystart);
	Write_Cmd_Data(Yend>>8);
	Write_Cmd_Data(Yend);

  	Write_Cmd(0x2c);//LCD_WriteCMD(GRAMWR);
}

 void InitScreenSPI()
 {

 	spi_master_config_t SPI_Config = {0};


 	CLOCK_AttachClk(kMAIN_CLK_to_HSLSPI);

 	/* reset FLEXCOMM for SPI */
 	RESET_PeripheralReset(kHSLSPI_RST_SHIFT_RSTn);

 	SPI_MasterGetDefaultConfig(&SPI_Config);

 	SPI_Config.sselNum = 0;
 	SPI_Config.enableMaster = true;
 	SPI_Config.phase = 0;
 	SPI_Config.polarity =  0;
 	SPI_Config.dataWidth = kSPI_Data8Bits;
 	SPI_Config.baudRate_Bps = 50000000;

 	SPI_MasterInit(SPI8,&SPI_Config, CLOCK_GetHsLspiClkFreq());

 	SPI8->FIFOCFG |= 3<<16; /*Flush the Tx & Rx buffers*/

 	SPI8->FIFOCFG |= 1; // Enable the fifo

     ST7789V_Initial();

   	LCD_BACKLIGHT_ON;
 }




