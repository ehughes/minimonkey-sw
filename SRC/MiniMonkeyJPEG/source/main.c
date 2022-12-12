
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "LPC55S69_cm33_core0.h"
#include "fsl_debug_console.h"
#include "fsl_i2s.h"
#include "eGFX.h"

#include "fsl_powerquad.h"
#include "math.h"
#include "JPEG/ImageData/lilly.h"
#include "JPEG/ImageData/gizmo.h"
#include "JPEG/ImageData/hiro.h"
#include "JPEG/ImageData/lucy.h"
#include "JPEG/JPEGDEC.h"


volatile uint32_t OneSecTicker = 0;
volatile uint32_t FPS = 0;
volatile uint32_t Frames = 0;

volatile int ImageSelect = 0;

void SysTick_Handler()
{
	OneSecTicker++;

	if(OneSecTicker>=3000)
	{
		OneSecTicker = 0;

		if(ImageSelect == 3)
			ImageSelect = 0;
		else
			ImageSelect++;
	}

}

#define  BTN1_IS_PRESSED  ((GPIO->PIN[BOARD_INITPINS_BTN1_PORT] & 1<<BOARD_INITPINS_BTN1_PIN)==0)
#define  BTN2_IS_PRESSED  ((GPIO->PIN[BOARD_INITPINS_BTN2_PORT] & 1<<BOARD_INITPINS_BTN2_PIN)==0)
#define  BTN3_IS_PRESSED  ((GPIO->PIN[BOARD_INITPINS_BTN3_PORT] & 1<<BOARD_INITPINS_BTN3_PIN)==0)


JPEGIMAGE _jpg;

void JPEGDraw(JPEGDRAW *pDraw)
{
	uint16_t *GFX = (uint16_t *)(&(eGFX_BackBuffer[0].Data[0]));

	if (pDraw->x < 240 && pDraw->y < 240)
	{
		int i = 0;

		for(int y=pDraw->y; y<pDraw->y+pDraw->iHeight; y++)
		{
			for(int x=pDraw->x; x<pDraw->x+pDraw->iWidth; x++)
			{

				//todo DMA
				GFX[y * eGFX_PHYSICAL_SCREEN_SIZE_X + x] = pDraw->pPixels[i++];

			}
		}
	}

}





int main(void)
{



  	/* Init board hardware. */

	BOARD_InitBootPins();

    BOARD_BootClockPLL150M();

    /* Init FSL debug console. */
    BOARD_InitDebugConsole();

    SystemCoreClockUpdate();

    SysTick_Config(SystemCoreClock/1000);

    eGFX_InitDriver();


    while (1)
    {
    	switch(ImageSelect)
    	{
    	default:
    	case 0:
    		JPEG_openRAM(&_jpg, (uint8_t *)Gizmo_jpg, Gizmo_jpg_size, JPEGDraw);
			break;

    	case 1:
    		JPEG_openRAM(&_jpg, (uint8_t *)hiro_jpg, hiro_jpg_size, JPEGDraw);
    			break;

    	case 2:
    		JPEG_openRAM(&_jpg, (uint8_t *)lilly_jpg, lilly_jpg_size, JPEGDraw);
    			break;


    	case 3:
    		JPEG_openRAM(&_jpg, (uint8_t *)Lucy_jpg, Lucy_jpg_size, JPEGDraw);
    			break;


    	}

       JPEG_decode(&_jpg, 0, 0, 0);

       JPEG_close(&_jpg);

   	    eGFX_Dump(&eGFX_BackBuffer[0]);
   };



    return 0 ;
}

