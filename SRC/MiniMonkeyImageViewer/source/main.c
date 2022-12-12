
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "LPC55S69_cm33_core0.h"
#include "fsl_debug_console.h"
#include "fsl_i2s.h"
#include "eGFX.h"
#include "Sprites_16BPP_RGB565.h"
#include "fsl_powerquad.h"
#include "math.h"

void SysTick_Handler()
{


}

#define  BTN1_IS_PRESSED  ((GPIO->PIN[BOARD_INITPINS_BTN1_PORT] & 1<<BOARD_INITPINS_BTN1_PIN)==0)
#define  BTN2_IS_PRESSED  ((GPIO->PIN[BOARD_INITPINS_BTN2_PORT] & 1<<BOARD_INITPINS_BTN2_PIN)==0)
#define  BTN3_IS_PRESSED  ((GPIO->PIN[BOARD_INITPINS_BTN3_PORT] & 1<<BOARD_INITPINS_BTN3_PIN)==0)



int main(void) {


	eGFX_ImagePlane *Next;

  	/* Init board hardware. */

	BOARD_InitBootPins();

    BOARD_BootClockPLL150M();
//    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    SystemCoreClockUpdate();

    SysTick_Config(SystemCoreClock/1000);

    CLOCK_AttachClk(kMAIN_CLK_to_FLEXCOMM3);


    eGFX_InitDriver();

    Next = (eGFX_ImagePlane *)&Sprite_16BPP_RGB565_monkey;

    while(1)
    {

    	if(BTN1_IS_PRESSED)
    	{
    		Next = (eGFX_ImagePlane *)&Sprite_16BPP_RGB565_monkey;
    	}

    	if(BTN2_IS_PRESSED)
       	{
        	Next = (eGFX_ImagePlane *)&Sprite_16BPP_RGB565_lilly;
       	}

    	if(BTN3_IS_PRESSED)
        {
        	Next = (eGFX_ImagePlane *)&Sprite_16BPP_RGB565_bg_fade;
        }


    	if(Next)
    	{
        	eGFX_Blit(&eGFX_BackBuffer[0],
        	          0,
        	          0,
        	          Next);

        	eGFX_Dump(&eGFX_BackBuffer[0]);

        	Next = 0;
    	}


    }

    return 0 ;
}

