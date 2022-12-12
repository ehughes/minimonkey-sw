
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
#include "GIF/AnimatedGIF.h"
#include "GIF/tank.h"
#include "GIF/homer.h"
#include "GIF/old_men.h"
#include "GIF/duck_hunt.h"
#include "GIF/eyeball.h"
#include "GIF/eye2.h"


volatile uint32_t OneSecTicker = 0;
volatile uint32_t FPS = 0;
volatile uint32_t Frames = 0;

volatile uint32_t millis_tick = 0;

uint32_t millis()
{
	return millis_tick;
}
uint32_t micros()
{
	return millis_tick*1000;
}

volatile uint32_t DelayTicker = 0;

void SysTick_Handler()
{
	millis_tick++;
	OneSecTicker++;
	DelayTicker++;

	if(OneSecTicker>=1000)
	{
		OneSecTicker =0;
		FPS = Frames;
		Frames = 0;
	}

}

void DelayMS(uint32_t C)
{

	DelayTicker = 0;

	while(DelayTicker<C)
	{

	}
}


#define  BTN1_IS_PRESSED  ((GPIO->PIN[BOARD_INITPINS_BTN1_PORT] & 1<<BOARD_INITPINS_BTN1_PIN)==0)
#define  BTN2_IS_PRESSED  ((GPIO->PIN[BOARD_INITPINS_BTN2_PORT] & 1<<BOARD_INITPINS_BTN2_PIN)==0)
#define  BTN3_IS_PRESSED  ((GPIO->PIN[BOARD_INITPINS_BTN3_PORT] & 1<<BOARD_INITPINS_BTN3_PIN)==0)



GIFIMAGE _gif;

extern void InitEyes();

extern void EyeLoop();

int main(void)
{

	BOARD_InitBootPins();

    BOARD_BootClockPLL150M();

    BOARD_InitDebugConsole();

    SystemCoreClockUpdate();

    SysTick_Config(SystemCoreClock/1000);

    CLOCK_AttachClk(kMAIN_CLK_to_FLEXCOMM3);

    eGFX_InitDriver();

    InitEyes();

    while(1)
    {

    	EyeLoop();

    }

    return 0 ;
}

