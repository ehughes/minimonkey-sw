
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

#include "lvgl.h"
#include "lv_examples/src/lv_demo_widgets/lv_demo_widgets.h"


#define SAMPLE_RATE (16000)

/* 1 ms per tick. */
#ifndef LVGL_TICK_MS
#define LVGL_TICK_MS 1U
#endif

/* lv_task_handler is called every 5-tick. */
#ifndef LVGL_TASK_PERIOD_TICK
#define LVGL_TASK_PERIOD_TICK 5U
#endif

static volatile uint32_t s_tick        = 0U;

static volatile bool s_lvglTaskPending = false;

void SysTick_Handler()
{

    s_tick++;
    lv_tick_inc(LVGL_TICK_MS);

    if ((s_tick % LVGL_TASK_PERIOD_TICK) == 0U)
    {
        s_lvglTaskPending = true;
    }
}



lv_obj_t * gauge1;

void lv_ex_gauge_1(void)
{
    /*Describe the color for the needles*/
    static lv_color_t needle_colors[1];
    needle_colors[0] = LV_COLOR_ORANGE;

    /*Create a gauge*/
    gauge1 = lv_gauge_create(lv_scr_act(), NULL);

    lv_gauge_set_needle_count(gauge1, 1, needle_colors);

    lv_obj_set_size(gauge1, 230, 230);

    lv_obj_align(gauge1, NULL, LV_ALIGN_CENTER, 0, 0);


    lv_obj_set_style_local_image_recolor_opa(gauge1, LV_GAUGE_PART_NEEDLE, LV_STATE_DEFAULT, LV_OPA_COVER);

    lv_gauge_set_range(gauge1, 0, 100);

}

uint32_t Value = 0;



float EnvelopeOut;
float envIn;
float attack = 0.996879878;
float release = 0.9996879878;
float DC_Filter = 0;
float X_Prev = 0;
float X= 0;


int main(void) {


  	/* Init board hardware. */

	BOARD_InitBootPins();

    BOARD_BootClockPLL150M();
//    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    SystemCoreClockUpdate();

    SysTick_Config(SystemCoreClock/1000);

    CLOCK_AttachClk(kMAIN_CLK_to_FLEXCOMM3);

    RESET_PeripheralReset(kFC3_RST_SHIFT_RSTn);

    NVIC_ClearPendingIRQ(FLEXCOMM3_IRQn);
    CLOCK_AttachClk(kTRACE_DIV_to_TRACE);

    lv_port_pre_init();
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();

    lv_ex_gauge_1();



    /* Enable interrupts for I2S */
    EnableIRQ(FLEXCOMM3_IRQn);

    i2s_config_t s_RxConfig;

       /*
        *
        * The SPH0645LM4H microphone operates as an I2S slave.
			The master must provide the BCLK and WS signals. The Over
			Sampling Rate is fixed at 64 therefore the WS signal must be
			BCLK/64 and synchronized to the BCLK. Clock frequencies
			from 1.024Mhz to 4.096MHz are supported so sampling rates
			from 16KHz to 64KHz can be had by changing the clock
			frequency.
			The Data Format is I2S, 24-bit, 2’s compliment, MSB first. The
			data precision is 18 bits; unused bits are zeros.
        */
    I2S_RxGetDefaultConfig(&s_RxConfig);
    s_RxConfig.masterSlave = kI2S_MasterSlaveNormalMaster;
    s_RxConfig.position = 0;
    s_RxConfig.oneChannel = true;
    s_RxConfig.dataLength = 32;
	s_RxConfig.frameLength = 64;

    s_RxConfig.divider     = CLOCK_GetPll0OutFreq() / (SAMPLE_RATE*64) - 1;


    /*
     * Note!  I hacked the I2S driver a bit.   The NXP driver is really complicated to handle all the possible
     * use cases.   I am doing a very simple I2S per sample Rx.   I am overriding their default IRQ handler to keep things
     * simple.  In the future you might want to consider DMA, etc.  I just wanted a very simple IRQ handler everytime I get a new mic sample
     * Lots of room for optimization but this is the simplest case
     */

    I2S_RxInit(I2S3, &s_RxConfig);
    I2S_EnableInterrupts(I2S3, (uint32_t)kI2S_RxErrorFlag | (uint32_t)kI2S_RxLevelFlag);
    I2S_Enable(I2S3);


     attack = expf(-1.0/((float)SAMPLE_RATE * .050)); //50mS Attack
     release = expf(-1.0/((float)SAMPLE_RATE * .100)); //100mS Release

    while(1)
    {
        if(s_lvglTaskPending)
        {
        	  s_lvglTaskPending = false;

			  lv_task_handler();
        }

        lv_gauge_set_value(gauge1, 0, EnvelopeOut);

    }

    return 0 ;
}


void FLEXCOMM3_DriverIRQHandler(void)
{
	I2S_Type *base = I2S3;
	uint32_t intstat = base->FIFOINTSTAT;

	//This is a hacked up version of the IRQ handler from the SDK.  I simplified it to handle
	//sample by sample processing of the Microphone data.

	    if ((intstat & I2S_FIFOINTSTAT_RXERR_MASK) != 0UL)
	    {
	        /* Clear RX error interrupt flag */
	        base->FIFOSTAT = I2S_FIFOSTAT_RXERR(1U);
	    }

	    if ((intstat & I2S_FIFOINTSTAT_RXLVL_MASK) != 0UL)
	    {

	    	X = ((float)((int32_t)(base->FIFORD))) / (float)(1<<28);


	    	X = X * 300;

	    	DC_Filter = X - X_Prev + (.99*DC_Filter);

	    	X_Prev = X;

	    	envIn = fabs(DC_Filter);

    		 if(EnvelopeOut<envIn)
    		 {
    			 EnvelopeOut = envIn + attack * (EnvelopeOut - envIn);
    		 }
    		 else
    		 {
    			 EnvelopeOut = envIn + release * (EnvelopeOut - envIn);
    		 }


	        /* Clear RX level interrupt flag */
	        base->FIFOSTAT = I2S_FIFOSTAT_RXLVL(1U);
	    }

/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
