
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

#define MIC_SAMPLES_TO_CAPTURE	256
#define RAW_BUFFER_SIZE			(MIC_SAMPLES_TO_CAPTURE)

volatile int32_t RawBuffers[2][RAW_BUFFER_SIZE];
volatile bool RequestBuffer = false;
volatile int32_t *DataBuffer;
volatile int32_t BufferIndex = 0;
volatile int32_t CurrentBufferFilling = 0;

int32_t MIC_Data[RAW_BUFFER_SIZE];

int main(void) {

	int32_t DataPoint;
	int32_t LastDataPoint;


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

    s_RxConfig.divider     = CLOCK_GetPll0OutFreq() / (16000*64) - 1;


    /*
     * Note!  I hacked the I2S driver a bit.   The NXP driver is really complicated to handle all the possible
     * use cases.   I am doing a very simple I2S per sample Rx.   I am overriding their default IRQ handler to keep things
     * simple.  In the future you might want to consider DMA, etc.  I just wanted a very simple IRQ handler everytime I get a new mic sample
     * Lots of room for optimization but this is the simplest case
     */

    I2S_RxInit(I2S3, &s_RxConfig);
    I2S_EnableInterrupts(I2S3, (uint32_t)kI2S_RxErrorFlag | (uint32_t)kI2S_RxLevelFlag);
    I2S_Enable(I2S3);

    eGFX_InitDriver();

    while(1)
    {

    	//request a new buffer from the IRQ.
    	RequestBuffer = true;

    	//While we are waiting for the data to come in,  do some GFX work.

    	//Clear the backbuffer to white.
    	//
    	//ToDo :   DMA this or use a dirty rectangle approach
    	//

        for(int i=0; i<(240*240/2); i++)
        {
             ((uint32_t *)(&(eGFX_BackBuffer[0].Data[0])))[i] = 0xFFFFFFFF;
        }

        //Draw the Monkey
    	eGFX_Blit(&eGFX_BackBuffer[0],
    	          0,
    	          0,
    	          &Sprite_16BPP_RGB565_monkey_small2);

    	//Wait for the rest of the data to come in.
    	while(RequestBuffer == true)
    	{

    	}


    	for(int i=0;i<240;i++)
    	{
    		LastDataPoint = DataPoint;


    		DataPoint = (DataBuffer[i] ) >> 5;

    		//Draw some lines..
    		if(i)
    		{

    			eGFX_DrawLine(&eGFX_BackBuffer[0],i-1,LastDataPoint+120,i,DataPoint+120,0x0000);
    			eGFX_DrawLine(&eGFX_BackBuffer[0],i-1,LastDataPoint+119,i,DataPoint+119,0x0000);
    			eGFX_DrawLine(&eGFX_BackBuffer[0],i-1,LastDataPoint+121,i,DataPoint+121,0x0000);

    		}

    	}

    	//Dump to the screen.

    	eGFX_Dump(&eGFX_BackBuffer[0]);
    }

    return 0 ;
}

int32_t X;
int32_t X_Prev;
int32_t Sample;
int32_t DC_Filter;

void FLEXCOMM3_DriverIRQHandler(void)
{
	I2S_Type *base = I2S3;
	uint32_t intstat = base->FIFOINTSTAT;
	GPIO->NOT[BOARD_INITPINS_LCD_BL_PORT] = 1<<BOARD_INITPINS_LCD_BL_PIN;
	//This is a hacked up version of the IRQ handler from the SDK.  I simplified it to handle
	//sample by sample processing of the Microphone data.

	    if ((intstat & I2S_FIFOINTSTAT_RXERR_MASK) != 0UL)
	    {
	        /* Clear RX error interrupt flag */
	        base->FIFOSTAT = I2S_FIFOSTAT_RXERR(1U);
	    }

	    if ((intstat & I2S_FIFOINTSTAT_RXLVL_MASK) != 0UL)
	    {
	    	Sample = ((int32_t)base->FIFORD)>>16;

	    	X = Sample;

	    	DC_Filter = X - X_Prev + ((31000*DC_Filter)>>15);

	    	X_Prev = X;

	    		RawBuffers[CurrentBufferFilling][BufferIndex] = (int32_t)DC_Filter;

	    		BufferIndex++;
	    		if(BufferIndex >= (RAW_BUFFER_SIZE))
	    		{
	    			if(RequestBuffer == true)
	    			{
	    				DataBuffer = &RawBuffers[CurrentBufferFilling][0];
	    				RequestBuffer = false;
	    			}

					BufferIndex = 0;
	    			CurrentBufferFilling++;
	    			CurrentBufferFilling &= 0x01;
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
