
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

volatile uint32_t OneSecTicker = 0;
volatile uint32_t FPS = 0;
volatile uint32_t Frames = 0;


void SysTick_Handler()
{
	OneSecTicker++;

	if(OneSecTicker>=1000)
	{
		OneSecTicker =0;
		FPS = Frames;
		Frames = 0;
	}

}

#define  BTN1_IS_PRESSED  ((GPIO->PIN[BOARD_INITPINS_BTN1_PORT] & 1<<BOARD_INITPINS_BTN1_PIN)==0)
#define  BTN2_IS_PRESSED  ((GPIO->PIN[BOARD_INITPINS_BTN2_PORT] & 1<<BOARD_INITPINS_BTN2_PIN)==0)
#define  BTN3_IS_PRESSED  ((GPIO->PIN[BOARD_INITPINS_BTN3_PORT] & 1<<BOARD_INITPINS_BTN3_PIN)==0)



GIFIMAGE _gif;

// Draw a line of image directly on the LCD
void GIFDraw(GIFDRAW* pDraw)
{
    uint8_t* s;
    uint16_t* d, * usPalette, usTemp[320];
    int x;
    int tx,ty;

    usPalette = pDraw->pPalette;
    //y = pDraw->iY + pDraw->y; // current line


    s = pDraw->pPixels;
    if (pDraw->ucDisposalMethod == 2) // restore to background color
    {
        for (x = 0; x < pDraw->iWidth; x++)
        {
            if (s[x] == pDraw->ucTransparent)
                s[x] = pDraw->ucBackground;
        }
        pDraw->ucHasTransparency = 0;
    }


    // Apply the new pixels to the main image
    if (pDraw->ucHasTransparency) // if transparency used
    {
        uint8_t* pEnd, c, ucTransparent = pDraw->ucTransparent;
        int x, iCount;
        pEnd = s + pDraw->iWidth;
        x = 0;
        iCount = 0; // count non-transparent pixels
        while (x < pDraw->iWidth)
        {
            c = ucTransparent - 1;
            d = usTemp;
            while (c != ucTransparent && s < pEnd)
            {
                c = *s++;
                if (c == ucTransparent) // done, stop
                {
                    s--; // back up to treat it like transparent
                }
                else // opaque
                {
                    *d++ = usPalette[c];
                    iCount++;
                }
            } // while looking for opaque pixels
            if (iCount) // any opaque pixels?
            {

                for (int i =  0; i < iCount; i++)
                {

                    /*
                     * ToDo optimize...
                     *
                     */
//                	eGFX_PutPixel(&eGFX_BackBuffer[0], pDraw->iX + x + i, pDraw->iY + y, usTemp[i]);


                	 tx = pDraw->iX + x + i;
                	 ty = pDraw->y + pDraw->iY;


                	  ((uint16_t *)(&(eGFX_BackBuffer[0].Data[0])))[(ty * 240) + tx] = usTemp[i];


                }

                x += iCount;
                iCount = 0;
            }
            // no, look for a run of transparent pixels
            c = ucTransparent;
            while (c == ucTransparent && s < pEnd)
            {
                c = *s++;
                if (c == ucTransparent)
                    iCount++;
                else
                    s--;
            }
            if (iCount)
            {
                x += iCount; // skip these
                iCount = 0;
            }
        }
    }
    else
    {
        s = pDraw->pPixels;
        // Translate the 8-bit pixels through the RGB565 palette (already byte reversed)
        for (x = 0; x < pDraw->iWidth; x++)
        {

            usTemp[x] = usPalette[*s++];

            /*
             * ToDo optimize...
             *
             */
          //eGFX_PutPixel(&eGFX_BackBuffer[0], pDraw->iX + x, pDraw->y + pDraw->iY, usTemp[x]);

            tx = pDraw->iX + x;
            ty = pDraw->y + pDraw->iY;

          ((uint16_t *)(&(eGFX_BackBuffer[0].Data[0])))[(ty * 240) + tx] = usTemp[x];

        }

    }
} /* GIFDraw() */




int main(void)
{

	BOARD_InitBootPins();

    BOARD_BootClockPLL150M();

    BOARD_InitDebugConsole();

    SystemCoreClockUpdate();

    SysTick_Config(SystemCoreClock/1000);

    CLOCK_AttachClk(kMAIN_CLK_to_FLEXCOMM3);

    eGFX_InitDriver();


    memset(&_gif, 0, sizeof(_gif));

    _gif.ucLittleEndian = (LITTLE_ENDIAN_PIXELS);
    _gif.pfnRead = readMem;
    _gif.pfnSeek = seekMem;
    _gif.pfnDraw = GIFDraw;
    _gif.pfnOpen = NULL;
    _gif.pfnClose = NULL;
    _gif.GIFFile.iSize = old_men_gif_size;
    _gif.GIFFile.pData = (uint8_t *)old_men_gif;

    GIFInit(&_gif);

    for(int i=0; i<(240*240/2); i++)
    {
         ((uint32_t *)(&(eGFX_BackBuffer[0].Data[0])))[i] = 0xFFFFFFFF;
    }
    eGFX_Dump(&eGFX_BackBuffer[0]);


    eGFX_Rect R;

    R.P1.X = 168;
    R.P1.Y = 218;
    R.P2.X = 239;
    R.P2.Y = 235;


    while(1)
    {

    	if(BTN1_IS_PRESSED)
        {
    	    memset(&_gif, 0, sizeof(_gif));

    	    _gif.ucLittleEndian = (LITTLE_ENDIAN_PIXELS);
    	    _gif.pfnRead = readMem;
    	    _gif.pfnSeek = seekMem;
    	    _gif.pfnDraw = GIFDraw;
    	    _gif.pfnOpen = NULL;
    	    _gif.pfnClose = NULL;
    	    _gif.GIFFile.iSize = old_men_gif_size;
    	    _gif.GIFFile.pData = (uint8_t *)old_men_gif;

    	    GIFInit(&_gif);
    	}

    	if(BTN2_IS_PRESSED)
        {
    	    memset(&_gif, 0, sizeof(_gif));

    	    _gif.ucLittleEndian = (LITTLE_ENDIAN_PIXELS);
    	    _gif.pfnRead = readMem;
    	    _gif.pfnSeek = seekMem;
    	    _gif.pfnDraw = GIFDraw;
    	    _gif.pfnOpen = NULL;
    	    _gif.pfnClose = NULL;
    	    _gif.GIFFile.iSize = homer_gif_size;
    	    _gif.GIFFile.pData = (uint8_t *)homer_gif;

    	    GIFInit(&_gif);
    	}

    	if(BTN3_IS_PRESSED)
        {


    	    for(int i=0; i<(240*240/2); i++)
    	    {
    	         ((uint32_t *)(&(eGFX_BackBuffer[0].Data[0])))[i] = 0xFFFFFFFF;
    	    }

        	eGFX_Blit(&eGFX_BackBuffer[0],
        	          240-67,
        	          2,
        	          &Sprite_16BPP_RGB565_TZeroBrew);

    	    memset(&_gif, 0, sizeof(_gif));

    	    _gif.ucLittleEndian = (LITTLE_ENDIAN_PIXELS);
    	    _gif.pfnRead = readMem;
    	    _gif.pfnSeek = seekMem;
    	    _gif.pfnDraw = GIFDraw;
    	    _gif.pfnOpen = NULL;
    	    _gif.pfnClose = NULL;
    	    _gif.GIFFile.iSize = tank_gif_size;
    	    _gif.GIFFile.pData = (uint8_t *)tank_gif;

    	    GIFInit(&_gif);
    	}

        if (_gif.GIFFile.iPos >= _gif.GIFFile.iSize - 1) // no more data exists
        {
            (*_gif.pfnSeek)(&_gif.GIFFile, 0); // seek to start
        }
        if (GIFParseInfo(&_gif, 0))
        {
             DecodeLZW(&_gif, 0);
        }




        eGFX_DrawSolidRectangle(&eGFX_BackBuffer[0],
        	&R,
        	0xFFFF);

        eGFX_printf_Colored(&eGFX_BackBuffer[0],170,220,&FONT_10_14_1BPP,0x0000, "FPS:%d",FPS);

      	eGFX_Dump(&eGFX_BackBuffer[0]);

      	Frames++;
    }

    return 0 ;
}

