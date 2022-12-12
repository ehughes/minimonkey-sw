//This example is configured for using 2 displays on SPI and SPI
//using a ST7789 240x240 display without a CS pin.
//If using a display with a CS pin you can change pin configuration
//in config.h
//#define USE_ASYNC_UPDATES
//#define ST77XX_ON_SPI_SPI1
#define USE_ST7789

//#define EYE_MASTER

// Define if you wish to see debug information on the ST7789 displays
//#define DEBUG_ST7789

// Define if you wish to debug memory usage.  Only works on T4.x
//#define DEBUG_MEMORY

#define BUTTON_ISR 7
//--------------------------------------------------------------------------
// Uncanny eyes for Adafruit 1.5" OLED (product #1431) or 1.44" TFT LCD
// (#2088).  Works on PJRC Teensy 3.x and on Adafruit M0 and M4 boards
// (Feather, Metro, etc.).  This code uses features specific to these
// boards and WILL NOT work on normal Arduino or other boards!
//
// SEE FILE "config.h" FOR MOST CONFIGURATION (graphics, pins, display type,
// etc).  Probably won't need to edit THIS file unless you're doing some
// extremely custom modifications.
//
// Adafruit invests time and resources providing this open source code,
// please support Adafruit and open-source hardware by purchasing products
// from Adafruit!
//
// Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.
// MIT license.  SPI FIFO insight from Paul Stoffregen's ILI9341_t3 library.
// Inspired by David Boccabella's (Marcwolf) hybrid servo/OLED eye concept.
//--------------------------------------------------------------------------

#include "stdint.h"
#include "stdbool.h"
#include "stdio.h"

#include "eGFX.h"
#include "pin_mux.h"
#include "LPC55S69_cm33_core0.h"
#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_spi.h"
#include "eGFX_Driver_ST7789_240_240_BD.h"
#include "fsl_debug_console.h"
#include "fsl_usart.h"
#include "Queue.h"


#define  BOARD_IS_MASTER_EYE  ((GPIO->PIN[BOARD_INITPINS_PIO0_27_PORT] & 1<<BOARD_INITPINS_PIO0_27_PIN))


extern uint32_t millis();

extern uint32_t micros();

extern void DelayMS(uint32_t C);

#define ENABLE_LIDS

uint32_t random(uint32_t  R)
{
	return (rand() % R);
}

uint32_t random_range(uint32_t  R1, uint32_t R2)
{
	return R1 + random(R2 -R1);
}

#define HIGH 1
#define LOW 0

uint32_t digitalRead(uint32_t R)
{
	return HIGH;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  if ((in_max - in_min) > (out_max - out_min)) {
    return (x - in_min) * (out_max - out_min+1) / (in_max - in_min+1) + out_min;
  }
  else
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}

typedef struct {        // Struct is defined before including config.h --
  //int8_t  select;       // pin numbers for each eye's screen select line
  int8_t  cs;            // Chip select pin.
  int8_t  dc;            // DC pin
  int8_t  mosi;         // mosi
  int8_t  sck;          // sck pin
  int8_t  rst;          // reset pin
  int8_t  wink;         // and wink button (or -1 if none) specified there,
  uint8_t rotation;     // also display rotation.
  uint8_t init_option;  // option for Init
} eyeInfo_t;


#define DISPLAY_SIZE 240

#include "config.h"     // ****** CONFIGURATION IS DONE IN HERE ******


// A simple state machine is used to control eye blinks/winks:
#define NOBLINK 0       // Not currently engaged in a blink
#define ENBLINK 1       // Eyelid is currently closing
#define DEBLINK 2       // Eyelid is currently opening
typedef struct {
  uint8_t  state;       // NOBLINK/ENBLINK/DEBLINK
  uint32_t duration;    // Duration of blink state (micros)
  uint32_t startTime;   // Time (micros) of last state change
} eyeBlink;

#define NUM_EYES (sizeof eyeInfo / sizeof eyeInfo[0]) // config.h pin list

struct {                // One-per-eye structure
 // displayType *display; // -> OLED/TFT object
  eyeBlink     blink;   // Current blink/wink state
} eye[NUM_EYES];


uint32_t startTime;  // For FPS indicator


// EYE-RENDERING FUNCTION --------------------------------------------------

//SPISettings settings(SPI_FREQ, MSBFIRST, SPI_MODE0);

void drawEye( // Renders one eye.  Inputs must be pre-clipped & valid.
  uint8_t  e,       // Eye array index; 0 or 1 for left/right
  uint16_t iScale,  // Scale factor for iris (0-1023)
  uint16_t  scleraX, // First pixel X offset into sclera image
  uint16_t  scleraY, // First pixel Y offset into sclera image
  uint8_t  uT,      // Upper eyelid threshold value
  uint8_t  lT) {    // Lower eyelid threshold value

  uint8_t  screenX, screenY;
  uint16_t scleraXsave;
  int16_t  irisX, irisY;
  uint16_t p, a;
  uint32_t d;
  uint16_t max_d = 0;
  uint16_t max_a = 0;
  uint16_t min_d = 0xff;
  uint16_t min_a = 0xff;
  
  uint32_t  irisThreshold = (DISPLAY_SIZE * (1023 - iScale) + 512) / 1024;
  uint32_t irisScale     = IRIS_MAP_HEIGHT * 65536 / irisThreshold;


  LCD_SetPos(0,0,239,239);

  ST7789V_CS_LOW;
  ST7789V_DC_HIGH;


  // Set up raw pixel dump to entire screen.  Although such writes can wrap
  // around automatically from end of rect back to beginning, the region is
  // reset on each frame here in case of an SPI glitch.
  // Now just issue raw 16-bit values for every pixel...

  scleraXsave = scleraX; // Save initial X value to reset on each line
  irisY       = scleraY - (SCLERA_HEIGHT - IRIS_HEIGHT) / 2;
  // Lets wait for any previous update screen to complete.
  for (screenY = 0; screenY < SCREEN_HEIGHT; screenY++, scleraY++, irisY++) {
    scleraX = scleraXsave;
    irisX   = scleraXsave - (SCLERA_WIDTH - IRIS_WIDTH) / 2;
    for (screenX = 0; screenX < SCREEN_WIDTH; screenX++, scleraX++, irisX++) {

#ifdef ENABLE_LIDS
      if ((lower[screenY][screenX] <= lT) ||
          (upper[screenY][screenX] <= uT))
      {             // Covered by eyelid
        p = 0;
      } else
#endif
    	  if ((irisY < 0) || (irisY >= IRIS_HEIGHT) ||
                 (irisX < 0) || (irisX >= IRIS_WIDTH)) { // In sclera
        p = sclera[scleraY][scleraX];
      } else {                                          // Maybe iris...
        p = polar[irisY][irisX];                        // Polar angle/dist
        d = p & 0x7F;                                   // Distance from edge (0-127)
        if (d < irisThreshold) {                        // Within scaled iris area
          d = d * irisScale / 65536;                    // d scaled to iris image height
          a = (IRIS_MAP_WIDTH * (p >> 7)) / 512;        // Angle (X)
          p = iris[d][a];                               // Pixel = iris
          if (d > max_d) max_d = d;
          if (a > max_a) max_a = a;
          if (d < min_d) min_d = d;
          if (a < min_a) min_a = a;
        } else {                                        // Not in iris
          p = sclera[scleraY][scleraX];                 // Pixel = sclera
        }
      }

    //  eye[e].display->drawPixel(screenX, screenY, p);

      /* wait for the response*/
      		while((SPI8->FIFOSTAT & 1<<5) == 0)
      		{
      		}

      		(SPI8->FIFOWR) = p | SPI_FIFOWR_LEN(16-1) | (1<<SPI_FIFOWR_RXIGNORE_SHIFT);

    } // end column
  } // end scanline

	while((SPI8->FIFOSTAT & 1<<5) == 0)
	{
	}

   	ST7789V_CS_HIGH;

}

// EYE ANIMATION -----------------------------------------------------------

const uint8_t ease[]  = { // Ease in/out curve for eye movements 3*t^2-2*t^3
  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,  3,   // T
  3,  3,  4,  4,  4,  5,  5,  6,  6,  7,  7,  8,  9,  9, 10, 10,   // h
  11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 22, 23,   // x
  24, 25, 26, 27, 27, 28, 29, 30, 31, 33, 34, 35, 36, 37, 38, 39,   // 2
  40, 41, 42, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 56, 57, 58,   // A
  60, 61, 62, 63, 65, 66, 67, 69, 70, 72, 73, 74, 76, 77, 78, 80,   // l
  81, 83, 84, 85, 87, 88, 90, 91, 93, 94, 96, 97, 98, 100, 101, 103, // e
  104, 106, 107, 109, 110, 112, 113, 115, 116, 118, 119, 121, 122, 124, 125, 127, // c
  128, 130, 131, 133, 134, 136, 137, 139, 140, 142, 143, 145, 146, 148, 149, 151, // J
  152, 154, 155, 157, 158, 159, 161, 162, 164, 165, 167, 168, 170, 171, 172, 174, // a
  175, 177, 178, 179, 181, 182, 183, 185, 186, 188, 189, 190, 192, 193, 194, 195, // c
  197, 198, 199, 201, 202, 203, 204, 205, 207, 208, 209, 210, 211, 213, 214, 215, // o
  216, 217, 218, 219, 220, 221, 222, 224, 225, 226, 227, 228, 228, 229, 230, 231, // b
  232, 233, 234, 235, 236, 237, 237, 238, 239, 240, 240, 241, 242, 243, 243, 244, // s
  245, 245, 246, 246, 247, 248, 248, 249, 249, 250, 250, 251, 251, 251, 252, 252, // o
  252, 253, 253, 253, 254, 254, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255
}; // n

#ifdef AUTOBLINK
uint32_t timeOfLastBlink = 0L, timeToNextBlink = 0L;
#endif


uint8_t out_buf[256];

void frame( // Process motion for a single frame of left or right eye
  uint16_t        iScale) {     // Iris scale (0-1023) passed in
  static uint8_t  eyeIndex = 0; // eye[] array counter
  int16_t         eyeX, eyeY;

  if (++eyeIndex >= NUM_EYES) eyeIndex = 0; // Cycle through eyes, 1 per call


  uint32_t        t = micros(); // Time at start of function


 // Autonomous X/Y eye motion
  // Periodically initiates motion to a new random point, random speed,
  // holds there for random period until next motion.

  static bool  eyeInMotion      = false;
  static int16_t  eyeOldX = 512, eyeOldY = 512, eyeNewX = 512, eyeNewY = 512;
  static uint32_t eyeMoveStartTime = 0L;
  static int32_t  eyeMoveDuration  = 0L;

  int32_t dt = t - eyeMoveStartTime;      // uS elapsed since last eye event
  if (eyeInMotion) {                      // Currently moving?
    if (dt >= eyeMoveDuration) {          // Time up?  Destination reached.
      eyeInMotion      = false;           // Stop moving
      eyeMoveDuration  = random(3000000); // 0-3 sec stop
      eyeMoveStartTime = t;               // Save initial time of stop
      eyeX = eyeOldX = eyeNewX;           // Save position
      eyeY = eyeOldY = eyeNewY;
    } else { // Move time's not yet fully elapsed -- interpolate position
      int16_t e = ease[255 * dt / eyeMoveDuration] + 1;   // Ease curve
      eyeX = eyeOldX + (((eyeNewX - eyeOldX) * e) / 256); // Interp X
      eyeY = eyeOldY + (((eyeNewY - eyeOldY) * e) / 256); // and Y
    }
  } else {                                // Eye stopped
    eyeX = eyeOldX;
    eyeY = eyeOldY;
    if (dt > eyeMoveDuration) {           // Time up?  Begin new move.
      int16_t  dx, dy;
      uint32_t d;
      do {                                // Pick new dest in circle
        eyeNewX = random(1024);
        eyeNewY = random(1024);
        dx      = (eyeNewX * 2) - 1023;
        dy      = (eyeNewY * 2) - 1023;
      } while ((d = (dx * dx + dy * dy)) > (1023 * 1023)); // Keep trying
      eyeMoveDuration  = random_range(72000, 144000); // ~1/14 - ~1/7 sec
      eyeMoveStartTime = t;               // Save initial time of move
      eyeInMotion      = true;            // Start move on next frame
      //Serial.printf("%d: Motion: %d %d (%d,%d)\n", eyeIndex, eyeMoveStartTime,
      //eyeMoveDuration, dx, dy);
    }
  }

  // Blinking

#ifdef AUTOBLINK
  // Similar to the autonomous eye movement above -- blink start times
  // and durations are random (within ranges).
  if ((t - timeOfLastBlink) >= timeToNextBlink) { // Start new blink?
    timeOfLastBlink = t;
    uint32_t blinkDuration = random_range(36000, 72000); // ~1/28 - ~1/14 sec
    // Set up durations for both eyes (if not already winking)
    for (uint8_t e = 0; e < NUM_EYES; e++) {
      if (eye[e].blink.state == NOBLINK) {
        eye[e].blink.state     = ENBLINK;
        eye[e].blink.startTime = t;
        eye[e].blink.duration  = blinkDuration;
      }
    }
    timeToNextBlink = blinkDuration * 3 + random(4000000);
  }
#endif

  if (eye[eyeIndex].blink.state) { // Eye currently blinking?
    // Check if current blink state time has elapsed
    if ((t - eye[eyeIndex].blink.startTime) >= eye[eyeIndex].blink.duration) {
      // Yes -- increment blink state, unless...
      if ((eye[eyeIndex].blink.state == ENBLINK) && ( // Enblinking and...
#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
            (digitalRead(BLINK_PIN) == LOW) ||           // blink or wink held...
#endif
            ((eyeInfo[eyeIndex].wink >= 0) &&
             digitalRead(eyeInfo[eyeIndex].wink) == LOW) )) {
        // Don't advance state yet -- eye is held closed instead
      } else { // No buttons, or other state...
        if (++eye[eyeIndex].blink.state > DEBLINK) { // Deblinking finished?
          eye[eyeIndex].blink.state = NOBLINK;      // No longer blinking
        } else { // Advancing from ENBLINK to DEBLINK mode
          eye[eyeIndex].blink.duration *= 2; // DEBLINK is 1/2 ENBLINK speed
          eye[eyeIndex].blink.startTime = t;
        }
      }
    }
  } else { // Not currently blinking...check buttons!
#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
    if (digitalRead(BLINK_PIN) == LOW) {
      // Manually-initiated blinks have random durations like auto-blink
      uint32_t blinkDuration = random(36000, 72000);
      for (uint8_t e = 0; e < NUM_EYES; e++) {
        if (eye[e].blink.state == NOBLINK) {
          eye[e].blink.state     = ENBLINK;
          eye[e].blink.startTime = t;
          eye[e].blink.duration  = blinkDuration;
        }
      }
    } else
#endif
      if ((eyeInfo[eyeIndex].wink >= 0) &&
          (digitalRead(eyeInfo[eyeIndex].wink) == LOW)) { // Wink!
        eye[eyeIndex].blink.state     = ENBLINK;
        eye[eyeIndex].blink.startTime = t;
        eye[eyeIndex].blink.duration  = random_range(45000, 90000);
      }
  }

  // Process motion, blinking and iris scale into renderable values

  // Scale eye X/Y positions (0-1023) to pixel units used by drawEye()
  eyeX = map(eyeX, 0, 1023, 0, SCLERA_WIDTH  - DISPLAY_SIZE);
  eyeY = map(eyeY, 0, 1023, 0, SCLERA_HEIGHT - DISPLAY_SIZE);
  if (eyeIndex == 1) eyeX = (SCLERA_WIDTH - DISPLAY_SIZE) - eyeX; // Mirrored display

  // Horizontal position is offset so that eyes are very slightly crossed
  // to appear fixated (converged) at a conversational distance.  Number
  // here was extracted from my posterior and not mathematically based.
  // I suppose one could get all clever with a range sensor, but for now...
  if (NUM_EYES > 1) eyeX += 4;
  if (eyeX > (SCLERA_WIDTH - DISPLAY_SIZE)) eyeX = (SCLERA_WIDTH - DISPLAY_SIZE);

  // Eyelids are rendered using a brightness threshold image.  This same
  // map can be used to simplify another problem: making the upper eyelid
  // track the pupil (eyes tend to open only as much as needed -- e.g. look
  // down and the upper eyelid drops).  Just sample a point in the upper
  // lid map slightly above the pupil to determine the rendering threshold.
  static uint16_t uThreshold = DISPLAY_SIZE;
  uint16_t        lThreshold, n;
#ifdef TRACKING
  int16_t sampleX = SCLERA_WIDTH  / 2 - (eyeX / 2), // Reduce X influence
          sampleY = SCLERA_HEIGHT / 2 - (eyeY + IRIS_HEIGHT / 4);
  // Eyelid is slightly asymmetrical, so two readings are taken, averaged
  if (sampleY < 0) n = 0;
  else            n = (upper[sampleY][sampleX] +
                         upper[sampleY][SCREEN_WIDTH - 1 - sampleX]) / 2;
  uThreshold = (uThreshold * 3 + n) / 4; // Filter/soften motion
  // Lower eyelid doesn't track the same way, but seems to be pulled upward
  // by tension from the upper lid.
  lThreshold = 254 - uThreshold;
#else // No tracking -- eyelids full open unless blink modifies them
  uThreshold = lThreshold = 0;
#endif

  // The upper/lower thresholds are then scaled relative to the current
  // blink position so that blinks work together with pupil tracking.
  if (eye[eyeIndex].blink.state) { // Eye currently blinking?
    uint32_t s = (t - eye[eyeIndex].blink.startTime);
    if (s >= eye[eyeIndex].blink.duration) s = 255;  // At or past blink end
    else s = 255 * s / eye[eyeIndex].blink.duration; // Mid-blink
    s          = (eye[eyeIndex].blink.state == DEBLINK) ? 1 + s : 256 - s;
    n          = (uThreshold * s + 254 * (257 - s)) / 256;
    lThreshold = (lThreshold * s + 254 * (257 - s)) / 256;
  } else {
    n          = uThreshold;
  }

  sprintf((char *)out_buf,"eye,%i,%i,%i,%i,%i,%i\r\n",eyeIndex, iScale, eyeX, eyeY, n, lThreshold);

  USART_WriteBlocking(USART0, out_buf, strlen((char *)out_buf));

  DelayMS(1);
  // Pass all the derived values to the eye-rendering function:
  drawEye(eyeIndex, iScale, eyeX, eyeY, n, lThreshold);
}


// Autonomous iris motion uses a fractal behavior to similate both the major
// reaction of the eye plus the continuous smaller adjustments that occur.

uint16_t oldIris = (IRIS_MIN + IRIS_MAX) / 2, newIris;

void split( // Subdivides motion path into two sub-paths w/randimization
  int16_t  startValue, // Iris scale value (IRIS_MIN to IRIS_MAX) at start
  int16_t  endValue,   // Iris scale value at end
  uint32_t startTime,  // micros() at start
  int32_t  duration,   // Start-to-end time, in microseconds
  int16_t  range) {    // Allowable scale value variance when subdividing

  if (range >= 8) {    // Limit subdvision count, because recursion
    range    /= 2;     // Split range & time in half for subdivision,
    duration /= 2;     // then pick random center point within range:
    int16_t  midValue = (startValue + endValue - range) / 2 + random(range);
    uint32_t midTime  = startTime + duration;
    split(startValue, midValue, startTime, duration, range); // First half
    split(midValue  , endValue, midTime  , duration, range); // Second half
  }
else
  {             // No more subdivisons, do iris motion...
    int32_t dt;        // Time (micros) since start of motion
    int16_t v;         // Interim value
    while ((dt = (micros() - startTime)) < duration) {
      v = startValue + (((endValue - startValue) * dt) / duration);
      if (v < IRIS_MIN)      v = IRIS_MIN; // Clip just in case
      else if (v > IRIS_MAX) v = IRIS_MAX;
      frame(v);        // Draw frame w/interim iris scale value
    }
  }
}

int32_t s_eyeIndex = 0;
int32_t s_iScale = 0;
int32_t s_eyeX = 0;
int32_t s_eyeY = 0;
int32_t s_n = 0;
int32_t s_lThreshold = 0;

uint8_t NextChar = 0;

uint32_t idx = 0;

char cmd[256];
char instr[256];

#define RX_RING_BUFFER_SIZE 256U

usart_handle_t g_uartHandle;
uint8_t g_tipString[] = "USART RX ring buffer example\r\nSend back received data\r\nEcho every 8 bytes\r\n";
uint8_t g_rxRingBuffer[RX_RING_BUFFER_SIZE] = {0}; /* RX ring buffer. */

uint8_t g_rxBuffer[RX_RING_BUFFER_SIZE] = {0}; /* Buffer for receive data to echo. */
uint8_t g_txBuffer[RX_RING_BUFFER_SIZE] = {0}; /* Buffer for send data to echo. */
volatile bool rxBufferEmpty          = true;
volatile bool txBufferFull           = false;
volatile bool txOnGoing              = false;
volatile bool rxOnGoing              = false;

usart_transfer_t receiveXfer;


size_t receivedBytes;

/* USART user callback */
void USART_UserCallback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData);

/* USART user callback */
void USART_UserCallback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_USART_TxIdle == status)
    {
        txBufferFull = false;
        txOnGoing    = false;
    }

    if (kStatus_USART_RxIdle == status)
    {
        rxBufferEmpty = false;
        rxOnGoing     = false;
    }
}


#define SHELL_SERIAL_QUEUE_SIZE	2048

ByteQueue ShellInputQueue;
ByteQueue ShellOutputQueue;

uint8_t ShellInputQueueStorage[SHELL_SERIAL_QUEUE_SIZE];
uint8_t ShellOutputQueueStorage[SHELL_SERIAL_QUEUE_SIZE];

void InitEyes()
{
   usart_config_t config;

   CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);

   USART_GetDefaultConfig(&config);
   config.baudRate_Bps = 115200;
   config.enableTx = true;
   config.enableRx = true;
   config.txWatermark = 0;


	InitByteQueue(&ShellInputQueue, SHELL_SERIAL_QUEUE_SIZE, ShellInputQueueStorage);
	InitByteQueue(&ShellOutputQueue, SHELL_SERIAL_QUEUE_SIZE, ShellOutputQueueStorage);


   USART_Init(USART0, &config, CLOCK_GetFlexCommClkFreq(0));
   USART_EnableInterrupts(USART0, kUSART_RxLevelInterruptEnable);
   EnableIRQ(FLEXCOMM0_IRQn);

}


void FLEXCOMM0_IRQHandler(void)
{
    uint8_t Data;

    if((kUSART_RxError) & USART_GetStatusFlags(USART0))
    {
    	USART_ClearStatusFlags(USART0,kUSART_RxError);
    }

	/* If new data arrived. */
	if ((kUSART_RxFifoNotEmptyFlag) & USART_GetStatusFlags(USART0))
	{
		Data = USART_ReadByte(USART0);
		ByteEnqueue(&ShellInputQueue,Data);

    }

    if(USART0->INTSTAT & 1<<3)
    {
    	//if(uxQueueMessagesWaitingFromISR(MySerialShell.ShellOutQueue))
    	//	{
		//		xQueueReceiveFromISR(MySerialShell.ShellOutQueue,&Data,NULL);
		//		USART_WriteByte(SHELL_UART,Data);
    //		}
    //	else
    	{


    		USART0->INTENCLR = 1<<3;

    	}
    }
}

void EyeLoop() {

if(BOARD_IS_MASTER_EYE)
{
  newIris = random_range(IRIS_MIN, IRIS_MAX);
  split(oldIris, newIris, micros(), 10000000L, IRIS_MAX - IRIS_MIN);
  oldIris = newIris;
}
else
{

   while(BytesInQueue(&ShellInputQueue))
   {
 		ByteDequeue(&ShellInputQueue,&NextChar);

	  if(NextChar == '\n')
	  {
		  cmd[idx] = 0;
		  idx=0;
		  sscanf(cmd,"eye,%i,%i,%i,%i,%i,%i",&s_eyeIndex, &s_iScale, &s_eyeX, &s_eyeY, &s_n, &s_lThreshold);
		  drawEye(s_eyeIndex,s_iScale,s_eyeX,s_eyeY,s_n,s_lThreshold);
	  }
	  else if (NextChar > 0x20)
	  {

		  cmd[idx] = NextChar;

		  if(idx<(sizeof(cmd)-1))
		  {
			  idx++;
		  }
	  }

  }
}




}

