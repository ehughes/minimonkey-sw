# Mini Monkey - Software 

This repo has test firmware for the MiniMonkey.     

### Build Tools :

- NXP MCUXpresso 10.2 IDE (newever)

- LPC55S69 SDK 2.8.2 Should be baked into the project.   If not you will need to install. A copy is located in the SDK folder.

### Debug tools

The Mini-Monkey has a standard connector SWD as well as tag connect pads.

I like to use J-Link + Ozone for debug.  MCUXpresso can also program/debug.   

Low cost debuggers:

- MCU Link ($10) 
- LPC-LINK-2 (@$25  Can be used as CMSIS-DAP or J-Link)
- J-Link EDU  (@$20 for non-commercial use)

Note:   I have noticed that the J-Link seems to fail on "fresh" silicon.   If I program once through a CMSIS-DAP connection in MCUXpresso,  J-link will work afterwards.  This might be fixed in newer j-link builds.


The Mini-Monkey can be booted into a special USB Boot mode (HID device) if BTN1 is pressed while the CPU is powered on/reset.   FlashMagic is a windows based tool that can be used to load via USB once the bootloader is invoked.


## Projects

### SRC/MiniMonkeyTimeSeries 

This is a simple project that displays microphone time series data on the display.

### SRC/MiniMonkeyImageView

Reads the switches to select an image to render on the screen.

### SRC/MiniMonkeyGIF

Plays an Animated GIF from internal flash memory using this library:

https://github.com/bitbank2/AnimatedGIF

### SRC/MiniMonkeyVU

An Audio VU meter based using the LVGL graphics library.

### SRC/MiniMonkeyJPEG

Decodes JPEG files stored in internal flash using this library:

https://github.com/bitbank2/JPEGDEC

### SRC/MM_C_IO

A project to define IO for the Rev C board.   Just a Skeleton project to use the pin mux tool.


## Notes

- The eGFX library used in the test projects is an experimental library I use for testing displays.   If you want a fully fledged library, look into LVGL, etc. eGFX is something I hacked together for simple samples, etc.
- All the projects use MCUXpresso IDE (Eclipse based).
- The projects were all bootstrapped from MCUXpresso SDK examples.



