#ifndef __STATUS_DISPLAY_H__
#define __STATUS_DISPLAY_H__

#include <Arduino.h>
#include "config.h"
#include "pins.h"

#include <Adafruit_GFX.h>    // Core graphics library
//#include <Adafruit_ST7735.h> // Hardware-specific library
#include <TFT_ST7735.h> // Hardware-specific library
#include <SPI.h>

#include <stdint.h>

/*
 * Global variables to hold status information for the pypilot motor controller display
 */
// extern Adafruit_ST7735 tft;
extern TFT_ST7735 tft;

extern unsigned long lastUpdateMillis;

extern uint16_t display_rudder_ADC;

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initializes the display, prints the layout
 */
void display_init(void);

/*
 * Blacks out everything that was spelled out before and redraws all status and debugging information
 */
void display_update(void);

/*
 * Actually 
 */
void display_PrintText(String textBuffer, int x, int y, int textSize, int color);

#ifdef __cplusplus
}
#endif

#endif
