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

#define DISPLAY_UPDATE_MS 500 // Delay in milliseconds between display updates

#define X_OFFSET_FOR_STATUS 2 // Just for beautification of the display, an offset
#define Y_OFFSET_FOR_STATUS 4 // Just for beautification of the display, an offset

/*
 * Global variables to hold status information for the pypilot motor controller display
 */
// extern Adafruit_ST7735 tft;
extern TFT_ST7735 tft;

extern unsigned long lastUpdateMillis;

extern uint16_t display_supply_voltage;
extern uint16_t display_sensor_ADC;
extern uint16_t display_sensor_scaled;
extern uint8_t display_endstop_prt;
extern uint8_t display_endstop_stb;
extern uint8_t display_endstop_type;
extern uint16_t display_motor_temperature;
extern uint16_t display_controller_temperature;
extern uint16_t display_motor_current;
extern uint16_t display_motor_command;
extern uint16_t display_motor_PWM;
extern uint16_t display_flags;
extern uint16_t display_faults;

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

void display_motor_engaged(void);
void display_motor_disengaged(void);

#ifdef __cplusplus
}
#endif

#endif
