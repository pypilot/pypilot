#include "status_display.h"

TFT_ST7735 tft = TFT_ST7735();       // Invoke custom library - insanely fast alternative! https://github.com/Bodmer/TFT_ST7735
// Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
unsigned long lastUpdateMillis = 0;
uint16_t display_rudder_ADC = 0;

uint16_t display_supply_voltage = 0;
uint16_t display_sensor_ADC = 0;
uint16_t display_sensor_scaled = 0;
uint8_t display_endstop_prt = 0;
uint8_t display_endstop_stb = 0;
uint8_t display_endstop_type = 0;
uint16_t display_motor_temperature = 0;
uint16_t display_controller_temperature = 0;
uint16_t display_motor_current = 0;
uint16_t display_motor_command = 0;
uint16_t display_motor_PWM = 0;
uint16_t display_flags = 0;
uint16_t display_faults = 0;

void display_init(void)
{
    //tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab  
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(ST7735_BLACK);

  tft.fillRect(0, 0, 159, 8, ST7735_BLUE);
  display_PrintText("   Pypilot Motor Status   ", 0, 0, 0, ST7735_WHITE);
  
  uint8_t row = 1;
  display_PrintText("Supply Voltage:          V", X_OFFSET_FOR_STATUS, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_WHITE);
  display_PrintText("Rudder Sensor ADC:",         X_OFFSET_FOR_STATUS, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_WHITE);
  display_PrintText("Rudder Sens. Scaled:",       X_OFFSET_FOR_STATUS, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_WHITE);
  display_PrintText("EndStop Sw:",                X_OFFSET_FOR_STATUS, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_WHITE);
  display_PrintText("Endstop SW Type:",           X_OFFSET_FOR_STATUS, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_WHITE);
  display_PrintText("Motor Temp:          deg C", X_OFFSET_FOR_STATUS, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_WHITE);
  display_PrintText("Controller Temp:     deg C", X_OFFSET_FOR_STATUS, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_WHITE);
  display_PrintText("Motor Current:        Amps", X_OFFSET_FOR_STATUS, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_WHITE);
  display_PrintText("Motor Command:",             X_OFFSET_FOR_STATUS, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_WHITE);
  display_PrintText("Motor PWM:",                 X_OFFSET_FOR_STATUS, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_WHITE);
  display_PrintText("--------------------------", X_OFFSET_FOR_STATUS, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_WHITE);
  display_PrintText("Flags:", X_OFFSET_FOR_STATUS, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_WHITE);
  display_PrintText("Faults:", X_OFFSET_FOR_STATUS, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_WHITE);
  
  display_motor_disengaged();
}

void display_update(void)
{
  // Update display every 500 milliseconds
  if (millis() - lastUpdateMillis > 500)
  {
    // Update your display stuff here
    uint8_t row = 1;
    display_PrintText("13.50",                 X_OFFSET_FOR_STATUS + 121, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_YELLOW);
    display_PrintText("11526",                X_OFFSET_FOR_STATUS + 127, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_YELLOW);
    display_PrintText("32768",                X_OFFSET_FOR_STATUS + 127, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_YELLOW);
    display_PrintText("PORT | STARBRD",       X_OFFSET_FOR_STATUS + 72, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_YELLOW);
    display_PrintText("SOFTWARE",             X_OFFSET_FOR_STATUS + 108, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_YELLOW);
    display_PrintText("115",                  X_OFFSET_FOR_STATUS + 102, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_YELLOW);
    display_PrintText("112",                  X_OFFSET_FOR_STATUS + 102, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_YELLOW);
    display_PrintText("1.5",                  X_OFFSET_FOR_STATUS + 102, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_YELLOW);
    display_PrintText("2000",                 X_OFFSET_FOR_STATUS + 132, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_YELLOW);
    display_PrintText("1450",                 X_OFFSET_FOR_STATUS + 132, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_YELLOW);
    row++;
    display_PrintText("0b00000000 00000000",  X_OFFSET_FOR_STATUS + 42, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_YELLOW);
    display_PrintText("0b00000000 00000000",  X_OFFSET_FOR_STATUS + 42, Y_OFFSET_FOR_STATUS + row++ * 8, 0, ST7735_YELLOW);
    
    lastUpdateMillis = millis(); // Update milliseconds for next loop
  }
}

void display_PrintText(String textBuffer, int x, int y, int textSize, int color)
{
  if (textSize == 0) textSize = 1;
  tft.setTextColor(color);
  tft.setTextSize(textSize);
  tft.setCursor(x, y);
  tft.print(textBuffer.c_str());
}

void display_motor_engaged(void)
{
  tft.fillRect(0, 119, 159, 127, ST7735_RED);
  display_PrintText("ENGAGED", 60, 120, 0, ST7735_WHITE);
}

void display_motor_disengaged(void)
{
  tft.fillRect(0, 119, 159, 127, ST7735_GREEN);
  display_PrintText("DISENGAGED", 50, 120, 0, ST7735_BLACK);
}
