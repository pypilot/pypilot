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

uint16_t display_supply_voltage_old = 1;
uint16_t display_sensor_ADC_old = 1;
uint16_t display_sensor_scaled_old = 1;
uint8_t display_endstop_prt_old = 1;
uint8_t display_endstop_stb_old = 1;
uint8_t display_endstop_type_old = 1;
uint16_t display_motor_temperature_old = 1;
uint16_t display_controller_temperature_old = 1;
uint16_t display_motor_current_old = 1;
uint16_t display_motor_command_old = 1;
uint16_t display_motor_PWM_old = 1;
uint16_t display_flags_old = 1;

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
  
  display_motor_disengaged();
}

void display_update(void)
{
  // Update display every 500 milliseconds
  if (millis() - lastUpdateMillis > 500)
  {
    // Update your display stuff here
    if (display_supply_voltage_old != display_supply_voltage){
      display_PrintText(String(display_supply_voltage_old / 100.0f),  X_OFFSET_FOR_STATUS + 121, Y_OFFSET_FOR_STATUS + 1 * 8, 0, ST7735_BLACK);
      display_PrintText(String(display_supply_voltage / 100.0f),      X_OFFSET_FOR_STATUS + 121, Y_OFFSET_FOR_STATUS + 1 * 8, 0, ST7735_YELLOW);
      display_supply_voltage_old = display_supply_voltage;
    }
    if (display_sensor_ADC_old != display_sensor_ADC){
      display_PrintText(String(display_sensor_ADC_old),               X_OFFSET_FOR_STATUS + 127, Y_OFFSET_FOR_STATUS + 2 * 8, 0, ST7735_BLACK);
      display_PrintText(String(display_sensor_ADC),                   X_OFFSET_FOR_STATUS + 127, Y_OFFSET_FOR_STATUS + 2 * 8, 0, ST7735_YELLOW);
      display_sensor_ADC_old = display_sensor_ADC;    
    }
    if (display_sensor_scaled_old != display_sensor_scaled){
      display_PrintText(String(display_sensor_scaled_old),            X_OFFSET_FOR_STATUS + 127, Y_OFFSET_FOR_STATUS + 3 * 8, 0, ST7735_BLACK);
      display_PrintText(String(display_sensor_scaled),                X_OFFSET_FOR_STATUS + 127, Y_OFFSET_FOR_STATUS + 3 * 8, 0, ST7735_YELLOW);
      display_sensor_scaled_old = display_sensor_scaled;
    }
    if (display_endstop_prt_old != display_endstop_prt){
      display_PrintText("PORT | STARBRD",                             X_OFFSET_FOR_STATUS + 72, Y_OFFSET_FOR_STATUS + 4 * 8, 0, ST7735_BLACK);
      display_PrintText("PORT | STARBRD",                             X_OFFSET_FOR_STATUS + 72, Y_OFFSET_FOR_STATUS + 4 * 8, 0, ST7735_YELLOW);
      display_endstop_prt_old = display_endstop_prt;
    }
    if (display_endstop_stb_old != display_endstop_stb){
      display_PrintText("PORT | STARBRD",                             X_OFFSET_FOR_STATUS + 72, Y_OFFSET_FOR_STATUS + 4 * 8, 0, ST7735_BLACK);
      display_PrintText("PORT | STARBRD",                             X_OFFSET_FOR_STATUS + 72, Y_OFFSET_FOR_STATUS + 4 * 8, 0, ST7735_YELLOW);
      display_endstop_stb_old = display_endstop_stb;
    }
    if (display_endstop_type_old != display_endstop_type){
      display_PrintText("SOFTWARE",                                   X_OFFSET_FOR_STATUS + 108, Y_OFFSET_FOR_STATUS + 5 * 8, 0, ST7735_BLACK);
      display_PrintText("SOFTWARE",                                   X_OFFSET_FOR_STATUS + 108, Y_OFFSET_FOR_STATUS + 5 * 8, 0, ST7735_YELLOW);
      display_endstop_type_old = display_endstop_type;
    }
    if (display_motor_temperature_old != display_motor_temperature){
      display_PrintText(String(display_motor_temperature_old / 100.0f), X_OFFSET_FOR_STATUS + 96, Y_OFFSET_FOR_STATUS + 6 * 8, 0, ST7735_BLACK);
      display_PrintText(String(display_motor_temperature / 100.0f),     X_OFFSET_FOR_STATUS + 96, Y_OFFSET_FOR_STATUS + 6 * 8, 0, ST7735_YELLOW);
      display_motor_temperature_old = display_motor_temperature;
    }
    if (display_controller_temperature_old != display_controller_temperature){
      display_PrintText(String(display_controller_temperature_old / 100.0f), X_OFFSET_FOR_STATUS + 96, Y_OFFSET_FOR_STATUS + 7 * 8, 0, ST7735_BLACK);
      display_PrintText(String(display_controller_temperature / 100.0f),     X_OFFSET_FOR_STATUS + 96, Y_OFFSET_FOR_STATUS + 7 * 8, 0, ST7735_YELLOW);
      display_controller_temperature_old = display_controller_temperature;    
    }
    if (display_motor_current_old != display_motor_current){
      display_PrintText(String(display_motor_current_old),            X_OFFSET_FOR_STATUS + 102, Y_OFFSET_FOR_STATUS + 8 * 8, 0, ST7735_BLACK);
      display_PrintText(String(display_motor_current),                X_OFFSET_FOR_STATUS + 102, Y_OFFSET_FOR_STATUS + 8 * 8, 0, ST7735_YELLOW);
      display_motor_current_old = display_motor_current;
    }
    if (display_motor_command_old != display_motor_command){
      display_PrintText(String(display_motor_command_old),            X_OFFSET_FOR_STATUS + 132, Y_OFFSET_FOR_STATUS + 9 * 8, 0, ST7735_BLACK);
      display_PrintText(String(display_motor_command),                X_OFFSET_FOR_STATUS + 132, Y_OFFSET_FOR_STATUS + 9 * 8, 0, ST7735_YELLOW);
      display_motor_command_old = display_motor_command;    
    }
    if (display_motor_PWM_old != display_motor_PWM){
      display_PrintText(String(display_motor_PWM_old),                X_OFFSET_FOR_STATUS + 132, Y_OFFSET_FOR_STATUS + 10 * 8, 0, ST7735_BLACK);
      display_PrintText(String(display_motor_PWM),                    X_OFFSET_FOR_STATUS + 132, Y_OFFSET_FOR_STATUS + 10 * 8, 0, ST7735_YELLOW);
      display_motor_PWM_old = display_motor_PWM;
    }
    if (display_flags_old != display_flags){
      display_PrintText("0b00000000 00000000",                        X_OFFSET_FOR_STATUS + 42, Y_OFFSET_FOR_STATUS + 12 * 8, 0, ST7735_BLACK);
      display_PrintText("0b00000000 00000000",                        X_OFFSET_FOR_STATUS + 42, Y_OFFSET_FOR_STATUS + 12 * 8, 0, ST7735_YELLOW);
      display_flags_old = display_flags;
    }
    
    
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
