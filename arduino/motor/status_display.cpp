#include "status_display.h"

TFT_ST7735 tft = TFT_ST7735();       // Invoke custom library - insanely fast alternative! https://github.com/Bodmer/TFT_ST7735
// Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
unsigned long lastUpdateMillis = 0;
uint16_t display_rudder_ADC = 0;

void display_init(void)
{
    //tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab  
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(ST7735_BLACK);
}

void display_update(void)
{
  // Update display every 500 milliseconds
  if (millis() - lastUpdateMillis > 500)
  {
    // Display your stuff here
    
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
