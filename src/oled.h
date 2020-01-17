#pragma once
#include <Arduino.h>
#include <SeeedOLED.h>

//Initialize the display.
void setupOled();

void setupOled()
{
  SeeedOled.init();  //initialze SEEED OLED display
  SeeedOled.clearDisplay();          //clear the screen and set start position to top left corner
  SeeedOled.setNormalDisplay();      //Set display to normal mode (i.e non-inverse mode)
  SeeedOled.setPageMode();           //Set addressing mode to Page Mode
}