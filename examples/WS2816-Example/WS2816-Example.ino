/*
 * This project implements a driver for WS2816C-1313 neopixel LEDs.
 * The code is based on the Adafruit NeoPixel Library (https://github.com/adafruit/Adafruit_NeoPixel) released under the LGPL-3.0 license.
 *
 * Adafruit_NeoPixel is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * Adafruit_NeoPixel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with NeoPixel.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * This code was modified by Jan-Ole Giebel (2024) to implement the driver.
 *
 * This code will also be published under the LGPL-3.0 license
 *
 * This file is part of the WS2816_Driver by Jan-Ole Giebel
 * WS2816_Driver is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * WS2816_Driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with NeoPixel. If not, see
 * <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include "WS2816_Driver.h"

#define LED_PIN PA1  // Change this to your LED data pin
#define NUM_LEDS 16    // Change this to your number of LEDs
#define DELAY_MS 50   // Animation delay in milliseconds

WS2816_Driver strip(NUM_LEDS, LED_PIN);

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  
  Serial.println("WS2816C Test");
  
  strip.begin();
  strip.setBrightness(128);  // Set to 50% brightness
  strip.clear();
  strip.show();

  delay(50);
  // Test all LEDs sequentially
  for(int i = 0; i < NUM_LEDS; i++) {
    Serial.println("WS2816C Test");
    strip.clear();
    strip.setPixelColor(i, 65535, 0, 0); // Full red
    strip.show();
    Serial.print("Testing LED: ");
    Serial.println(i);
    delay(1000);
  }
  
  delay(500);
}

uint16_t animation_step = 0;

void loop() {
  // Rainbow cycle animation
  for(uint16_t i = 0; i < strip.numPixels(); i++) {
    uint16_t phase = (i + animation_step) % 6;
    
    switch(phase) {
      case 0:
        strip.setPixelColor(i, COLOR_RED);
        break;
      case 1:
        strip.setPixelColor(i, COLOR_YELLOW);
        break;
      case 2:
        strip.setPixelColor(i, COLOR_GREEN);
        break;
      case 3:
        strip.setPixelColor(i, COLOR_CYAN);
        break;
      case 4:
        strip.setPixelColor(i, COLOR_BLUE);
        break;
      case 5:
        strip.setPixelColor(i, COLOR_MAGENTA);
        break;
    }
  }
  
  strip.show();
  delay(DELAY_MS);
  animation_step++;
}
