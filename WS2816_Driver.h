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

#pragma once

#include <Arduino.h>

// Some pre-defined 16-bit colors
#define COLOR_RED     (uint16_t)65535, (uint16_t)0,     (uint16_t)0
#define COLOR_GREEN   (uint16_t)0,     (uint16_t)65535, (uint16_t)0
#define COLOR_BLUE    (uint16_t)0,     (uint16_t)0,     (uint16_t)65535
#define COLOR_YELLOW  (uint16_t)65535, (uint16_t)65535, (uint16_t)0
#define COLOR_MAGENTA (uint16_t)65535, (uint16_t)0,     (uint16_t)65535
#define COLOR_CYAN    (uint16_t)0,     (uint16_t)65535, (uint16_t)65535
#define COLOR_WHITE   (uint16_t)65535, (uint16_t)65535, (uint16_t)65535
#define COLOR_OFF     (uint16_t)0,     (uint16_t)0,     (uint16_t)0

// WS2816C NeoPixel Class definition
class WS2816_Driver {
public:
  WS2816_Driver(uint16_t n, int16_t pin);
  ~WS2816_Driver();

  void begin(void);
  void show(void);
  void setPin(int16_t p);
  void setPixelColor(uint16_t n, uint16_t r, uint16_t g, uint16_t b);
  void setBrightness(uint8_t brightness);
  void clear(void);
  inline bool canShow(void) { return (micros() - endTime) >= 50L; }
  uint32_t getPixelColor(uint16_t n) const;
  uint16_t numPixels(void) const { return numLEDs; }
  
private:
  bool begun;         // true if begin() previously called
  uint16_t numLEDs;   // Number of RGB LEDs in strip
  uint32_t numBytes;  // Size of 'pixels' buffer below (6 bytes per pixel)
  int16_t pin;        // Output pin number (-1 if not yet set)
  uint8_t brightness;
  uint16_t *pixels;   // Holds LED color values (6 bytes per pixel)
  uint32_t endTime;   // Latch timing reference
  GPIO_TypeDef *gpioPort;
  uint32_t gpioPin;
};
