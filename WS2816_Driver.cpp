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

#include "WS2816_Driver.h"

#if defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_ARDUINO_CORE_STM32)
WS2816_Driver::WS2816_Driver(uint16_t n, int16_t p) :
  begun(false), numLEDs(n), numBytes(n * 6), pin(p), brightness(255),
  pixels(NULL), endTime(0) {
  if((pixels = (uint16_t *)malloc(numBytes))) {
    memset(pixels, 0, numBytes);
  }
}

WS2816_Driver::~WS2816_Driver() {
  free(pixels);
  if(pin >= 0) pinMode(pin, INPUT);
}

void WS2816_Driver::begin(void) {
  if(pin >= 0) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  begun = true;


  gpioPort = (GPIO_TypeDef *)digitalPinToPort(pin);
  gpioPin = STM_LL_GPIO_PIN(digitalPinToPinName(pin));
}

void WS2816_Driver::show(void) {
  if(!pixels) return;
  
  while(!canShow());
  
  noInterrupts();
  
  // Using direct STM32 GPIO manipulation for precise timing
  volatile uint16_t *ptr = pixels;
  
  // Send data for each LED
  for(uint16_t led = 0; led < numLEDs; led++) {
    // Send all color components for current LED position
    for(uint16_t color = 0; color < 3; color++) {
      uint16_t colorData = ptr[led * 3 + color];
      
      // Send each bit
      for(uint8_t bit = 0; bit < 16; bit++) {
        LL_GPIO_SetOutputPin(gpioPort, gpioPin);
        
        if(colorData & 0x8000) {
          // T1H: 600ns
          __asm volatile(
            "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
            ::);
          
          LL_GPIO_ResetOutputPin(gpioPort, gpioPin);
          
          // T1L: 600ns
          __asm volatile(
            "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n nop\n"
            ::);
        } else {
          // T0H: 300ns
          __asm volatile(
            "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n"
            ::);
            
          LL_GPIO_ResetOutputPin(gpioPort, gpioPin);
          
          // T0L: 900ns
          __asm volatile(
            "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n nop\n"
            ::);
        }
        
        colorData <<= 1;
      }
    }
  }
  
  // Reset code
  LL_GPIO_ResetOutputPin(gpioPort, gpioPin);
  delayMicroseconds(280); // >280µs according to diagram
  
  interrupts();
  
  endTime = micros();
}


void WS2816_Driver::setPixelColor(uint16_t n, uint16_t r, uint16_t g, uint16_t b) {
  if(n < numLEDs) {
    if(brightness) {
      r = (r * brightness) >> 8;
      g = (g * brightness) >> 8;
      b = (b * brightness) >> 8;
    }
    uint16_t *p = &pixels[n * 3];
    p[0] = g; // GRB order
    p[1] = r;
    p[2] = b;
  }
}

void WS2816_Driver::setBrightness(uint8_t b) {
  uint8_t newBrightness = b;
  if(newBrightness != brightness) {
    uint16_t c, *ptr = pixels;
    uint8_t oldBrightness = brightness;
    uint16_t scale;
    
    if(oldBrightness == 0) scale = 0;
    else if(b == 255) scale = 65535 / oldBrightness;
    else scale = (((uint16_t)newBrightness << 8) - 1) / oldBrightness;
    
    for(uint16_t i = 0; i < numBytes/2; i++) {
      c = *ptr;
      *ptr++ = (c * scale) >> 8;
    }
    brightness = newBrightness;
  }
}

void WS2816_Driver::clear(void) {
  memset(pixels, 0, numBytes);
}

uint32_t WS2816_Driver::getPixelColor(uint16_t n) const {
  if(n >= numLEDs) return 0;
  
  uint16_t *p = &pixels[n * 3];
  uint32_t g = p[0];
  uint32_t r = p[1];
  uint32_t b = p[2];
  
  if(brightness) {
    r = (r << 8) / brightness;
    g = (g << 8) / brightness;
    b = (b << 8) / brightness;
  }
  
  return ((r & 0xFFFF) << 32) | ((g & 0xFFFF) << 16) | (b & 0xFFFF);
}

void WS2816_Driver::setPin(int16_t p) {
  if(begun && (pin >= 0)) pinMode(pin, INPUT);
  pin = p;
  if(begun) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  gpioPort = (GPIO_TypeDef *)digitalPinToPort(p);
  gpioPin = STM_LL_GPIO_PIN(digitalPinToPinName(p));
}
#else
#error "The selected board is not supported"
#endif
