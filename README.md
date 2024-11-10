# WS2816-Driver

## Introduction

This repo contains a basic driver to control 16-bit WS2816 Neopixels.
It is based on the Adafruit NeoPixel Library ((https://github.com/adafruit/Adafruit_NeoPixel) released under the LGPL-3.0 license) but modified to suit the 16-bit mode and different timing needs of the WS2816.

## Installation

Download the latest .zip file from the releases tab and install the library in your Arduino-IDE via: Sketch->Include Library->Add .ZIP Library.

Now you should be able to open the example file WS2816-Example.ino.

Make sure to adjust the pin and number of LEDs accourding to your setup.

## Supported architectures

At the moment only the ARDUINO_ARCH_STM32/ARDUINO_ARCH_ARDUINO_CORE_STM32 architecture is supported.

## Contribution

If you would like to add support for different architectures, add more functionality or would like to fix a bug, feel free to open a pull request.

## Additional notes

Please note that I mainly created this library to control WS2816 Neopixels in a recent project of mine, so don't expect a huge amount of active development in the future.