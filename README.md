Adafruit_INA260 [![Build Status](https://github.com/adafruit/Adafruit_INA260/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_INA260/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_INA260/html/index.html)
================

This is the Adafruit INA260 Current and Power sensor library

Tested and works great with the [Adafruit INA260 Breakout Board](http://www.adafruit.com/products/4226)

This chip uses I2C to communicate, 2 pins are required to interface

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Bryan Siepert for Adafruit Industries.
BSD license, check license.txt for more information
All text above must be included in any redistribution

To install, use the Arduino Library Manager and search for "Adafruit INA260" and install the library.



Sets the averaging mode (1x to 1024x) by writing to config register.
Value from 0–7 corresponding to 1x–1024x
Added by Sverd Industries to support power smoothing

ina260.setAveragingMode(4); // 128 samples

| avg\_mode | Samples Averaged |
| --------- | ---------------- |
| `0`       | 1x               |
| `1`       | 4x               |
| `2`       | 16x              |
| `3`       | 64x              |
| `4`       | 128x             |
| `5`       | 256x             |
| `6`       | 512x             |
| `7`       | 1024x            |
