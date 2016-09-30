# Binary-Clock-with-Temp-Sensor

This Project and Code Creates a very simple binary clock with added Temp Sensor Mode.

# Hardware
 
* Ardunio
* PCA9685
* DS3231 RTC
* ProtoBoard
* Header Pins
* Push Buttons
* 16 LED's Ideally 3 different Colours (6,6,4)
* 10K Resistor  x 2
* 4.7K Resistor

# Basic Circuit Overview

LEDS are Driven via PWM from the PCA9685

* Channels 0-5 Are Second Bits 1,2,4,8,16,32
* Channels 6-11 Are Minute Bits 1,2,4,8,16,32
* Channels 12-15 Are Hour Bits 1,2,4,8 
* Note: Current Design doesnt have an AM/PM or 24 hour feature.


