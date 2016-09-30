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


# Operating Modes Toggled via Mode Button

1. SHOW_TIME
2. SHOW_TEMP    (No timeout return in this mode)
3. SHOW_DATE
4. SHOW_YEAR
5. SET_SEC
6. SET_MIN
7. SET_HOUR
8. SET_DAY
9. SET_MONTH
10. SET_YEAR
11. SET_BRIGHTNESS
12. LED_TEST 12 (Long Hold to Exist this mode as only checks at end of each Test Loop)

All modes baring Time and Temp Timout and Return to Time Mode
Current Mode is indicated on Hour LEDs Blinking the Mode where appropriate.

