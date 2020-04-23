# VirtFlight
VirtFlight is a PWM,PPM,IBUS to USB converter firmware for STM32F103 BluePill
board. Firmware implements USB HID for two devices: joystick and keyboard.

## Support

* 6 axis joystick
* PWM - if detected makes 1 long blink in a second with a green status led
* PPM - if detected makes 2 short blinks in a second with a green status led

## Pinout

![Alt text](https://i.imgur.com/ucgJGEs.jpg)

* 6 PWM channels can be connected to A0-A5 pins of the BluePill board.
* PPM can be connected to the A0 pin of the BluePill board.

IBUS is not yet implemented.
