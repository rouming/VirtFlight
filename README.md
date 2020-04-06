# VirtFlight
VirtFlight is a PWM,PPM,IBUS to USB converter firmware for STM32F103 BluePill
board. Firmware implements USB HID for two devices: joystick and keyboard.

## Support
VirtFlight joystick HID has 6 axis, keyboard HID is used for mapping 5'th
and 6'th channels (SWA and SWD switches on my FS-i6/TGY-i6 transmitter) to
't' and 'r' keys respectively, which provides convenient way to
do RESET or do TURTLE in "Liftoff" quad simulator.

* 6 axis joystick
* Mapping of 5'th and 6'th channel to 'r' and 't' keys to support
  Liftoff key commands
* PWM

## Pinout

![Alt text](https://i.imgur.com/ucgJGEs.jpg)

6 PWM channels can be connected to A0-A5 pins on BluePill board.

PPM and IBUS are not yet implemented.
