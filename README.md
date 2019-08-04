# LX-16A Servo Library

Simple ESP32-Arduino library to operate LX-16A serial servos.

This library sends simple commands to LewanSoul LX-16A serial bus servos.
It is designed for the ESP32 Arduino framework and uses a single pin to interface to the servos
as opposed to the more common 3-pin configuration (TX, RX, direction).

The library's LX16AServo class provides two main methods to write a command and to read settings.
It's very simple!
