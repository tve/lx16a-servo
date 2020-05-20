# LX-16A, LX-224 and LX-15D Servo Library

Simple Arduino library to operate LX-16A, LX-224 and LX-15D serial servos.

This library sends simple commands to LewanSoul LX-16A, LX-224  and Hiwonder LX-15D serial bus servos.
It is designed for the Arduino framework and uses a single pin to interface to the servos
as opposed to the more common 3-pin configuration (TX, RX, direction).

The library's LX16AServo class provides two main methods to write a command and to read settings.
It's very simple!

# Electrical

The LX-* servos all use a 3.3v driven bi directional asynchronus serial. It is similar to UART, but uses both signals on one pin. Because of this, the Master TX line has to be connected only while transmitting. The correct way to do this is a buffer chip 74HC126. 

https://www.digikey.com/product-detail/en/texas-instruments/SN74HC126N/296-8221-5-ND

This library uses an IO pin passed to the begin() method to flag when the master is transmitting on the bus. When the flag is de-asserted, then the bus is freed for a motor to transmit, with the Masters UART TX line held in high-impedance.

```
MCU RX -> Direct Connection -> LX-* Serial Pin
MCU TX -> 74HC126 A   
MCU Flag GPIO -> 74HC126 OE
74HC126 Y -> LX-* Serial Pin
6v-7.5v ->  LX-* Power (center) pin
GND     ->  LX-* GND Pin
```

The MCU Rx pin always listens,a nd hears its own bytes comming in. This library clears out the incomming bytes and will hang if it could not hear itself talking. 

### ESP32 and Teensy

The esp32 and the Teensy microcontrollers have the ability to run the serial Tx in Open Drain mode. This is detected by the library and done automatically. This means the pins for tx and rx can be connected together and used to talk to the motors without bus conflicts. 

```
MCU RX -> Direct Connection -> LX-16a Serial Pin
MCU TX -> Direct Connection   -> LX-16a Serial Pin
LX-16a Serial Pin -> 1k Ohm resistor -> 3.3v
6v-7.5v ->  LX-16a Power (center) pin
GND     ->  LX-16a GND Pin
```
### Other Arduino Quick and dirty/ Hacky one motor setup

This wireing configuration will get you up and running fast. You will get a few failed commands and errored bytes using this method. It is usefull to quickly test a servo on a new system.  

```
MCU RX -> Direct Connection -> LX-16a Serial Pin
MCU TX -> 1K Ohm resistor   -> LX-16a Serial Pin
6v-7.5v ->  LX-16a Power (center) pin
GND     ->  LX-16a GND Pin
```

