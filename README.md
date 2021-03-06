# LX-16A Servo Library

Simple ESP32-Arduino library to operate LX-16A serial servos.

This library sends simple commands to LewanSoul LX-16A serial bus servos.
It is designed for the ESP32 Arduino framework and uses a single pin to interface to the servos
as opposed to the more common 3-pin configuration (TX, RX, direction).

The library's LX16AServo class provides two main methods to write a command and to read settings.
It's very simple!

## Quick start

Allocate a bus object to represent the serial line, and a servo object for the first servo:
```
LX16ABus servoBus;
LX16AServo servo(servoBus, 1);
```

Initialize the bus to use Serial1 on pin 33:
```
servoBus.begin(Serial1, 33);
```

Step servo through its 240 degrees range, 10% at a time:
```
int n = 0;
loop() {
    uint16_t angle = (n%11) * 100;

    uint8_t params[] = { (uint8_t)angle, (uint8_t)(angle>>8), 500&0xff, 500>>8 };
    bool ok = servo.write(1, params, sizeof(params));
    printf("Move to %d -> %s\n", angle, ok?"OK":"ERR");

    delay(10);

    ok = servo.read(2, params, 4);
    printf("Position at %d -> %s\n", params[0]|(params[1]<<8), ok?"OK":"ERR");

    n++;
    delay(2000);
}
```

## High-level methods

The library implements a number of high-level functions which correspond to the lx16-a commands. For
example:

```
// angle_adjust sets the position angle offset in centi-degrees (-3000..3000)
bool angle_adjust(int16_t angle);

// temp_read returns the servo temperature in centigrade
bool temp(uint8_t &temp);
```
Not all commands have been implemented, but it's easy to add any that are needed and missing.
See `src/LC16AServo.h`.
