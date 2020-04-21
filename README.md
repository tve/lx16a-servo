# LX-16A Servo Library

Simple Arduino library to operate LX-16A serial servos.

This library sends simple commands to LewanSoul LX-16A serial bus servos.
It is designed for the Arduino framework and uses a single pin to interface to the servos
as opposed to the more common 3-pin configuration (TX, RX, direction).

The library's LX16AServo class provides two main methods to write a command and to read settings.
It's very simple!

# Electrical

```
MCU RX -> Direct Connection -> LX-16a Serial Pin
MCU TX -> 1K Ohm resistor   -> LX-16a Serial Pin
6v-7.5v ->  LX-16a Power (center) pin
GND     ->  LX-16a GND Pin
```

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
