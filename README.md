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
servoBus.begin(Serial1);
```

Step servo through its 240 degrees range, 10% at a time:
```
	for (int i = 0; i < 10; i++) {
		int16_t pos = 0;
		pos = servo.pos_read();
		Serial.printf("\n\nPosition at %d -> %s\n", pos,
				servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");

		uint16_t angle = i * 2400;

		do {
			servo.move_time(angle, 500);
		} while (!servo.isCommandOk());
		Serial.printf("Move to %d -> %s\n", angle,
				servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");
		Serial.println("Voltage = " + String(servo.vin()));
		Serial.println("Temp = " + String(servo.temp()));
		Serial.println("ID  = " + String(servo.id_read()));
		Serial.println("Motor Mode  = " + String(servo.readIsMotorMode()));

		delay(200);

		//servo.stopAll();
		delay(1800);

	}
```
