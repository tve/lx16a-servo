Direct Connection# LX-16A, LX-224 and LX-15D Servo Library

Simple Arduino library to operate LX-16A, LX-224 and LX-15D serial servos.

This library sends simple commands to LewanSoul LX-16A, LX-224  and Hiwonder LX-15D serial bus servos.
It is designed for the Arduino framework and uses a single pin to interface to the servos
as opposed to the more common 3-pin configuration (TX, RX, direction).

The library's LX16AServo class provides two main methods to write a command and to read settings.
It's very simple!

# Electrical

The LX-* servos all use a 3.3v driven bi directional asynchronus serial. It is similar to UART, but uses both signels on one pin. Because of this, the Master TX line has to be connected only while transmitting. THe correct way to do this is a Buffer chip. 

https://www.digikey.com/product-detail/en/texas-instruments/SN74HC126N/296-8221-5-ND

This library uses an IO pin passed to the begin() method toi flag when the master is transmitting on the bus. When the flag is desserted, then the bus is freed for a motor to transmit, and the Masters UARD TX line should be held in high-impedance.

```
MCU RX -> Direct Connection -> LX-16a Serial Pin
MCU TX -> 74HC126 A   
MCU Flag GPIO -> 74HC126 OE
74HC126 Y -> LX-16a Serial Pin
6v-7.5v ->  LX-16a Power (center) pin
GND     ->  LX-16a GND Pin
```


### Quick and dirty/ Hacky one motor setup
This wireing configuration will get you up and running fast. You will get a few failed commands and errored bytes using this method. 

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

Initialize the bus to use Serial1:
```
servoBus.begin(Serial1,2);// use pin 2 as the TX flag for buffer
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
