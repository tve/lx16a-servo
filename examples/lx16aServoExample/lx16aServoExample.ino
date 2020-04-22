#include <Arduino.h>
#include <lx16a-servo.h>
LX16ABus servoBus;
LX16AServo servo(servoBus, 1);
void setup() {
	servoBus.begin(&Serial1);
	Serial.begin(115200);
}

void loop() {
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


