#include <Arduino.h>
#include <lx16a-servo.h>
LX16ABus servoBus;
LX16AServo servo(servoBus, 1);
LX16AServo servo2(servoBus, 2);
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


		servo.move_time_and_wait_for_sync(angle, 500);
		servo2.move_time_and_wait_for_sync(angle, 500);

		servoBus.move_sync_start();

		Serial.printf("Move to %d -> %s\n", angle,
				servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");
		Serial.println("Voltage = " + String(servo.vin()));
		Serial.println("Temp = " + String(servo.temp()));
		Serial.println("ID  = " + String(servo.id_read()));

		delay(2000);
	}

}
