#include <Arduino.h>
#include <lx16a-servo.h>
LX16ABus servoBus;
LX16AServo servo(servoBus, 1);
void setup(){
	servoBus.begin(&Serial1);
	Serial.begin(115200);
}

void loop(){
	for (int i=0;i<10;i++){
		int16_t pos=0 ;
		pos = servo.pos_read(&pos);

		Serial.printf("Position at %d -> %s\n", pos, servo.isCommandOk()?"OK":"ERR");

		uint16_t angle = i * 2400;


		servo.move_time(angle,500);
		Serial.printf("Move to %d -> %s\n", angle, servo.isCommandOk()?"OK":"ERR");

		delay(2000);
	}

}
