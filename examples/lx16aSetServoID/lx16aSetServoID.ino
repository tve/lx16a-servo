#include <Arduino.h>
#include <lx16a-servo.h>
LX16ABus servoBus;
LX16AServo servo(&servoBus, LX16A_BROADCAST_ID);// send these commands to any motor on the bus
void setup() {
	servoBus.begin(&Serial1);
	Serial.begin(115200);
}

void loop() {
	// Set any motor plugged in to ID 3
	// this INO acts as an auto-provisioner for any motor plugged in
	servo.id_write(1);
}

