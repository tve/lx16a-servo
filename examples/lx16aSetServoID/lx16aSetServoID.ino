#include <Arduino.h>
#include <lx16a-servo.h>
LX16ABus servoBus;
LX16AServo servo(&servoBus, LX16A_BROADCAST_ID);// send these commands to any motor on the bus
int id =3;
void setup() {
	servoBus.begin(&Serial1,
			1,// on TX pin 1
			2);// use pin 2 as the TX flag for buffer
	Serial.begin(115200);
}

void loop() {
	// Set any motor plugged in to ID 3
	// this INO acts as an auto-provisioner for any motor plugged in
	servo.id_write(id);
	Serial.println("Attempting set to ID "+String (id));
	int read=servo.id_read();
	if(read!=id || read==0){
		Serial.println("\r\nERROR ID set failed");
		delay(500);
	}else{
		Serial.println("\r\nCurrent ID is now "+String(read));
	}
	delay(200);

}

