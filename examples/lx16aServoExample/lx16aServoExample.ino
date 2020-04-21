#include <Arduino.h>
#include <lx16a-servo.h>
LX16ABus servoBus;
LX16AServo servo(servoBus, 1);
int n = 0;
void setup(){
	servoBus.begin(Serial1, 33);
}

void loop(){
    uint16_t angle = (n%11) * 100;

    uint8_t params[] = { (uint8_t)angle, (uint8_t)(angle>>8), 500&0xff, 500>>8 };
    bool ok = servo.write(1, params, sizeof(params));
//    printf("Move to %d -> %s\n", angle, ok?"OK":"ERR");

    delay(10);

    ok = servo.read(2, params, 4);
//    printf("Position at %d -> %s\n", params[0]|(params[1]<<8), ok?"OK":"ERR");

    n++;
    delay(2000);
}
