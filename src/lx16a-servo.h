#pragma once
#include <Arduino.h>
class LX16ABus {
public:
	LX16ABus() {
	}
	void begin(HardwareSerial  * port,int baud = 115200) {
		_port = port;
		_baud = baud;
		port->begin(baud, SERIAL_8N1);
		delay(3);
		while (port->available())
			port->read();
	}
	void beginOnPin(HardwareSerial * port, int pin, int baud = 115200) {
		_port = port;
		_baud = baud;
#if defined ARDUINO_ARCH_ESP32
        port->begin(baud, SERIAL_8N1, pin, pin);
        pinMode(pin, OUTPUT|PULLUP|OPEN_DRAIN);
#else
		port->begin(baud, SERIAL_8N1);
		pinMode(pin, OUTPUT_OPENDRAIN);
#endif
		delay(3);
		while (port->available())
			port->read();
	}

	// time returns the number of ms to TX/RX n characters
	uint32_t time(uint8_t n) {
		return n * 10 * 1000 / _baud; // 10 bits per char
	}

	// methods passed through to Serial
	bool available() {
		return _port->available();
	}
	int read() {
		return _port->read();
	}
	void write(const uint8_t *buf, int buflen) {
		_port->write(buf, buflen);
	}

//private:
	HardwareSerial * _port =NULL;
	int _baud=115200;
};

class LX16AServo {
private:
	bool commandOK = true;
public:
	LX16AServo(LX16ABus &bus, int id) :
			_bus(bus), _id(id), _debug(false) {
	}

	// debug enables/disables debug printf's for this servo
	void debug(bool on) {
		_debug = on;
	}
	bool isCommandOk(){
		return commandOK;
	}

	// write a command with the provided parameters
	// returns true if the command was written without conflict onto the bus
	bool write(uint8_t cmd, const uint8_t *params, int param_cnt);

	// read sends a command to the servo and reads back the response into the params buffer.
	// returns true if everything checks out correctly.
	bool read(uint8_t cmd, uint8_t *params, int param_len);

	// motor_mode causes the motor to rotate at a fixed speed (-1000..1000) and switches to
	// position (servo) mode if speed==0
	void motor_mode(uint16_t speed) {
		uint8_t params[] = { (uint8_t) (speed == 0 ? 0 : 1), 0, (uint8_t) speed,
				(uint8_t) (speed >> 8) };
		commandOK=  write(29, params, sizeof(params));
	}

	// angle_adjust sets the position angle offset in centi-degrees (-3000..3000)
	void angle_adjust(int16_t angle) {
		uint8_t params[] = { (uint8_t) ((int32_t) angle * 125 / 30) };
		commandOK= write(17, params, sizeof(params));
	}

	// angle_limit sets the upper and lower position limit in centi-degrees (0..24000)
	void angle_limit(uint16_t min_angle, uint16_t max_angle) {
		min_angle = min_angle / 24;
		max_angle = max_angle / 24;
		uint8_t params[] = { (uint8_t) min_angle, (uint8_t) (min_angle >> 8),
				(uint8_t) max_angle, (uint8_t) (max_angle >> 8) };
		commandOK=  write(20, params, sizeof(params));
	}

	// move_time positions the servo to the angle in centi-degrees (0..24000) in time milliseconds (0..1000)
	void move_time(uint16_t angle, uint16_t time) {
		angle = angle / 24;
		uint8_t params[] = { (uint8_t) angle, (uint8_t) (angle >> 8),
				(uint8_t) time, (uint8_t) (time >> 8) };
		commandOK=  write(1, params, sizeof(params));
	}

	// pos_read returns the servo position in centi-degrees (0..24000)
	int16_t pos_read(int16_t * angle) {
		uint8_t params[2];
		if (!read(28, params, sizeof(params))){
			commandOK=  false;
			return 0;
		}
		commandOK=  true;
		return ((int16_t) params[0] | ((int16_t) params[1] << 8)) * 24;
	}

	// id_read returns the ID of the servo, useful if the id is 0xfe, which is broadcast...
	uint8_t id_read() {
		uint8_t params[1];
		if (!read(14, params, sizeof(params))){
			commandOK=  false;
			return 0;
		}
		commandOK=  true;
		return params[0];

	}

	// id_write sets the id of the servo, updates the object's id if write appears successful
	void id_write(uint8_t id) {
		uint8_t params[] = { id };
		bool ok = write(13, params, sizeof(params));
		if (ok)
			_id = id;
		commandOK=  ok;
	}

	// temp_read returns the servo temperature in centigrade
	uint8_t temp() {
		uint8_t params[1];
		if (!read(26, params, sizeof(params))){
			commandOK=  false;
			return 0;
		}
		commandOK=  true;
		return params[0];
	}

	// vin_read returns the servo input voltage in millivolts
	uint16_t vin() {
		uint8_t params[2];
		if (!read(27, params, sizeof(params))){
			commandOK=  false;
			return 0;
		}
		commandOK=  true;
		return params[0] | ((uint16_t) params[1] << 8);
	}

//private:
	LX16ABus &_bus;
	uint8_t _id;
	bool _debug;
};
