#pragma once
#include <Arduino.h>
#define LX16A_BROADCAST_ID 0xFE
#define LX16A_SERVO_MOVE_TIME_WRITE 1
#define LX16A_SERVO_MOVE_TIME_READ 2
#define LX16A_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LX16A_SERVO_MOVE_TIME_WAIT_READ 8
#define LX16A_SERVO_MOVE_START 11
#define LX16A_SERVO_MOVE_STOP 12
#define LX16A_SERVO_ID_WRITE 13
#define LX16A_SERVO_ID_READ 14
#define LX16A_SERVO_ANGLE_OFFSET_ADJUST 17
#define LX16A_SERVO_ANGLE_OFFSET_WRITE 18
#define LX16A_SERVO_ANGLE_OFFSET_READ 19
#define LX16A_SERVO_ANGLE_LIMIT_WRITE 20
#define LX16A_SERVO_ANGLE_LIMIT_READ 21
#define LX16A_SERVO_VIN_LIMIT_WRITE 22
#define LX16A_SERVO_VIN_LIMIT_READ 23
#define LX16A_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LX16A_SERVO_TEMP_MAX_LIMIT_READ 25
#define LX16A_SERVO_TEMP_READ 26
#define LX16A_SERVO_VIN_READ 27
#define LX16A_SERVO_POS_READ 28
#define LX16A_SERVO_OR_MOTOR_MODE_WRITE 29
#define LX16A_SERVO_OR_MOTOR_MODE_READ 30
#define LX16A_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LX16A_SERVO_LOAD_OR_UNLOAD_READ 32
#define LX16A_SERVO_LED_CTRL_WRITE 33
#define LX16A_SERVO_LED_CTRL_READ 34
#define LX16A_SERVO_LED_ERROR_WRITE 35
#define LX16A_SERVO_LED_ERROR_READ 36

class LX16ABus {
	bool _debug;
public:

	LX16ABus() : _debug(false){
	}

	// debug enables/disables debug printf's for this servo
	void debug(bool on) {
		_debug = on;

	}
	void begin(HardwareSerial * port, int baud = 115200) {
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
	int retry = 3;
	void setRetryCount(int count){
		retry = count;
	}
	// write a command with the provided parameters
	// returns true if the command was written without conflict onto the bus
	bool write_no_retry(uint8_t cmd, const uint8_t *params, int param_cnt,
			uint8_t MYID);

	// read sends a command to the servo and reads back the response into the params buffer.
	// returns true if everything checks out correctly.
	bool read_no_retry(uint8_t cmd, uint8_t *params, int param_len,
			uint8_t MYID);

	// write a command with the provided parameters
	// returns true if the command was written without conflict onto the bus
	bool write(uint8_t cmd, const uint8_t *params, int param_cnt,
			uint8_t MYID) {
		if (retry == 0) {
			return write_no_retry(cmd, params, param_cnt, MYID);
		} else {
			for (int i = 0; i < retry; i++) {
				bool ok = write_no_retry(cmd, params, param_cnt, MYID);
				if (ok)
					return true;
			}
		}
		return false;
	}

	// read sends a command to the servo and reads back the response into the params buffer.
	// returns true if everything checks out correctly.
	bool read(uint8_t cmd, uint8_t *params, int param_len, uint8_t MYID) {
		if (retry == 0) {
			return read_no_retry(cmd, params, param_len, MYID);
		} else {
			for (int i = 0; i < retry; i++) {
				bool ok = read_no_retry(cmd, params, param_len, MYID);
				if (ok)
					return true;
			}
		}
		return false;
	}

	HardwareSerial * _port = NULL;
	int _baud = 115200;
	/**
	 * Command name: SERVO_LOAD_OR_UNLOAD_WRITE
	 Command value: 31 Length: 4
	 Parameter 1: Whether the internal motor of the servo is unloaded power-down
	 or not, the range 0 or 1, 0 represents the unloading power down, and the servo
	 has no torque output. 1 represents the loaded motor, then the servo has a
	 torque output, the default value is 0.
	 @return sucess
	 */
	bool disableAll() {
		uint8_t params[] = { 0 };
		return write(LX16A_SERVO_ID_WRITE, params, 1,
		LX16A_BROADCAST_ID);
	}
	/**
	 * Command name: SERVO_LOAD_OR_UNLOAD_WRITE
	 Command value: 31 Length: 4
	 Parameter 1: Whether the internal motor of the servo is unloaded power-down
	 or not, the range 0 or 1, 0 represents the unloading power down, and the servo
	 has no torque output. 1 represents the loaded motor, then the servo has a
	 torque output, the default value is 0.
	 @return sucess
	 */
	bool enableAll() {
		uint8_t params[] = { 1 };
		return write(LX16A_SERVO_ID_WRITE, params, 1,
		LX16A_BROADCAST_ID);
	}
	/**
	 * Command name: SERVO_MOVE_START Command value: 11
	 Length: 3
	 With the use of command SERVO_MOVE_TIME_WAIT_WRITE, described in
	 point 3
	 @return sucess
	 */
	bool move_sync_start() {
		uint8_t params[1];
		return write(LX16A_SERVO_MOVE_START, params, 1,
		LX16A_BROADCAST_ID);
	}
	/**
	 * Command name: SERVO_MOVE_STOP Command value: 12
	 Length: 3
	 When the command arrives at the servo, it will stop running
	 This is sent to all servos at once
	 */
	void stopAll() {
		uint8_t params[1];
		write(LX16A_SERVO_MOVE_STOP, params, 1,
		LX16A_BROADCAST_ID);
	}
};

class LX16AServo {
private:
	bool commandOK = true;
	int16_t lastKnownGoodPosition = 0;
	bool isMotorMode = false;
	bool isInitialized = false;
	//private:
	LX16ABus &_bus;
	uint8_t _id = LX16A_BROADCAST_ID;

public:
	LX16AServo(LX16ABus &bus, int id) :
			_bus(bus), _id(id) {
	}

	bool isCommandOk() {
		return commandOK;
	}
	void initialize() {
		if (isInitialized) {
			return;
		}
		isInitialized = true;
		motor_mode(0);
		pos_read();
	}

	/**
	 * Length: 7
	 Parameter 1: lower 8 bits of angle value
	 Parameter 2: higher 8 bits of angle value.range 0~100. corresponding to the
	 servo angle of 0 ~ 240 °, that means the minimum angle of the servo can be
	 varied is 0.24 degree.
	 Parameter 3: lower 8 bits of time value
	 Parameter 4: higher 8 bits of time value. the range of time is 0~30000ms.
	 When the command is sent to servo, the servo will be rotated from current
	 angle to parameter angle at uniform speed within param
	 */
	void move_time(uint16_t angle, uint16_t time) {
		initialize();
		if (isMotorMode)
			motor_mode(0);
		angle = angle / 24;
		uint8_t params[] = { (uint8_t) angle, (uint8_t) (angle >> 8),
				(uint8_t) time, (uint8_t) (time >> 8) };
		commandOK = _bus.write(LX16A_SERVO_MOVE_TIME_WRITE, params, 4, _id);
	}
	/**
	 * Command name: SERVO_MOVE_TIME_WAIT_WRITE
	 Command value: 7
	 Length : 7
	 Parameter1: lower 8 bits of preset angle
	 Parameter2: higher 8 bits of preset angle. range 0~100. corresponding to the
	 servo angle of 0 ~ 240 °. that means the minimum angle of the servo can be
	 varied is 0.24 degree.
	 Parameter3: lower 8 bits of preset time
	 Parameter3: higher 8 bits of preset time. the range of time is 0~30000ms.
	 The function of this command is similar to this
	 “SERVO_MOVE_TIME_WRITE” command in the first point. But the difference
	 is that the servo will not immediately turn when the command arrives at the
	 servo,the servo will be rotated from current angle to parameter angle at unifor
	 m speed within parameter time until the command name SERVO_MOVE_ST
	 ART sent to servo(command value of 11)
	 , then the servo will be rotate
	 */
	void move_time_and_wait_for_sync(uint16_t angle, uint16_t time) {
		initialize();
		if (isMotorMode)
			motor_mode(0);
		angle = angle / 24;
		uint8_t params[] = { (uint8_t) angle, (uint8_t) (angle >> 8),
				(uint8_t) time, (uint8_t) (time >> 8) };
		commandOK = _bus.write(LX16A_SERVO_MOVE_TIME_WAIT_WRITE, params, 4,
				_id);
	}

	/**
	 * Command name: SERVO_MOVE_STOP Command value: 12
	 Length: 3
	 When the command arrives at the servo, it will stop running
	 */
	void stop() {
		uint8_t params[1];
		commandOK = _bus.write(LX16A_SERVO_MOVE_STOP, params, 1, _id);
	}

	/**
	 * Command name: SERVO_LOAD_OR_UNLOAD_WRITE
	 Command value: 31 Length: 4
	 Parameter 1: Whether the internal motor of the servo is unloaded power-down
	 or not, the range 0 or 1, 0 represents the unloading power down, and the servo
	 has no torque output. 1 represents the loaded motor, then the servo has a
	 torque output, the default value is 0.
	 */
	void disable() {
		uint8_t params[] = { 0 };
		commandOK = _bus.write(LX16A_SERVO_ID_WRITE, params, 1, _id);
	}
	/**
	 * Command name: SERVO_LOAD_OR_UNLOAD_WRITE
	 Command value: 31 Length: 4
	 Parameter 1: Whether the internal motor of the servo is unloaded power-down
	 or not, the range 0 or 1, 0 represents the unloading power down, and the servo
	 has no torque output. 1 represents the loaded motor, then the servo has a
	 torque output, the default value is 0.
	 */
	void enable() {
		uint8_t params[] = { 1 };
		commandOK = _bus.write(LX16A_SERVO_ID_WRITE, params, 1, _id);
	}

	/**
	 * Command name: SERVO_OR_MOTOR_MODE_WRITE
	 Command value: 29
	 Length: 7
	 Parameter 1: Servo mode, range 0 or 1, 0 for position control mode, 1 for
	 motor control mode, default 0,
	 Parameter 2: null value
	 Parameter 3: lower 8 bits of rotation speed value
	 Parameter 4: higher 8 bits of rotation speed value. range -1000~1000,
	 Only in the motor control mode is valid, control the motor speed, the value of
	 the negative value represents the reverse, positive value represents the
	 forward rotation. Write mode and speed do not support power-down save.
	 Note: Since the rotation speed is the “signed short int” type of data, it is forced
	 to convert the data to convert the data to “unsigned short int “type of data before sending the
	 command packet.
	 */
	void motor_mode(int16_t speed) {
		bool isMotorMode_tmp = speed != 0;
		uint8_t params[] = { (uint8_t) (isMotorMode_tmp ? 1 : 0), 0,
				(uint8_t) speed, (uint8_t) (speed >> 8) };
		commandOK = _bus.write(LX16A_SERVO_OR_MOTOR_MODE_WRITE, params, 4, _id);
		if (commandOK)
			isMotorMode = isMotorMode_tmp;
	}

	// angle_adjust sets the position angle offset in centi-degrees (-3000..3000)
	void angle_adjust(int16_t angle) {
		uint8_t params[] = { (uint8_t) ((int32_t) angle * 125 / 30) };
		commandOK = _bus.write(LX16A_SERVO_ANGLE_OFFSET_ADJUST, params, 1, _id);
	}

	// angle_limit sets the upper and lower position limit in centi-degrees (0..24000)
	void angle_limit(uint16_t min_angle, uint16_t max_angle) {
		min_angle = min_angle / 24;
		max_angle = max_angle / 24;
		uint8_t params[] = { (uint8_t) min_angle, (uint8_t) (min_angle >> 8),
				(uint8_t) max_angle, (uint8_t) (max_angle >> 8) };
		commandOK = _bus.write(LX16A_SERVO_ANGLE_LIMIT_WRITE, params, 4, _id);
	}

	// pos_read returns the servo position in centi-degrees (0..24000)
	int16_t pos_read() {
		uint8_t params[2];
		if (!_bus.read(LX16A_SERVO_POS_READ, params, 2, _id)) {
			commandOK = false;
			return lastKnownGoodPosition;
		}
		commandOK = true;
		lastKnownGoodPosition = ((int16_t) params[0]
				| ((int16_t) params[1] << 8)) * 24;
		return lastKnownGoodPosition;
	}

	// id_read returns the ID of the servo, useful if the id is 0xfe, which is broadcast...
	uint8_t id_read() {
		uint8_t params[1];
		if (!_bus.read(LX16A_SERVO_ID_READ, params, 1, _id)) {
			commandOK = false;
			return 0;
		}
		commandOK = true;
		return params[0];

	}
	/**
	 * Command name: SERVO_OR_MOTOR_MODE_READ
	 Command value: 30 Length: 3
	 Read the relative values of the servo, for the details of the command package
	 that the servo returns to host computer, please refer to the description of Table
	 4 below.
	 */
	bool readIsMotorMode() {
		uint8_t params[1];
		if (!_bus.read(LX16A_SERVO_OR_MOTOR_MODE_READ, params, 1, _id)) {
			commandOK = false;
			return false;
		}
		commandOK = true;
		isMotorMode = params[0] == 1;
		return isMotorMode;
	}
	// id_write sets the id of the servo, updates the object's id if write appears successful
	void id_write(uint8_t id) {
		uint8_t params[] = { id };
		bool ok = _bus.write(LX16A_SERVO_ID_WRITE, params, 1, _id);
		if (ok)
			_id = id;
		commandOK = ok;
	}

	// temp_read returns the servo temperature in centigrade
	uint8_t temp() {
		uint8_t params[1];
		if (!_bus.read(LX16A_SERVO_TEMP_READ, params, 1, _id)) {
			commandOK = false;
			return 0;
		}
		commandOK = true;
		return params[0];
	}

	// vin_read returns the servo input voltage in millivolts
	uint16_t vin() {
		uint8_t params[2];
		if (!_bus.read(LX16A_SERVO_VIN_READ, params, 2, _id)) {
			commandOK = false;
			return 0;
		}
		commandOK = true;
		return params[0] | ((uint16_t) params[1] << 8);
	}


};
