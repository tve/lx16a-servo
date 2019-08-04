class LX16ABus {
public:
    LX16ABus()
        : _port(Serial1)
        , _baud(115200)
    {}

    void begin(HardwareSerial &port, int pin, int baud=115200) {
        _port = port;
        _baud = baud;
        port.begin(baud, SERIAL_8N1, pin, pin);
        pinMode(pin, OUTPUT|PULLUP|OPEN_DRAIN);
        delay(3);
        while (port.available()) port.read();
    }

    // time returns the number of ms to TX/RX n characters
    uint32_t time(uint8_t n) {
        return n*10*1000/_baud; // 10 bits per char
    }

    // methods passed through to Serial
    bool available() { return _port.available(); }
    int read() { return _port.read(); }
    void write(const uint8_t *buf, int buflen) { _port.write(buf, buflen); }

//private:
    HardwareSerial &_port;
    int _baud;
};

class LX16AServo {
public:
    LX16AServo(LX16ABus &bus, int id)
        : _bus(bus)
        , _id(id)
        , _debug(false)
    {}

    // debug enables/disables debug printf's for this servo
    void debug(bool on) { _debug = on; }

    // write a command with the provided parameters
    // returns true if the command was written without conflict onto the bus
    bool write(uint8_t cmd, const uint8_t *params, int param_cnt);

    // read sends a command to the servo and reads back the response into the params buffer.
    // returns true if everything checks out correctly.
    bool read(uint8_t cmd, uint8_t *params, int param_len);

//private:
    LX16ABus &_bus;
    uint8_t _id;
    bool _debug;
};
