#ifndef HARDWARESERIAL_H_INCLUDED
#define HARDWARESERIAL_H_INCLUDED

#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <termios.h>

class Stream
{
    public:
        Stream();
		void setSerial(int _fd);
        int available();
        int read();
		void flush();
        void write(const uint8_t);
		
    protected:
        int m_fd = 0;
};

Stream::Stream()
{
}

void Stream::setSerial(int _fd)
{
	m_fd = _fd;
}

int Stream::available()
{
    return (bool)(serialDataAvail(m_fd));
}

int Stream::read()
{
    return serialGetchar(m_fd);
}

void Stream::flush()
{
    tcdrain(m_fd);
	serialFlush(m_fd);
}

void Stream::write(const uint8_t ch)
{
    serialPutchar(m_fd, ch);
}

class HardwareSerial : public Stream
{
    public:
        HardwareSerial();
        int begin(const char *device, const speed_t baudrate);
        void end();

    private:
};

extern HardwareSerial Serial;

HardwareSerial::HardwareSerial()
{
}

int HardwareSerial::begin(const char *device, const speed_t baudrate)
{
    // open the port
    if ((m_fd = serialOpen(device, baudrate)) < 0) {
		return -1;
	}

    return 1;
}

void HardwareSerial::end()
{
    serialClose(m_fd);
}

HardwareSerial Serial;

#endif // HARDWARESERIAL_H_INCLUDED