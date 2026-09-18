// SerialPort.cpp - termios 기반 시리얼 구현
#include "amr_bridge/SerialPort.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& dev, int baud) {
    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) { close(); return false; }

    speed_t sp = B115200;
    if (baud == 9600)   sp = B9600;
    if (baud == 57600)  sp = B57600;
    if (baud == 115200) sp = B115200;
    cfsetospeed(&tty, sp);
    cfsetispeed(&tty, sp);

    // 8N1, raw 모드
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag = 0;   // non-canonical, no echo
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close(); return false; }
    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

ssize_t SerialPort::write(const std::vector<uint8_t>& data) {
    if (fd_ < 0) return -1;
    return ::write(fd_, data.data(), data.size());
}

ssize_t SerialPort::readAvailable(std::vector<uint8_t>& out) {
    if (fd_ < 0) return -1;
    uint8_t buf[256];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) out.insert(out.end(), buf, buf + n);
    return n;
}
