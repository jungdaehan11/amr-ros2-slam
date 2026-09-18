// SerialPort.h - termios 기반 시리얼 (외부 의존성 0)
#pragma once
#include <string>
#include <vector>
#include <cstdint>

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    bool open(const std::string& dev, int baud);  // 성공 시 true
    void close();
    bool isOpen() const { return fd_ >= 0; }
    ssize_t write(const std::vector<uint8_t>& data);
    ssize_t readAvailable(std::vector<uint8_t>& out);  // 논블로킹, 있는 만큼 읽음
private:
    int fd_ = -1;
};
