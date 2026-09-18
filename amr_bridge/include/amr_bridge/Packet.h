// Packet.h - AMR 패킷 프로토콜 선언 (프로젝트1 C++ 포팅본 재사용)
#pragma once
#include <vector>
#include <cstdint>

constexpr uint8_t STX = 0x02;
constexpr uint8_t ETX = 0x03;

enum Cmd : uint8_t {
    CMD_MOVE_FWD = 0x10,
    CMD_MOVE_BACK = 0x11,
    CMD_MOVE_LEFT = 0x12,
    CMD_MOVE_RIGHT = 0x13,
    CMD_MOVE_STOP = 0x14,
    CMD_DIST = 0x20,
    CMD_CURRENT = 0x21,
    CMD_HEARTBEAT = 0x30,
    CMD_WARN_ON = 0x40,
    CMD_WARN_OFF = 0x41,
};

struct ParseResult {
    bool ok;
    uint8_t cmd;
    std::vector<uint8_t> data;
    const char* err;
};

// 프로젝트1 범용 함수 (PC간 통신용, chk = CMD^DATA)
uint8_t calcChecksum(uint8_t cmd, const std::vector<uint8_t>& data);
std::vector<uint8_t> buildPacket(uint8_t cmd, const std::vector<uint8_t>& data);
ParseResult parsePacket(const std::vector<uint8_t>& pkt);
void printHex(const std::vector<uint8_t>& pkt);

// ★ 3-B 추가: 아두이노 펌웨어 프로토콜 어댑터
//   - 명령(Pi→아두이노): 5바이트, DATA 없음, chk = LEN(0x01) ^ CMD
//   - 센서(아두이노→Pi): 6바이트, DATA 1개, chk = LEN(0x01) ^ CMD  (펌웨어가 DATA를 chk에 안 넣음)
std::vector<uint8_t> buildArduinoCmd(uint8_t cmd);
ParseResult parseArduinoSensor(const std::vector<uint8_t>& pkt);
