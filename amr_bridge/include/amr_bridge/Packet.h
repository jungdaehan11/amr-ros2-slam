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
    CMD_SET_PWM = 0x15,     // ★ 4단계 추가: 연속 PWM (L,R int8)
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
//   - 센서(아두이노→Pi): 6바이트, DATA 1개, chk = LEN(0x01) ^ CMD ^ DATA
std::vector<uint8_t> buildArduinoCmd(uint8_t cmd);
ParseResult parseArduinoSensor(const std::vector<uint8_t>& pkt);

// ★ 4단계 추가: 연속 PWM 명령 (7바이트)
//   [STX][LEN=0x03][CMD=0x15][L_int8][R_int8][CHK=LEN^CMD^L^R][ETX]
//   주의: 이 명령만 chk에 DATA(L,R) 포함 — 기존 5바이트 명령(chk=len^cmd)과 비대칭.
//   left/right: 부호 있는 8비트 (-127~127). 펌웨어에서 x2 스케일업 후 방향+PWM 분리.
std::vector<uint8_t> buildArduinoSetPWM(int8_t left, int8_t right);
