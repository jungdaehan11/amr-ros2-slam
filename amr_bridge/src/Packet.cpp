// Packet.cpp - AMR 패킷 프로토콜 구현
#include "amr_bridge/Packet.h"
#include <iostream>
#include <iomanip>

// ===== 프로젝트1 범용 (chk = CMD ^ DATA) =====
uint8_t calcChecksum(uint8_t cmd, const std::vector<uint8_t>& data) {
    uint8_t chk = cmd;
    for (uint8_t b : data) chk ^= b;
    return chk;
}

std::vector<uint8_t> buildPacket(uint8_t cmd, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> pkt;
    pkt.push_back(STX);
    pkt.push_back(static_cast<uint8_t>(data.size()));
    pkt.push_back(cmd);
    for (uint8_t b : data) pkt.push_back(b);
    pkt.push_back(calcChecksum(cmd, data));
    pkt.push_back(ETX);
    return pkt;
}

ParseResult parsePacket(const std::vector<uint8_t>& pkt) {
    if (pkt.size() < 5)     return { false, 0, {}, "too short" };
    if (pkt.front() != STX) return { false, 0, {}, "no STX" };
    if (pkt.back() != ETX)  return { false, 0, {}, "no ETX" };
    uint8_t len = pkt[1];
    if (pkt.size() != static_cast<size_t>(len) + 5)
        return { false, 0, {}, "len mismatch" };
    uint8_t cmd = pkt[2];
    std::vector<uint8_t> data(pkt.begin() + 3, pkt.begin() + 3 + len);
    uint8_t chk = pkt[3 + len];
    if (chk != calcChecksum(cmd, data))
        return { false, 0, {}, "checksum fail" };
    return { true, cmd, data, "ok" };
}

void printHex(const std::vector<uint8_t>& pkt) {
    std::cout << std::hex << std::uppercase << std::setfill('0');
    for (uint8_t b : pkt)
        std::cout << std::setw(2) << static_cast<int>(b) << ' ';
    std::cout << std::dec << '\n';
}

// ===== 3-B: 아두이노 펌웨어 어댑터 (chk = LEN ^ CMD) =====
std::vector<uint8_t> buildArduinoCmd(uint8_t cmd) {
    const uint8_t len = 0x01;
    uint8_t chk = static_cast<uint8_t>(len ^ cmd);
    return { STX, len, cmd, chk, ETX };   // 5바이트
}

// 센서 패킷: 펌웨어 sendPacket chk = len ^ cmd ^ data
ParseResult parseArduinoSensor(const std::vector<uint8_t>& pkt) {
    if (pkt.size() != 6)    return { false, 0, {}, "not 6 bytes" };
    if (pkt[0] != STX)      return { false, 0, {}, "no STX" };
    if (pkt[5] != ETX)      return { false, 0, {}, "no ETX" };
    uint8_t len = pkt[1];   // 0x01
    uint8_t cmd = pkt[2];
    uint8_t data = pkt[3];
    uint8_t chk = pkt[4];
    uint8_t calc = static_cast<uint8_t>(len ^ cmd ^ data);
    if (chk != calc)        return { false, 0, {}, "checksum fail" };
    return { true, cmd, { data }, "ok" };
}

// ===== 4단계: 연속 PWM 명령 (7바이트, chk = LEN ^ CMD ^ L ^ R) =====
std::vector<uint8_t> buildArduinoSetPWM(int8_t left, int8_t right) {
    const uint8_t len = 0x03;
    const uint8_t cmd = CMD_SET_PWM;
    uint8_t l = static_cast<uint8_t>(left);
    uint8_t r = static_cast<uint8_t>(right);
    uint8_t chk = static_cast<uint8_t>(len ^ cmd ^ l ^ r);
    return { STX, len, cmd, l, r, chk, ETX };   // 7바이트
}
