// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
///
/// UART transport adapter for moteus.  This translates between
/// CANFDMessage and the fdcanusb text protocol over a serial port,
/// allowing MoteusController to communicate with moteus via a direct
/// UART connection.
///
/// Usage with Arduino:
///
///   #include <MoteusUart.h>
///   #include <Moteus.h>
///
///   MoteusUart<HardwareSerial> uart_bus(Serial1);
///   MoteusController<MoteusUart<HardwareSerial>> controller(uart_bus);
///
/// The SerialPort template parameter must provide:
///   int available()
///   int read()
///   size_t write(const uint8_t*, size_t)

#pragma once

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "moteus_can.h"

#ifdef ARDUINO
#include <Arduino.h>
inline uint32_t moteus_uart_micros() { return ::micros(); }
#else
extern uint32_t moteus_micros();
inline uint32_t moteus_uart_micros() { return moteus_micros(); }
#endif

template <typename SerialPort>
class MoteusUart {
 public:
  struct Options {
    // Default baud rate for UART communication with moteus.
    long baudrate = 921600;

    // Maximum number of retry attempts when waiting for OK.
    int max_retries = 3;

    // Timeout in microseconds for waiting for an OK response.
    uint32_t ok_timeout_us = 200000;

    Options() {}
  };

  MoteusUart(SerialPort& serial, const Options& options = {})
      : serial_(serial),
        options_(options) {
  }

  /// Call once in setup() to initialize the serial port.
  void begin() {
    serial_.begin(options_.baudrate);
  }

  /// Read any available serial data into the line buffer.
  void poll() {
    while (serial_.available() > 0) {
      const int c = serial_.read();
      if (c < 0) { break; }

      if (c == '\r' || c == '\n') {
        if (line_pos_ > 0) {
          line_buffer_[line_pos_] = '\0';
          ProcessLine(line_buffer_, line_pos_);
          line_pos_ = 0;
        }
        continue;
      }

      if (line_pos_ < static_cast<int>(sizeof(line_buffer_)) - 1) {
        line_buffer_[line_pos_++] = static_cast<char>(c);
      } else {
        // Line too long, discard.
        line_pos_ = 0;
      }
    }
  }

  /// Returns true if a received CAN frame is available.
  bool available() {
    return rcv_available_;
  }

  /// Copy the received frame into @p msg and clear the available flag.
  bool receive(CANFDMessage& msg) {
    if (!rcv_available_) { return false; }
    msg = rcv_message_;
    rcv_available_ = false;
    return true;
  }

  /// Format and send a CAN frame as a text command, then wait for OK.
  bool tryToSend(const CANFDMessage& msg) {
    for (int attempt = 0; attempt <= options_.max_retries; attempt++) {
      char buf[256];
      int pos = FormatSendLine(msg, buf, sizeof(buf));
      serial_.write(reinterpret_cast<const uint8_t*>(buf), pos);

      if (WaitForOk()) {
        return true;
      }
    }
    return false;
  }

 private:
  static int ParseHexNybble(char c) {
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    return -1;
  }

  static uint8_t ComputeCrc8(const char* data, size_t len) {
    static constexpr uint8_t kCrc8Table[16] = {
        0x00, 0x97, 0xb9, 0x2e, 0xe5, 0x72, 0x5c, 0xcb,
        0x5d, 0xca, 0xe4, 0x73, 0xb8, 0x2f, 0x01, 0x96,
    };
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
      const uint8_t b = static_cast<uint8_t>(data[i]);
      crc = kCrc8Table[((crc >> 4) ^ (b >> 4)) & 0x0f] ^ (crc << 4);
      crc = kCrc8Table[((crc >> 4) ^ (b & 0x0f)) & 0x0f] ^ (crc << 4);
    }
    return crc;
  }

  bool ValidateAndStripChecksum(char* line, int* len) {
    // Find the last '*' in the line.
    int star_pos = -1;
    for (int i = *len - 1; i >= 0; i--) {
      if (line[i] == '*') {
        star_pos = i;
        break;
      }
    }

    if (star_pos < 0 || star_pos + 3 > *len) {
      // No checksum found - reject.
      return false;
    }

    const int hi = ParseHexNybble(line[star_pos + 1]);
    const int lo = ParseHexNybble(line[star_pos + 2]);
    if (hi < 0 || lo < 0) {
      return false;
    }

    const uint8_t claimed = static_cast<uint8_t>((hi << 4) | lo);
    const uint8_t computed = ComputeCrc8(line, star_pos);
    if (claimed != computed) {
      return false;
    }

    // Strip checksum and trailing spaces.
    int end = star_pos;
    while (end > 0 && line[end - 1] == ' ') { end--; }
    line[end] = '\0';
    *len = end;
    return true;
  }

  void ProcessLine(char* line, int len) {
    if (!ValidateAndStripChecksum(line, &len)) {
      return;
    }

    if (len == 2 && line[0] == 'O' && line[1] == 'K') {
      ok_received_ = true;
      return;
    }

    if (len >= 3 && line[0] == 'E' && line[1] == 'R' && line[2] == 'R') {
      return;
    }

    if (len >= 4 && line[0] == 'r' && line[1] == 'c' &&
        line[2] == 'v' && line[3] == ' ') {
      ParseRcvLine(line + 4, len - 4);
    }
  }

  void ParseRcvLine(const char* data, int len) {
    // Parse "XXXX YYYYYY [flags]"
    // Find the space between address and data.
    int space = -1;
    for (int i = 0; i < len; i++) {
      if (data[i] == ' ') { space = i; break; }
    }
    if (space < 0) { return; }

    // Parse arbitration ID.
    uint32_t arb_id = 0;
    for (int i = 0; i < space; i++) {
      const int n = ParseHexNybble(data[i]);
      if (n < 0) { return; }
      arb_id = (arb_id << 4) | n;
    }

    // Parse hex data.
    const char* hex_start = data + space + 1;
    int hex_end = len;
    // Find end of hex data (next space or end).
    for (int i = space + 1; i < len; i++) {
      if (data[i] == ' ') { hex_end = i; break; }
    }

    const int hex_len = hex_end - (space + 1);
    if (hex_len < 0 || hex_len > 128) { return; }

    rcv_message_ = CANFDMessage();
    rcv_message_.id = arb_id;
    rcv_message_.ext = true;
    rcv_message_.len = hex_len / 2;

    for (int i = 0; i < hex_len; i += 2) {
      const int hi = ParseHexNybble(hex_start[i]);
      const int lo = ParseHexNybble(hex_start[i + 1]);
      if (hi < 0 || lo < 0) { return; }
      rcv_message_.data[i / 2] = static_cast<uint8_t>((hi << 4) | lo);
    }

    // Parse flags.
    for (int i = hex_end; i < len; i++) {
      switch (data[i]) {
        case 'b': rcv_message_.type = CANFDMessage::CAN_DATA; break;
        case 'B': rcv_message_.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH; break;
        case 'F': rcv_message_.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH; break;
        default: break;
      }
    }

    rcv_available_ = true;
  }

  int FormatSendLine(const CANFDMessage& msg, char* buf, int capacity) {
    int pos = snprintf(buf, capacity, "can send %04x ",
                       static_cast<unsigned>(msg.id));

    const int dlc = mjbots::moteus::RoundUpDlc(msg.len);
    for (int i = 0; i < msg.len && pos + 2 < capacity; i++) {
      const char hex[] = "0123456789abcdef";
      buf[pos++] = hex[(msg.data[i] >> 4) & 0xf];
      buf[pos++] = hex[msg.data[i] & 0xf];
    }
    for (int i = msg.len; i < dlc && pos + 2 < capacity; i++) {
      buf[pos++] = '5';
      buf[pos++] = '0';
    }

    if (pos + 5 < capacity) {
      buf[pos++] = ' ';
      const uint8_t crc = ComputeCrc8(buf, pos);
      buf[pos++] = '*';
      const char hex[] = "0123456789ABCDEF";
      buf[pos++] = hex[(crc >> 4) & 0xf];
      buf[pos++] = hex[crc & 0xf];
    }

    buf[pos++] = '\n';
    return pos;
  }

  bool WaitForOk() {
    ok_received_ = false;
    const auto start = moteus_uart_micros();
    const auto end = start + options_.ok_timeout_us;

    while (true) {
      const auto now = moteus_uart_micros();
      if (static_cast<long>(now - end) > 0) {
        return false;
      }
      poll();
      if (ok_received_) {
        ok_received_ = false;
        return true;
      }
    }
  }

  SerialPort& serial_;
  const Options options_;

  char line_buffer_[512] = {};
  int line_pos_ = 0;

  CANFDMessage rcv_message_;
  bool rcv_available_ = false;

  bool ok_received_ = false;
};
