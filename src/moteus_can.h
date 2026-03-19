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
/// Provides a fallback CANFDMessage definition for platforms that do
/// not use an ACAN library, plus common CAN-FD DLC utilities.

#pragma once

#include <stdint.h>
#include <string.h>

// If an ACAN library (ACAN2517FD, ACAN_T4, etc.) has already been
// included, it will have defined GENERIC_CANFD_MESSAGE_DEFINED and
// provided CANFDMessage.  We only define our own if that hasn't
// happened.
#ifndef GENERIC_CANFD_MESSAGE_DEFINED
#define GENERIC_CANFD_MESSAGE_DEFINED

class CANFDMessage {
 public:
  typedef enum : uint8_t {
    CAN_REMOTE,
    CAN_DATA,
    CANFD_NO_BIT_RATE_SWITCH,
    CANFD_WITH_BIT_RATE_SWITCH
  } Type;

  CANFDMessage()
      : id(0),
        ext(false),
        type(CANFD_WITH_BIT_RATE_SWITCH),
        idx(0),
        len(0),
        data() {}

  uint32_t id;
  bool ext;
  Type type;
  uint8_t idx;
  uint8_t len;
  union {
    uint64_t data64   [ 8];
    int64_t  data_s64 [ 8];
    uint32_t data32   [16];
    int32_t  data_s32 [16];
    float    dataFloat[16];
    uint16_t data16   [32];
    int16_t  data_s16 [32];
    int8_t   data_s8  [64];
    uint8_t  data     [64];
  };
};

#endif  // GENERIC_CANFD_MESSAGE_DEFINED

// CAN-FD DLC utilities — always defined regardless of which CAN
// library is in use.

namespace mjbots {
namespace moteus {

/// Round a byte count up to the next valid CAN-FD frame size.
/// Valid sizes: 0-8, 12, 16, 20, 24, 32, 48, 64.
inline int8_t RoundUpDlc(int8_t size) {
  if (size <= 0) { return 0; }
  if (size <= 1) { return 1; }
  if (size <= 2) { return 2; }
  if (size <= 3) { return 3; }
  if (size <= 4) { return 4; }
  if (size <= 5) { return 5; }
  if (size <= 6) { return 6; }
  if (size <= 7) { return 7; }
  if (size <= 8) { return 8; }
  if (size <= 12) { return 12; }
  if (size <= 16) { return 16; }
  if (size <= 20) { return 20; }
  if (size <= 24) { return 24; }
  if (size <= 32) { return 32; }
  if (size <= 48) { return 48; }
  if (size <= 64) { return 64; }
  return 0;
}

/// Pad a CANFDMessage to the next valid CAN-FD DLC boundary.
/// Padding bytes are filled with 0x50 (NOP in the moteus multiplex
/// protocol).
inline void PadCan(CANFDMessage* msg) {
  const auto new_size = RoundUpDlc(msg->len);
  for (int8_t i = msg->len; i < new_size; i++) {
    msg->data[i] = 0x50;
  }
  msg->len = new_size;
}

}
}
