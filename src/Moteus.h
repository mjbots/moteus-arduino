// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#pragma once

#include "moteus_protocol.h"

struct Command2 {
  int8_t destination = 1;
  int8_t source = 0;
  bool reply_required = false;
  uint8_t data[64] = {};
  uint8_t size = 0;

  uint16_t can_prefix = 0x0000;  // A 13 bit CAN prefix

  // If true, then the ID used is not calculated from destination and
  // source, but is instead determined directly from arbitration_id.
  bool raw = false;

  uint32_t arbitration_id = 0;
  int8_t bus = 0;

  enum Toggle {
    kDefault,
    kForceOff,
    kForceOn,
  };

  Toggle brs = kDefault;
  Toggle fdcan_frame = kDefault;
};

class Moteus {
 public:
  using Command = mjbots::moteus::Command;

  struct Options {
    mjbots::moteus::Query::Format query_format;
    // mjbots::moteus::PositionMode::Format position_format;

    bool disable_brs = true;
    int source = 0;
    uint16_t can_prefix = 0x0000;
    bool default_query = true;

    uint16_t min_rcv_wait_us = 800;
    uint16_t rx_extra_wait_us = 400;

    Options() {}
  };

  Moteus(ACAN2517FD& can_bus)
      : can_bus_(can_bus) {
  }

  void Begin(int8_t id, const Options& options) {
    id_ = id;
    options_ = options;

    mjbots::moteus::WriteCanFrame query_write(&query_frame_);
    mjbots::moteus::Query::Make(&query_write, options_.query_format);
  }


  struct Result {
    unsigned long timestamp = 0;
    Command can_frame;
    mjbots::moteus::Query::Result query_result;
  };

  const Result& last_result() const { return last_result_; }

  Command MakeStop() {
    return MakeCommand(mjbots::moteus::StopMode(), {}, {});
  }

  bool SetStop() {
    return ExecuteSingleCommand(MakeStop());
  }

  bool ExecuteSingleCommand(const Command& cmd) {
    CANFDMessage can_message;
    can_message.id = cmd.arbitration_id;
    can_message.ext = true;
    const bool desired_brs =
        (cmd.brs == Command::kDefault ? !options_.disable_brs :
         cmd.brs == Command::kForceOn ? true : false);

    if (cmd.fdcan_frame == Command::kDefault ||
        cmd.fdcan_frame == Command::kForceOn) {
      if (desired_brs) {
        can_message.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH;
      } else {
        can_message.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH;
      }
    } else {
      can_message.type = CANFDMessage::CAN_DATA;
    }
    can_message.len = cmd.size;
    ::memcpy(&can_message.data[0], &cmd.data[0], cmd.size);
    can_message.ext = true;

    PadCan(&can_message);

    can_bus_.tryToSend(can_message);

    if (!cmd.reply_required) { return false; }

    auto start = micros();
    auto end = start + options_.min_rcv_wait_us;

    while (true) {
      const auto now = micros();
      const auto delta = static_cast<long>(now - end);
      if (delta > 0) {
        // We timed out.
        return false;
      }

      if (!can_bus_.available()) { continue; }

      CANFDMessage rx_msg;
      can_bus_.receive(rx_msg);

      const int8_t source = (rx_msg.id >> 8) & 0x7f;
      const int8_t destination = (rx_msg.id & 0x7f);
      const uint16_t can_prefix = (rx_msg.id >> 16);

      if (source != id_ ||
          destination != options_.source ||
          can_prefix != options_.can_prefix) {
        continue;
      }

      last_result_.timestamp = now;

      auto& cf = last_result_.can_frame;
      cf.arbitration_id = rx_msg.id;
      cf.destination = destination;
      cf.source = source;
      cf.size = rx_msg.len;
      ::memcpy(&cf.data[0], &rx_msg.data[0], rx_msg.len);
      cf.can_prefix = can_prefix;

      if (rx_msg.type == CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH) {
        cf.brs = Command::kForceOn;
        cf.fdcan_frame = Command::kForceOn;
      } else if (rx_msg.type == CANFDMessage::CANFD_NO_BIT_RATE_SWITCH) {
        cf.brs = Command::kForceOff;
        cf.fdcan_frame = Command::kForceOn;
      } else {
        cf.brs = Command::kForceOff;
        cf.fdcan_frame = Command::kForceOff;
      }

      last_result_.query_result =
          mjbots::moteus::Query::Parse(&cf.data[0], cf.size);

      return true;
    }
  }

 private:
  enum ReplyMode {
    kNoReply,
    kReplyRequired,
  };

  Command2 ReturnCommand() {
    return {};
  }

  Command DefaultCommand(ReplyMode reply_mode = kReplyRequired) {
    Command result;
    result.destination = id_;
    result.reply_required = (reply_mode == kReplyRequired);

    result.arbitration_id =
        (result.destination) |
        (result.source << 8) |
        (result.reply_required ? 0x8000 : 0x0000) |
        (static_cast<uint32_t>(options_.can_prefix) << 16);
    result.bus = 0;

    return result;
  }

  template <typename CommandType>
  Command MakeCommand(const CommandType&,
                      const typename CommandType::Command& cmd,
                      const typename CommandType::Format& fmt) {
    auto result = DefaultCommand(
        options_.default_query ? kReplyRequired : kNoReply);

    mjbots::moteus::WriteCanFrame write_frame(result.data, &result.size);
    CommandType::Make(&write_frame, cmd, fmt);

    if (options_.default_query) {
      ::memcpy(&result.data[result.size],
               &query_frame_.data[0],
               query_frame_.size);
      result.size += query_frame_.size;
    }

    return result;
  }

  static int8_t RoundUpDlc(size_t size) {
    if (size == 0) { return 0; }
    if (size == 1) { return 1; }
    if (size == 2) { return 2; }
    if (size == 3) { return 3; }
    if (size == 4) { return 4; }
    if (size == 5) { return 5; }
    if (size == 6) { return 6; }
    if (size == 7) { return 7; }
    if (size == 8) { return 8; }
    if (size <= 12) { return 12; }
    if (size <= 16) { return 16; }
    if (size <= 20) { return 20; }
    if (size <= 24) { return 24; }
    if (size <= 32) { return 32; }
    if (size <= 48) { return 48; }
    if (size <= 64) { return 64; }
    return 0;
  }

  static void PadCan(CANFDMessage* msg) {
    const auto new_size = RoundUpDlc(msg->len);
    for (int8_t i = msg->len; i < new_size; i++) {
      msg->data[i] = 0x50;
    }
    msg->len = new_size;
  }

  ACAN2517FD& can_bus_;
  int8_t id_;
  Options options_;

  Result last_result_;
  mjbots::moteus::CanFrame query_frame_;
};
