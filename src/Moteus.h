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

namespace mm = mjbots::moteus;

/// This is the primary interface to a moteus controller.  One
/// instance of this class should be created per controller that is
/// commanded or monitored.
///
/// The primary control functions each have 3 possible forms:
///
///  1. A "Make" variant which constructs a CanFdFrame to be used in a
///     later call to Transport::Cycle.
///
///  2. A "Set" variant which sends a command to the controller and
///     waits for a response in a blocking manner.
///
///  3. A "Poll" variant which sends a command and requires that the
///     user poll regularly to check for a response.
class Moteus {
 public:
  using CanFdFrame = mm::CanFdFrame;

  struct Options {
    // The ID of the servo to communicate with.
    int8_t id = 1;

    // The source ID to use for the commanding node (i.e. the host or
    // master).
    int8_t source = 0;

    mm::Query::Format query_format;
    mm::PositionMode::Format position_format;
    mm::VFOCMode::Format vfoc_format;
    mm::CurrentMode::Format current_format;
    mm::StayWithinMode::Format stay_within_format;

    // Use the given prefix for all CAN IDs.
    uint16_t can_prefix = 0x0000;

    // Disable BRS on outgoing frames.
    bool disable_brs = true;

    // Request the configured set of registers as a query with every
    // command.
    bool default_query = true;

    uint16_t min_rcv_wait_us = 800;
    uint16_t rx_extra_wait_us = 400;

    Options() {}
  };

  Moteus(ACAN2517FD& can_bus,
         const Options& options = {})
      : can_bus_(can_bus),
        options_(options) {
    mm::WriteCanData query_write(&query_frame_);
    mm::Query::Make(&query_write, options_.query_format);
  }

  struct Result {
    unsigned long timestamp = 0;
    CanFdFrame frame;
    mm::Query::Result values;
  };

  // The most recent result from any command.
  const Result& last_result() const { return last_result_; }


  /////////////////////////////////////////
  // Query

  CanFdFrame MakeQuery(const mm::Query::Format* format_override = nullptr) {
    return MakeFrame(mm::EmptyMode(), {}, {}, format_override);
  }

  bool Query(const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeQuery(query_override));
  }


  /////////////////////////////////////////
  // StopMode

  CanFdFrame MakeStop(const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::StopMode(), {}, {}, query_override);
  }

  bool SetStop(const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeStop(query_override));
  }


  /////////////////////////////////////////
  // BrakeMode


  CanFdFrame MakeBrake(const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::BrakeMode(), {}, {}, query_override);
  }

  bool SetBrake(const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeBrake(query_override));
  }


  /////////////////////////////////////////
  // PositionMode

  CanFdFrame MakePosition(const mm::PositionMode::Command& cmd,
                          const mm::PositionMode::Format* command_override = nullptr,
                          const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::PositionMode(),
                     cmd,
                     (command_override == nullptr ?
                      options_.position_format : *command_override),
                     query_override);
  }

  bool SetPosition(const mm::PositionMode::Command& cmd,
                   const mm::PositionMode::Format* command_override = nullptr,
                   const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(
        MakePosition(cmd, command_override, query_override));
  }


  /////////////////////////////////////////
  // VFOCMode

  CanFdFrame MakeVFOC(const mm::VFOCMode::Command& cmd,
                      const mm::VFOCMode::Format* command_override = nullptr,
                      const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::VFOCMode(),
                     cmd,
                     (command_override == nullptr ?
                      options_.vfoc_format : *command_override),
                     query_override);
  }

  bool SetVFOC(const mm::VFOCMode::Command& cmd,
               const mm::VFOCMode::Format* command_override = nullptr,
               const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeVFOC(cmd, command_override, query_override));
  }


  /////////////////////////////////////////
  // CurrentMode

  CanFdFrame MakeCurrent(const mm::CurrentMode::Command& cmd,
                         const mm::CurrentMode::Format* command_override = nullptr,
                         const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::CurrentMode(),
                     cmd,
                     (command_override == nullptr ?
                      options_.current_format : *command_override),
                     query_override);
  }

  bool SetCurrent(const mm::CurrentMode::Command& cmd,
                  const mm::CurrentMode::Format* command_override = nullptr,
                  const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeCurrent(cmd, command_override, query_override));
  }


  /////////////////////////////////////////
  // StayWithinMode

  CanFdFrame MakeStayWithin(const mm::StayWithinMode::Command& cmd,
                            const mm::StayWithinMode::Format* command_override = nullptr,
                            const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::StayWithinMode(),
                     cmd,
                     (command_override == nullptr ?
                      options_.stay_within_format : *command_override),
                     query_override);
  }

  bool SetStayWithin(const mm::StayWithinMode::Command& cmd,
                     const mm::StayWithinMode::Format* command_override = nullptr,
                     const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeStayWithin(cmd, command_override, query_override));
  }


  bool ExecuteSingleCommand(const mm::CanFdFrame& frame) {
    CANFDMessage can_message;
    can_message.id = frame.arbitration_id;
    can_message.ext = true;
    const bool desired_brs =
        (frame.brs == CanFdFrame::kDefault ? !options_.disable_brs :
         frame.brs == CanFdFrame::kForceOn ? true : false);

    if (frame.fdcan_frame == CanFdFrame::kDefault ||
        frame.fdcan_frame == CanFdFrame::kForceOn) {
      if (desired_brs) {
        can_message.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH;
      } else {
        can_message.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH;
      }
    } else {
      can_message.type = CANFDMessage::CAN_DATA;
    }
    can_message.len = frame.size;
    ::memcpy(&can_message.data[0], &frame.data[0], frame.size);
    can_message.ext = true;

    PadCan(&can_message);

    can_bus_.tryToSend(can_message);

    if (!frame.reply_required) { return false; }

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

      if (source != options_.id ||
          destination != options_.source ||
          can_prefix != options_.can_prefix) {
        continue;
      }

      last_result_.timestamp = now;

      auto& cf = last_result_.frame;
      cf.arbitration_id = rx_msg.id;
      cf.destination = destination;
      cf.source = source;
      cf.size = rx_msg.len;
      ::memcpy(&cf.data[0], &rx_msg.data[0], rx_msg.len);
      cf.can_prefix = can_prefix;

      if (rx_msg.type == CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH) {
        cf.brs = mm::CanFdFrame::kForceOn;
        cf.fdcan_frame = mm::CanFdFrame::kForceOn;
      } else if (rx_msg.type == CANFDMessage::CANFD_NO_BIT_RATE_SWITCH) {
        cf.brs = mm::CanFdFrame::kForceOff;
        cf.fdcan_frame = mm::CanFdFrame::kForceOn;
      } else {
        cf.brs = mm::CanFdFrame::kForceOff;
        cf.fdcan_frame = mm::CanFdFrame::kForceOff;
      }

      last_result_.values =
          mm::Query::Parse(&cf.data[0], cf.size);

      return true;
    }
  }

 private:
  enum ReplyMode {
    kNoReply,
    kReplyRequired,
  };

  CanFdFrame DefaultFrame(ReplyMode reply_mode = kReplyRequired) {
    CanFdFrame result;
    result.destination = options_.id;
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
  CanFdFrame MakeFrame(const CommandType&,
                       const typename CommandType::Command& cmd,
                       const typename CommandType::Format& fmt,
                       const mm::Query::Format* query_override = nullptr) {
    auto result = DefaultFrame(
        options_.default_query ? kReplyRequired : kNoReply);

    mm::WriteCanData write_frame(result.data, &result.size);
    CommandType::Make(&write_frame, cmd, fmt);

    if (query_override) {
      mm::Query::Make(&write_frame, *query_override);
    } else if (options_.default_query) {
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
  const Options options_;

  Result last_result_;
  mm::CanData query_frame_;
};
