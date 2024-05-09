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
///  3. A "Begin" variant which sends a command and requires that the
///     user call Poll() regularly to check for a response, and then
///     retrieve that response from Moteus::last_result().
class Moteus {
 public:
  using CanFdFrame = mm::CanFdFrame;
  static constexpr int kDiagnosticTimeoutUs = 4000;

  using Query = mm::Query;
  using PositionMode = mm::PositionMode;
  using VFOCMode = mm::VFOCMode;
  using CurrentMode = mm::CurrentMode;
  using StayWithinMode = mm::StayWithinMode;

  static constexpr mm::Resolution kInt8 = mm::Resolution::kInt8;
  static constexpr mm::Resolution kInt16 = mm::Resolution::kInt16;
  static constexpr mm::Resolution kInt32 = mm::Resolution::kInt32;
  static constexpr mm::Resolution kFloat = mm::Resolution::kFloat;

  struct Options {
    // The ID of the servo to communicate with.
    int8_t id = 1;

    // The source ID to use for the commanding node (i.e. the host or
    // master).
    int8_t source = 0;

    mm::Query::Format query_format;

    // Use the given prefix for all CAN IDs.
    uint16_t can_prefix = 0x0000;

    // Disable BRS on outgoing frames.
    bool disable_brs = true;

    // Request the configured set of registers as a query with every
    // command.
    bool default_query = true;

    uint16_t min_rcv_wait_us = 2000;

    Options() {}
  };

  Moteus(ACAN2517FD& can_bus,
         const Options& options = {})
      : can_bus_(can_bus),
        options_(options) {
    mm::CanData can_data;
    mm::WriteCanData query_write(&can_data);
    mm::Query::Make(&query_write, options_.query_format);

    query_size_ = can_data.size;
    query_data_ = reinterpret_cast<char*>(realloc(query_data_, query_size_));
    ::memcpy(query_data_, &can_data.data[0], query_size_);
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
    return MakeFrame(mm::EmptyMode(), {}, {},
                     format_override == nullptr ?
                     &options_.query_format : format_override);
  }

  bool SetQuery(const mm::Query::Format* query_override = nullptr) {
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

  void BeginStop(const mm::Query::Format* query_override = nullptr) {
    BeginSingleCommand(MakeStop(query_override));
  }


  /////////////////////////////////////////
  // BrakeMode


  CanFdFrame MakeBrake(const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::BrakeMode(), {}, {}, query_override);
  }

  bool SetBrake(const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeBrake(query_override));
  }

  void BeginBrake(const mm::Query::Format* query_override = nullptr) {
    BeginSingleCommand(MakeBrake(query_override));
  }


  /////////////////////////////////////////
  // PositionMode

  CanFdFrame MakePosition(const mm::PositionMode::Command& cmd,
                          const mm::PositionMode::Format* command_override = nullptr,
                          const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::PositionMode(),
                     cmd,
                     (command_override == nullptr ?
                      mm::PositionMode::Format() : *command_override),
                     query_override);
  }

  bool SetPosition(const mm::PositionMode::Command& cmd,
                   const mm::PositionMode::Format* command_override = nullptr,
                   const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(
        MakePosition(cmd, command_override, query_override));
  }

  void BeginPosition(const mm::PositionMode::Command& cmd,
                     const mm::PositionMode::Format* command_override = nullptr,
                     const mm::Query::Format* query_override = nullptr) {
    BeginSingleCommand(
        MakePosition(cmd, command_override, query_override));
  }

  bool SetPositionWaitComplete(const mm::PositionMode::Command& cmd,
                               double period_s,
                               const mm::PositionMode::Format* command_override = nullptr,
                               const mm::Query::Format* query_override = nullptr) {
    mm::Query::Format query_format =
        query_override == nullptr ? options_.query_format : *query_override;
    query_format.trajectory_complete = kInt8;

    // The query returned from a mode change will always report the
    // previous state.  Thus we need to have at least 2 responses
    // before we have a valid trajectory complete flag.
    int count = 2;
    while (true) {
      const bool got_result = SetPosition(cmd, command_override, &query_format);
      if (got_result) {
        count = count - 1;
        if (count < 0) { count = 0; }
      }
      if (count == 0 &&
          got_result &&
          last_result_.values.trajectory_complete) {
        return true;
      }

      delay(static_cast<unsigned long>(period_s * 1000));
    }
  }


  /////////////////////////////////////////
  // VFOCMode

  CanFdFrame MakeVFOC(const mm::VFOCMode::Command& cmd,
                      const mm::VFOCMode::Format* command_override = nullptr,
                      const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::VFOCMode(),
                     cmd,
                     (command_override == nullptr ?
                      mm::VFOCMode::Format() : *command_override),
                     query_override);
  }

  bool SetVFOC(const mm::VFOCMode::Command& cmd,
               const mm::VFOCMode::Format* command_override = nullptr,
               const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeVFOC(cmd, command_override, query_override));
  }

  void BeginVFOC(const mm::VFOCMode::Command& cmd,
                 const mm::VFOCMode::Format* command_override = nullptr,
                 const mm::Query::Format* query_override = nullptr) {
    BeginSingleCommand(MakeVFOC(cmd, command_override, query_override));
  }


  /////////////////////////////////////////
  // CurrentMode

  CanFdFrame MakeCurrent(const mm::CurrentMode::Command& cmd,
                         const mm::CurrentMode::Format* command_override = nullptr,
                         const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::CurrentMode(),
                     cmd,
                     (command_override == nullptr ?
                      mm::CurrentMode::Format() : *command_override),
                     query_override);
  }

  bool SetCurrent(const mm::CurrentMode::Command& cmd,
                  const mm::CurrentMode::Format* command_override = nullptr,
                  const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeCurrent(cmd, command_override, query_override));
  }

  void BeginCurrent(const mm::CurrentMode::Command& cmd,
                    const mm::CurrentMode::Format* command_override = nullptr,
                    const mm::Query::Format* query_override = nullptr) {
    BeginSingleCommand(MakeCurrent(cmd, command_override, query_override));
  }


  /////////////////////////////////////////
  // StayWithinMode

  CanFdFrame MakeStayWithin(const mm::StayWithinMode::Command& cmd,
                            const mm::StayWithinMode::Format* command_override = nullptr,
                            const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::StayWithinMode(),
                     cmd,
                     (command_override == nullptr ?
                      mm::StayWithinMode::Format() : *command_override),
                     query_override);
  }

  bool SetStayWithin(const mm::StayWithinMode::Command& cmd,
                     const mm::StayWithinMode::Format* command_override = nullptr,
                     const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeStayWithin(cmd, command_override, query_override));
  }

  void BeginStayWithin(const mm::StayWithinMode::Command& cmd,
                       const mm::StayWithinMode::Format* command_override = nullptr,
                       const mm::Query::Format* query_override = nullptr) {
    BeginSingleCommand(MakeStayWithin(cmd, command_override, query_override));
  }


  /////////////////////////////////////////
  // OutputNearest

  CanFdFrame MakeOutputNearest(const mm::OutputNearest::Command& cmd,
                               const mm::OutputNearest::Format* command_override = nullptr,
                               const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::OutputNearest(),
                     cmd,
                     (command_override == nullptr ?
                      mm::OutputNearest::Format() : *command_override),
                     query_override);
  }

  bool SetOutputNearest(const mm::OutputNearest::Command& cmd,
                        const mm::OutputNearest::Format* command_override = nullptr,
                        const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeOutputNearest(cmd, command_override, query_override));
  }

  void BeginOutputNearest(const mm::OutputNearest::Command& cmd,
                          const mm::OutputNearest::Format* command_override = nullptr,
                          const mm::Query::Format* query_override = nullptr) {
    BeginSingleCommand(MakeOutputNearest(cmd, command_override, query_override));
  }


  /////////////////////////////////////////
  // OutputExact

  CanFdFrame MakeOutputExact(const mm::OutputExact::Command& cmd,
                               const mm::OutputExact::Format* command_override = nullptr,
                               const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::OutputExact(),
                     cmd,
                     (command_override == nullptr ?
                      mm::OutputExact::Format() : *command_override),
                     query_override);
  }

  bool SetOutputExact(const mm::OutputExact::Command& cmd,
                      const mm::OutputExact::Format* command_override = nullptr,
                      const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeOutputExact(cmd, command_override, query_override));
  }

  void BeginOutputExact(const mm::OutputExact::Command& cmd,
                        const mm::OutputExact::Format* command_override = nullptr,
                        const mm::Query::Format* query_override = nullptr) {
    BeginSingleCommand(MakeOutputExact(cmd, command_override, query_override));
  }


  /////////////////////////////////////////
  // RequireReindex

  CanFdFrame MakeRequireReindex(const mm::RequireReindex::Command& cmd,
                                const mm::RequireReindex::Format* command_override = nullptr,
                               const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::RequireReindex(),
                     cmd,
                     (command_override == nullptr ?
                      mm::RequireReindex::Format() : *command_override),
                     query_override);
  }

  bool SetRequireReindex(const mm::RequireReindex::Command& cmd,
                         const mm::RequireReindex::Format* command_override = nullptr,
                         const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeRequireReindex(cmd, command_override, query_override));
  }

  void BeginRequireReindex(const mm::RequireReindex::Command& cmd,
                           const mm::RequireReindex::Format* command_override = nullptr,
                           const mm::Query::Format* query_override = nullptr) {
    BeginSingleCommand(MakeRequireReindex(cmd, command_override, query_override));
  }


  /////////////////////////////////////////
  // RecapturePositionVelocity

  CanFdFrame MakeRecapturePositionVelocity(const mm::RecapturePositionVelocity::Command& cmd,
                                const mm::RecapturePositionVelocity::Format* command_override = nullptr,
                               const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::RecapturePositionVelocity(),
                     cmd,
                     (command_override == nullptr ?
                      mm::RecapturePositionVelocity::Format() : *command_override),
                     query_override);
  }

  bool SetRecapturePositionVelocity(const mm::RecapturePositionVelocity::Command& cmd,
                                    const mm::RecapturePositionVelocity::Format* command_override = nullptr,
                                    const mm::Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeRecapturePositionVelocity(cmd, command_override, query_override));
  }

  void BeginRecapturePositionVelocity(const mm::RecapturePositionVelocity::Command& cmd,
                                      const mm::RecapturePositionVelocity::Format* command_override = nullptr,
                                      const mm::Query::Format* query_override = nullptr) {
    BeginSingleCommand(MakeRecapturePositionVelocity(cmd, command_override, query_override));
  }


  /////////////////////////////////////////
  // Diagnostic channel operations

  enum DiagnosticReplyMode {
    kExpectOK,
    kExpectSingleLine,
  };

  String DiagnosticCommand(const String& message_in,
                           DiagnosticReplyMode reply_mode = kExpectOK) {
    {
      String message = message_in + "\n";
      while (message.length() > 0) {
        const auto to_write = message.length() < 48u ? message.length() : 48u;
        mm::DiagnosticWrite::Command write;
        write.data = message.c_str();
        write.size = to_write;

        auto frame = DefaultFrame(kNoReply);
        mm::WriteCanData write_frame(frame.data, &frame.size);
        mm::DiagnosticWrite::Make(&write_frame, write, {});

        BeginSingleCommand(frame);
        message.remove(0, to_write);
      }
    }

    // Now we read until we get a complete response.
    String response;
    String current_line;

    const auto start = micros();
    auto end = start + kDiagnosticTimeoutUs;

    while (true) {
      {
        const auto now = micros();
        if (static_cast<long>(now - end) > 0) {
          return {};
        }
      }
      {
        mm::DiagnosticRead::Command read;
        auto frame = DefaultFrame(kReplyRequired);
        mm::WriteCanData write_frame(frame.data, &frame.size);
        mm::DiagnosticRead::Make(&write_frame, read, {});

        BeginSingleCommand(frame);
      }

      while (true) {
        if (![&]() {
          while (!Poll()) {
            const auto now = micros();
            if (static_cast<long>(now - end) > 0) {
              return false;
            }
          }
          return true;
        }()) {
          break;
        }

        {
          const auto parsed = mm::DiagnosticResponse::Parse(
              last_result_.frame.data, last_result_.frame.size);
          if (parsed.channel != 1) {
            // This must not have been for us after all.
            continue;
          }

          if (parsed.size) {
            // We got some data, so bump our timeout forward.
            end = start + kDiagnosticTimeoutUs;
          }

          // Sigh... older versions of Arduino have no ability to
          // construct a string from a pointer and length.  Guess we'll
          // emulate it.
          {
            String this_data;
            this_data.reserve(parsed.size);
            for (int8_t i = 0; i < parsed.size; i++) {
              this_data.concat(static_cast<char>(parsed.data[i]));
            }

            current_line.concat(this_data);
          }
        }

        auto find_newline = [&]() {
          const int cr = current_line.indexOf('\r');
          const int nl = current_line.indexOf('\n');
          const int first_newline =
              (cr < 0 ? nl :
               nl < 0 ? cr :
               nl < cr ? nl : cr);
          return first_newline;
        };
        int first_newline = -1;
        while ((first_newline = find_newline()) != -1) {
          String this_line = current_line.substring(0, first_newline);
          if (reply_mode == kExpectSingleLine) {
            return this_line;
          } else if (this_line == "OK") {
            return response;
          } else {
            response.concat(current_line.substring(0, first_newline + 1));
            current_line.remove(0, first_newline + 1);
          }
        }

        // We got a frame, so break out of our inner loop.
        break;
      }
    }
  }

  String SetDiagnosticRead(int channel = 1) {
    {
      mm::DiagnosticRead::Command read;
      read.channel = channel;
      auto frame = DefaultFrame(kReplyRequired);
      mm::WriteCanData write_frame(frame.data, &frame.size);
      mm::DiagnosticRead::Make(&write_frame, read, {});

      BeginSingleCommand(frame);
    }

    const auto start = micros();
    auto end = start + kDiagnosticTimeoutUs;

    while (true) {
      if (![&]() {
        while (!Poll()) {
          const auto now = micros();
          if (static_cast<long>(now - end) > 0) {
            return false;
          }
        }
        return true;
      }()) {
        return "";
      }

      const auto parsed = mm::DiagnosticResponse::Parse(
          last_result_.frame.data, last_result_.frame.size);
      if (parsed.channel != channel) {
        continue;
      }

      String result;
      result.reserve(parsed.size);
      for (int8_t i = 0; i < parsed.size; i++) {
        result.concat(static_cast<char>(parsed.data[i]));
      }
      return result;
    }
  }

  void SetDiagnosticFlush(int channel = 1) {
    while (true) {
      const auto response = SetDiagnosticRead(channel);
      if (response.length() == 0) { return; }
    }
  }

  /////////////////////////////////////////
  // Non-command related methods

  /// Look for a response to a previous command.  Return true if one
  /// has been received.  The parsed results can be seen in
  /// Moteus::last_result()
  bool Poll() {
    const auto now = micros();

    // Ensure any interrupts have been handled.
    can_bus_.poll();

    if (!can_bus_.available()) { return false; }

    CANFDMessage rx_msg;
    can_bus_.receive(rx_msg);

    const int8_t source = (rx_msg.id >> 8) & 0x7f;
    const int8_t destination = (rx_msg.id & 0x7f);
    const uint16_t can_prefix = (rx_msg.id >> 16);

    if (source != options_.id ||
        destination != options_.source ||
        can_prefix != options_.can_prefix) {
      return false;
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

  bool BeginSingleCommand(const mm::CanFdFrame& frame) {
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

    // To work even when the ACAN2517FD doesn't have functioning
    // interrupts, we will just poll it before and after attempting to
    // send our message.  This slows things down, but we're on an
    // Arduino, so who cares?
    can_bus_.poll();
    can_bus_.tryToSend(can_message);
    can_bus_.poll();

    return frame.reply_required;
  }

  bool ExecuteSingleCommand(const mm::CanFdFrame& frame) {
    const bool reply_required = BeginSingleCommand(frame);

    if (!reply_required) { return false; }

    auto start = micros();
    auto end = start + options_.min_rcv_wait_us;
    bool got_a_response = false;

    while (true) {
      const auto now = micros();
      const auto delta = static_cast<long>(now - end);
      if (delta > 0) {
        // We timed out.
        return false;
      }

      if (Poll()) {
        got_a_response = true;
      } else if (got_a_response) {
        // We both received a response, and have no more queued up.
        return true;
      }
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
        query_override != nullptr ? kReplyRequired :
        options_.default_query ? kReplyRequired : kNoReply);

    mm::WriteCanData write_frame(result.data, &result.size);
    CommandType::Make(&write_frame, cmd, fmt);

    if (query_override) {
      mm::Query::Make(&write_frame, *query_override);
    } else if (options_.default_query) {
      ::memcpy(&result.data[result.size],
               query_data_,
               query_size_);
      result.size += query_size_;
    }

    return result;
  }

  static int8_t RoundUpDlc(int8_t size) {
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
  char* query_data_ = nullptr;
  size_t query_size_ = 0;
};
