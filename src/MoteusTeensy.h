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
/// Convenience header for using moteus with Teensy 4.x on-board
/// CAN-FD hardware via the ACAN_T4 library.
///
/// Usage:
///   #include <moteus_teensy.h>
///
/// Requires the ACAN_T4 library to be installed.

#pragma once

#include <ACAN_T4.h>
#include "Moteus.h"

/// Adapter that wraps an ACAN_T4 bus instance to provide the
/// interface expected by MoteusController.
class MoteusTeensyCanFD {
 public:
  /// Construct with a reference to an ACAN_T4 bus (e.g., ACAN_T4::can3)
  /// and the FD settings used to initialize it.  The settings are stored
  /// so that poll() can reinitialize the controller on bus-off recovery.
  ///
  /// The caller must call bus.beginFD(settings) in their setup() before
  /// using this adapter.
  MoteusTeensyCanFD(ACAN_T4& bus, const ACAN_T4FD_Settings& settings)
      : bus_(bus), settings_(settings) {}

  void poll() {
    // ACAN_T4 is interrupt-driven and does not need polling for
    // message processing.  However, it does not auto-recover from
    // bus-off, so we check and reinitialize if needed.
    if (bus_.controllerState() == ACAN_T4::kBusOff) {
      bus_.end();
      bus_.beginFD(settings_);
    }
  }

  bool available() { return bus_.availableFD(); }

  bool receive(CANFDMessage& msg) { return bus_.receiveFD(msg); }

  bool tryToSend(const CANFDMessage& msg) {
    return bus_.tryToSendReturnStatusFD(msg) == 0;
  }

 private:
  ACAN_T4& bus_;
  ACAN_T4FD_Settings settings_;
};

using Moteus = MoteusController<MoteusTeensyCanFD>;
