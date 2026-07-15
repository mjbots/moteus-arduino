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

// Compile test — not intended to run on hardware.
//
// MoteusController is a class template, so its member functions are
// only type-checked when instantiated.  This sketch forces every
// member function of the library to compile AND link for each
// transport available on this platform.  ExerciseMoteusApi (in
// compile_test_common.h) uses only the public spellings a sketch
// would use (e.g. Moteus::OutputExact::Command).
//
// The API calls are made behind a volatile flag that is always false
// at runtime, so everything must survive the linker's dead code
// elimination — this catches link errors (e.g. missing HAL modules),
// not just compile errors.

#if defined(TEENSYDUINO)
#include <MoteusTeensy.h>
#elif defined(ARDUINO_ARCH_STM32)
#include <MoteusStm32Fdcan.h>
#else
#include <MoteusAcan2517fd.h>
#endif

#include <MoteusUart.h>

#include "compile_test_common.h"

// A serial port for the UART transport.  On STM32 the Serial1
// instance is only defined when the variant enables it, so
// construct our own port there.
#if defined(ARDUINO_ARCH_STM32)
HardwareSerial test_serial(PA10, PA9);
using TestSerial = HardwareSerial;
#else
auto& test_serial = Serial1;
using TestSerial = decltype(Serial1);
#endif

// Explicit instantiation forces compilation of every member
// function, even those ExerciseMoteusApi might miss.
template class MoteusController<MoteusUart<TestSerial>>;
template class MoteusUart<TestSerial>;

// The platform's native CAN transport.
#if defined(TEENSYDUINO)
template class MoteusController<MoteusTeensyCanFD>;
ACAN_T4FD_Settings can_settings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD can_bus(ACAN_T4::can3, can_settings);
#elif defined(ARDUINO_ARCH_STM32)
template class MoteusController<MoteusStm32FdCan>;
MoteusStm32FdCan can_bus;
#else
template class MoteusController<ACAN2517FD>;
ACAN2517FD can_bus(17, SPI, 7);
#endif

Moteus moteus(can_bus);

// The UART transport, available everywhere.
MoteusUart<TestSerial> uart_bus(test_serial);
MoteusController<MoteusUart<TestSerial>> uart_moteus(uart_bus);

// Always false at runtime, but the compiler must assume the calls
// happen, so the linker has to resolve everything they reference.
volatile bool exercise_api = false;

void setup() {}

void loop() {
  if (exercise_api) {
    ExerciseMoteusApi(moteus);
    ExerciseMoteusApi(uart_moteus);
  }
}
