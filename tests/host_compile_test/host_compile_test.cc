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
/// Host (non-Arduino) compile test.  MoteusController is a class
/// template, so member functions are only type-checked when
/// instantiated.  The explicit instantiations below force every
/// member of every class template in the library to compile.  Nothing
/// here is intended to run.

#include "Moteus.h"
#include "MoteusUart.h"

#include "../arduino_compile_test/compile_test_common.h"

// Platform timing functions required on non-Arduino builds.
uint32_t moteus_micros() { return 0; }
void moteus_delay_ms(uint32_t) {}

// A minimal serial port satisfying the MoteusUart interface.
class FakeSerial {
 public:
  void begin(long) {}
  int available() { return 0; }
  int read() { return -1; }
  size_t write(const uint8_t*, size_t len) { return len; }
};

// Force compilation of every member function.
template class MoteusUart<FakeSerial>;
template class MoteusController<MoteusUart<FakeSerial>>;

// Force compilation of the user-facing API surface.
template void ExerciseMoteusApi(MoteusController<MoteusUart<FakeSerial>>&);

int main() { return 0; }
