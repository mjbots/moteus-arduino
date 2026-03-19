// Example: bare-metal STM32 usage of the moteus library (no Arduino framework).
//
// This file demonstrates how to use MoteusController with the STM32 HAL
// directly.  It is not an Arduino sketch (.ino) and will not appear in the
// Arduino IDE examples menu.
//
// To use this in your own project:
//   1. Add the moteus-arduino src/ directory to your include path
//   2. Implement moteus_micros() and moteus_delay_ms() for your platform
//   3. Initialize your FDCAN peripheral and pass the handle to MoteusStm32FdCan
//
// This example assumes a Nucleo-G474RE with FDCAN1 on PB8 (RX) / PB9 (TX)
// and uses the default begin() configuration.  Adapt GPIO and timing for
// your board.

#include "MoteusStm32Fdcan.h"

// ----- Platform timing functions required by Moteus.h ---------------------
//
// moteus_micros() must return a monotonically increasing microsecond count.
// Wrapping is fine — the library uses unsigned subtraction for timeouts.
//
// moteus_delay_ms() must block for approximately the given number of
// milliseconds.  Only needed if SetPositionWaitComplete() is used.

uint32_t moteus_micros() {
  // HAL_GetTick() returns milliseconds.  For true microsecond resolution,
  // use the DWT cycle counter or a dedicated hardware timer.
  return HAL_GetTick() * 1000;
}

void moteus_delay_ms(uint32_t ms) {
  HAL_Delay(ms);
}

// ----- CAN and moteus objects ---------------------------------------------

MoteusStm32FdCan canBus;

// Options must be set before constructing the controller.
static auto make_options() {
  MoteusController<MoteusStm32FdCan>::Options options;
  options.id = 1;
  return options;
}

MoteusController<MoteusStm32FdCan> moteus1(canBus, make_options());

// ----- Application --------------------------------------------------------

int main() {
  HAL_Init();

  // Configure system clocks, GPIO, etc. for your board here.
  // SystemClock_Config();

  // Initialize FDCAN1 with default board settings (1 Mbps, FD, no BRS).
  if (!canBus.begin()) {
    // Handle error — blink LED, halt, etc.
    while (true) {}
  }

  // Clear any faults.
  moteus1.SetStop();

  uint32_t next_send = HAL_GetTick();

  while (true) {
    const uint32_t now = HAL_GetTick();
    if (now < next_send) { continue; }
    next_send += 20;  // 50 Hz

    MoteusController<MoteusStm32FdCan>::PositionMode::Command cmd;
    cmd.position = NaN;
    cmd.velocity = 0.1;

    moteus1.SetPosition(cmd);

    // Access the most recent query result:
    // const auto& result = moteus1.last_result().values;
    // result.position, result.velocity, result.torque, etc.
  }
}
