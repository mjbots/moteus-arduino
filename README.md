# moteus library for Arduino #

## Summary ##

This is a C++ library that provides a convenient API for controlling and monitoring an mjbots moteus brushless servo controller.  It supports multiple CAN-FD hardware backends:

* **MCP2517FD / MCP2518FD** external CAN-FD controllers via SPI (using the ACAN2517FD library)
* **Teensy 4.x** on-board CAN-FD hardware (using the ACAN_T4 library)
* **STM32 FDCAN** on-board peripherals on H7, G4, and G0 families (using the STM32 HAL)
* **UART** moteus controllers with sufficiently new firmware support a logical level TTL UART connection to monitor and control

* [![CI](https://github.com/mjbots/moteus-arduino/actions/workflows/ci.yml/badge.svg)](https://github.com/mjbots/moteus-arduino/actions/workflows/ci.yml)

## Getting started ##

Note that this library can only communicate and operate a moteus controller which has already been calibrated.  Calibration currently can not be performed via an Arduino and requires `moteus_tool` executing on a desktop computer with some CAN-FD adapter.  See the moteus [getting started guide](https://github.com/mjbots/moteus/blob/main/docs/getting_started.md#calibration) for details.  The only controllers sold by mjbots.com which are pre-calibrated are those included in development kits.

### Hardware ###

You will need a board with CAN-FD capability.  This can be:

* An Arduino with an external MCP2517FD or MCP2518FD CAN-FD controller (e.g., the [CANBed FD](https://www.longan-labs.cc/1030009.html))
* A Teensy 4.0 or 4.1 with a CAN-FD transceiver (e.g., TJA1051T/3) connected to CAN3 (pins 30/31)
* An STM32 board with a built-in FDCAN peripheral (e.g., Portenta H7, Nucleo-G474RE, Nucleo-G0B1RE) and a CAN-FD transceiver connected to the FDCAN TX/RX pins

The CAN-FD bus needs to be connected to moteus, typically this would be at least the CANL and CANH wires, and likely the ground as well.  For more than 2 or 3 controllers, separate 120 ohm termination resistors will be required on each end of the CAN bus.  Some Arduino CAN-FD shields, like the CANBed FD, have one termination resistor built in.

### Software ###

If you are using version 1.6.x or later of the Arduino software (IDE) you can use the Library Manager to install this library:

1. In the Arduino IDE, open the "Sketch" menu, select "Include Library", then "Manage Libraries...".
2. Search for "moteus"
3. Click the moteus entry in the list.
4. Click "Install".

If this does not work, you can manually install the library:

1. Download the [latest release archive from GitHub](https://github.com/mjbots/moteus-arduino/releases) and decompress it
2. Rename the folder "moteus-arduino" to "moteus"
3. Drag the "moteus" folder into the "libraries" directory inside your Arduino sketchbook directory.  You can view your sketchbook location by opening the "File" menu and selecting "Preferences" in the Arduino IDE.  If there is not already a "libraries" folder in that location, you should make the folder yourself.
4. After installing the library, restart the Arduino IDE.

### Platform-specific libraries ###

Depending on your hardware, you will also need to install one of the following CAN-FD libraries:

| Platform | Required library | Install via |
|----------|-----------------|-------------|
| MCP2517FD / MCP2518FD | [ACAN2517FD](https://github.com/pierremolinaro/acan2517FD) | Arduino Library Manager |
| Teensy 4.x | [ACAN_T4](https://github.com/pierremolinaro/acan-t4) | Arduino Library Manager |
| STM32 (H7, G4, G0) | None (uses built-in STM32 HAL) | — |

Then include the appropriate header in your sketch:

```cpp
#include <MoteusAcan2517fd.h>   // for MCP2517FD / MCP2518FD
#include <MoteusTeensy.h>       // for Teensy 4.x
#include <MoteusStm32Fdcan.h>   // for STM32 H7, G4, G0
```

Each header provides a `Moteus` type alias so the rest of your sketch code is the same regardless of platform.

### Custom CAN-FD hardware ###

To use CAN-FD hardware not listed above, include `<Moteus.h>` directly and provide a class with `poll()`, `available()`, `receive(CANFDMessage&)`, and `tryToSend(const CANFDMessage&)` methods.  Then instantiate `MoteusController<YourClass>`.

### Bare-metal STM32 (no Arduino framework) ###

The library can be used without the Arduino framework on STM32 projects.  You need to provide two platform timing functions that the library calls internally:

```cpp
// Must return a monotonically increasing microsecond count (wrapping is OK).
uint32_t moteus_micros() {
    return HAL_GetTick() * 1000;  // or use DWT/timer for true µs resolution
}

// Must block for approximately the given number of milliseconds.
void moteus_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
```

Then include `MoteusStm32Fdcan.h` and use `MoteusController<MoteusStm32FdCan>` directly.  Diagnostic channel methods (which depend on the Arduino `String` class) are not available in bare-metal builds.  See `examples/Stm32BareMetal/Stm32BareMetal.cpp` for a complete example.

## Examples ##

Several examples are available.  You can access them from the Arduino IDE by opening the "File" menu, selecting "Examples", and then selecting "moteus".

* **BasicControl** — MCP2517FD: sine wave motion on controller 1, brake on controller 2
* **DiagnosticProtocol** — MCP2517FD: reading and writing configuration values
* **WaitComplete** — MCP2517FD: waiting for trajectory completion
* **TeensyBasicControl** — Teensy 4.x: same as BasicControl using on-board CAN3
* **Stm32BasicControl** — STM32 H7/G4/G0: same as BasicControl using on-board FDCAN1
* **Stm32BareMetal** — STM32 without Arduino framework: bare-metal HAL usage

## Documentation ##

For complete documentation, see [docs/reference.md](the reference documentation).
