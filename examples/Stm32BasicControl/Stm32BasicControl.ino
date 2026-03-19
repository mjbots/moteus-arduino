//——————————————————————————————————————————————————————————————————————————————
// Demonstration of control and monitoring of 2 moteus controllers
// using an STM32 board with on-board FDCAN hardware.
//  * https://mjbots.com/products/moteus-r4-11
//
// Default configuration is for Portenta H7 + Hat Carrier:
//   FDCAN1, PB8 (RX), PH13 (TX), 1 Mbps, FD, BRS disabled.
//   120 ohm termination required at both ends of the bus.
//
// Controller ID 1 is moved through a sine wave pattern, while
// controller ID 2 just has a brake command sent.
// ——————————————————————————————————————————————————————————————————————————————

#include <MoteusStm32Fdcan.h>

//——————————————————————————————————————————————————————————————————————————————
//  CAN-FD configuration
//——————————————————————————————————————————————————————————————————————————————

MoteusStm32FdCan canBus;

static uint32_t gNextSendMillis = 0;

//——————————————————————————————————————————————————————————————————————————————
//  Moteus controller objects
//——————————————————————————————————————————————————————————————————————————————

Moteus moteus1(canBus, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());
Moteus moteus2(canBus, []() {
  Moteus::Options options;
  options.id = 2;
  return options;
}());

Moteus::PositionMode::Command position_cmd;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("started"));

  // Initialize FDCAN1 with Portenta H7 defaults.
  if (!canBus.begin()) {
    while (true) {
      Serial.println(F("FDCAN init failed"));
      delay(1000);
    }
  }

  // To clear any faults the controllers may have, we start by sending
  // a stop command to each.
  moteus1.SetStop();
  moteus2.SetStop();
  Serial.println(F("all stopped"));
}

uint16_t gLoopCount = 0;

void loop() {
  // We intend to send control frames every 20ms.
  const auto time = millis();
  if (gNextSendMillis >= time) { return; }

  gNextSendMillis += 20;
  gLoopCount++;

  Moteus::PositionMode::Command cmd;
  cmd.position = NaN;
  cmd.velocity = 0.2 * ::sin(time / 1000.0);

  moteus1.SetPosition(cmd);
  moteus2.SetBrake();

  if (gLoopCount % 5 != 0) { return; }

  // Only print our status every 5th cycle, so every 1s.

  Serial.print(F("time "));
  Serial.print(gNextSendMillis);

  auto print_moteus = [](const Moteus::Query::Result& query) {
    Serial.print(static_cast<int>(query.mode));
    Serial.print(F(" "));
    Serial.print(query.position);
    Serial.print(F("  velocity "));
    Serial.print(query.velocity);
  };

  print_moteus(moteus1.last_result().values);
  Serial.print(F(" / "));
  print_moteus(moteus2.last_result().values);
  Serial.println();
}
