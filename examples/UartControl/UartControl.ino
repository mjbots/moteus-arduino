//——————————————————————————————————————————————————————————————————————————————
// Demonstration of controlling a moteus controller over a direct UART
// connection.  This can be used with any Arduino board that has a
// spare hardware serial port (Serial1, Serial2, etc.).
//
// The moteus must be configured to use UART mode on one of its aux
// ports with the baud rate set to 921600 (the default).
//
// Wiring:
//   Arduino TX -> moteus UART RX
//   Arduino RX -> moteus UART TX
//   Arduino GND -> moteus GND
//——————————————————————————————————————————————————————————————————————————————

#include <MoteusUart.h>
#include <Moteus.h>

MoteusUart<HardwareSerial> uart_bus(Serial1);

MoteusController<MoteusUart<HardwareSerial>> moteus1(uart_bus, []() {
  MoteusController<MoteusUart<HardwareSerial>>::Options options;
  options.id = 1;
  // UART connections can lose responses, so retry diagnostic reads.
  options.diagnostic_retry_count = 3;
  return options;
}());

static uint32_t gNextSendMillis = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("started"));

  // Initialize the UART connection to moteus at 921600 baud.
  uart_bus.begin();

  // Clear any faults.
  moteus1.SetStop();
  Serial.println(F("stopped"));
}

uint16_t gLoopCount = 0;

void loop() {
  // Send control frames every 20ms.
  const auto time = millis();
  if (gNextSendMillis >= time) { return; }

  gNextSendMillis += 20;
  gLoopCount++;

  MoteusController<MoteusUart<HardwareSerial>>::PositionMode::Command cmd;
  cmd.position = NaN;
  cmd.velocity = 0.2 * ::sin(time / 1000.0);

  moteus1.SetPosition(cmd);

  if (gLoopCount % 50 != 0) { return; }

  // Print status every 50th cycle (1s).
  const auto& query = moteus1.last_result().values;
  Serial.print(F("mode "));
  Serial.print(static_cast<int>(query.mode));
  Serial.print(F("  pos "));
  Serial.print(query.position);
  Serial.print(F("  vel "));
  Serial.print(query.velocity);
  Serial.println();
}
