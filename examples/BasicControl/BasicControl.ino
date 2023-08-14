//——————————————————————————————————————————————————————————————————————————————
// Demonstration of control and monitoring of 2 moteus controllers
// running on a CANBed FD from longan labs.
//  * https://mjbots.com/products/moteus-r4-11
//  * https://www.longan-labs.cc/1030009.html
//
// Controller ID 1 is moved through a sine wave pattern, while
// controller ID 2 just has a brake command sent.
// ——————————————————————————————————————————————————————————————————————————————

#include <ACAN2517FD.h>
#include <Moteus.h>

//——————————————————————————————————————————————————————————————————————————————
//  The following pins are selected for the CANBed FD board.
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2517_SCK = 9 ; // SCK input of MCP2517
static const byte MCP2517_SDI =  10 ; // SDI input of MCP2517
static const byte MCP2517_SDO =  11 ; // SDO output of MCP2517

static const byte MCP2517_CS  = 17 ; // CS input of MCP2517
static const byte MCP2517_INT = 7 ; // INT output of MCP2517

static uint32_t gNextSendMillis = 0;

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2517FD can (MCP2517_CS, SPI, MCP2517_INT) ;

Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 1;
  // By default, only position and velocity are queried.  Additional
  // fields can be requested by changing their resolution in the
  // options structure.
  options.query_format.torque = Moteus::kFloat;
  return options;
}());

Moteus moteus2(can, []() {
  Moteus::Options options;
  options.id = 2;
  options.query_format.torque = Moteus::kFloat;
  return options;
}());

Moteus::PositionMode::Command position_cmd;
Moteus::PositionMode::Format position_fmt;


void setup() {
  pinMode (LED_BUILTIN, OUTPUT);

  // Let the world know we have begun!
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("started"));

  SPI.begin();

  // This operates the CAN-FD bus at 1Mbit for both the arbitration
  // and data rate.  Most arduino shields cannot operate at 5Mbps
  // correctly, so the moteus Arduino library permanently disables
  // BRS.
  ACAN2517FDSettings settings(
      ACAN2517FDSettings::OSC_20MHz, 1000ll * 1000ll, DataBitRateFactor::x1);

  // The atmega32u4 on the CANbed has only a tiny amount of memory.
  // The ACAN2517FD driver needs custom settings so as to not exhaust
  // all of SRAM just with its buffers.
  settings.mArbitrationSJW = 2;
  settings.mDriverTransmitFIFOSize = 1;
  settings.mDriverReceiveFIFOSize = 2;

  const uint32_t errorCode = can.begin(settings, [] { can.isr(); });

  while (errorCode != 0) {
    Serial.print(F("CAN error 0x"));
    Serial.println(errorCode, HEX);
    delay(1000);
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
  cmd.velocity = 0.5 * ::sin(time / 300.0);

  moteus1.SetPosition(cmd, &position_fmt);
  moteus2.SetBrake();

  if (gLoopCount % 5 != 0) { return; }

  // Only print our status every 5th cycle, so every 1s.

  Serial.print(F("time "));
  Serial.print(gNextSendMillis);

  auto print_moteus = [](const Moteus::Query::Result& query) {
    Serial.print(static_cast<int>(query.mode));
    Serial.print(F(" pos "));
    Serial.print(query.position);
    Serial.print(F("  vel "));
    Serial.print(query.velocity);
    Serial.print(F("  torque "));
    Serial.print(query.torque);
  };

  Serial.print(F(" [ "));
  print_moteus(moteus1.last_result().values);
  Serial.print(F(" ] / [ "));
  print_moteus(moteus2.last_result().values);
  Serial.println(F(" ]"));
}
