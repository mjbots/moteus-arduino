//——————————————————————————————————————————————————————————————————————————————

// Demonstrates how to use SetPositionWaitComplete to wait until the
// exact time that a trajectory motion is completed.  Intended to
// execute on a CANBed FD from longan labs.
//  * https://mjbots.com/products/moteus-r4-11
//  * https://www.longan-labs.cc/1030009.html
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

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2517FD can (MCP2517_CS, SPI, MCP2517_INT) ;

Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 1;
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

  // First we'll clear faults.
  moteus1.SetStop();
  Serial.println(F("all stopped"));

  position_fmt.velocity_limit = Moteus::kFloat;
  position_fmt.accel_limit = Moteus::kFloat;

  position_cmd.velocity_limit = 2.0;
  position_cmd.accel_limit = 3.0;
}

void loop() {
  auto print_state = [&]() {
    const auto& v = moteus1.last_result().values;
    Serial.print(F(" mode="));
    Serial.print(static_cast<int>(v.mode));
    Serial.print(F(" pos="));
    Serial.print(v.position);
    Serial.print(F(" vel="));
    Serial.print(v.velocity);
    Serial.print(F(" torque="));
    Serial.print(v.torque);
    Serial.println();
  };

  position_cmd.position = 0.1;
  moteus1.SetPositionWaitComplete(position_cmd, 0.02, &position_fmt);

  Serial.print(F("sent 0.1 "));
  print_state();

  position_cmd.position = 0.5;
  moteus1.SetPositionWaitComplete(position_cmd, 0.02, &position_fmt);

  Serial.print(F("sent 0.5 "));
  print_state();
}
