#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Find or download arduino-cli.
if command -v arduino-cli >/dev/null 2>&1; then
  CLI=arduino-cli
elif [ -x "$SCRIPT_DIR/arduino-cli" ]; then
  CLI="$SCRIPT_DIR/arduino-cli"
elif [ -x ~/Downloads/arduino-cli/arduino-cli ]; then
  CLI=~/Downloads/arduino-cli/arduino-cli
else
  echo "Downloading arduino-cli..."
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$SCRIPT_DIR" sh
  CLI="$SCRIPT_DIR/arduino-cli"
fi

echo "Using: $CLI"

# Add third-party board manager URLs.
"$CLI" config init --overwrite 2>/dev/null || true
"$CLI" config add board_manager.additional_urls \
  https://www.pjrc.com/teensy/package_teensy_index.json \
  https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json

# Install board cores.
"$CLI" core update-index
"$CLI" core install arduino:avr
"$CLI" core install teensy:avr
"$CLI" core install STMicroelectronics:stm32

# Install library dependencies.
"$CLI" lib install ACAN2517FD
"$CLI" lib install ACAN_T4

# Compile each example.
PASS=0
FAIL=0

compile() {
  local fqbn="$1"
  local sketch="$2"
  local name
  name="$(basename "$sketch")"

  echo -n "Compiling $name for $fqbn ... "
  local output
  if output=$("$CLI" compile --library "$SCRIPT_DIR" --fqbn "$fqbn" "$sketch" 2>&1); then
    echo "OK"
    PASS=$((PASS + 1))
  else
    echo "FAILED"
    echo "$output" | tail -20
    FAIL=$((FAIL + 1))
  fi
}

# Host compile test: builds the whole library with plain g++ on the
# non-Arduino code path.  Fast, and gives readable errors.
echo -n "Compiling host_compile_test with g++ ... "
if OUTPUT=$(g++ -std=gnu++11 -Wall -Wextra -Werror -I"$SCRIPT_DIR/src" \
              "$SCRIPT_DIR/tests/host_compile_test/host_compile_test.cc" \
              -o /tmp/moteus_host_compile_test 2>&1); then
  echo "OK"
  PASS=$((PASS + 1))
else
  echo "FAILED"
  echo "$OUTPUT" | tail -20
  FAIL=$((FAIL + 1))
fi

# The examples.
compile arduino:avr:leonardo "$SCRIPT_DIR/examples/BasicControl/BasicControl.ino"
compile arduino:avr:leonardo "$SCRIPT_DIR/examples/DiagnosticProtocol/DiagnosticProtocol.ino"
compile arduino:avr:leonardo "$SCRIPT_DIR/examples/WaitComplete/WaitComplete.ino"
compile arduino:avr:leonardo "$SCRIPT_DIR/examples/UartControl/UartControl.ino"
compile teensy:avr:teensy41  "$SCRIPT_DIR/examples/TeensyBasicControl/TeensyBasicControl.ino"
compile "STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_G474RE" \
  "$SCRIPT_DIR/examples/Stm32BasicControl/Stm32BasicControl.ino"
compile "STMicroelectronics:stm32:Nucleo_144:pnum=NUCLEO_H743ZI2" \
  "$SCRIPT_DIR/examples/Stm32BasicControl/Stm32BasicControl.ino"

# The whole-API compile test, compiled AND linked for every
# transport on each platform.  (Uses mega rather than leonardo
# because the fully-linked API surface exceeds 32u4 flash; the
# leonardo example builds above cover that toolchain target.)
compile arduino:avr:mega    "$SCRIPT_DIR/tests/arduino_compile_test/arduino_compile_test.ino"
compile teensy:avr:teensy41 "$SCRIPT_DIR/tests/arduino_compile_test/arduino_compile_test.ino"
compile "STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_G474RE" \
  "$SCRIPT_DIR/tests/arduino_compile_test/arduino_compile_test.ino"
compile "STMicroelectronics:stm32:Nucleo_144:pnum=NUCLEO_H743ZI2" \
  "$SCRIPT_DIR/tests/arduino_compile_test/arduino_compile_test.ino"

echo ""
echo "$PASS passed, $FAIL failed"

if [ "$FAIL" -ne 0 ]; then
  exit 1
fi
