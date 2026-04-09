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

# Add Teensy board manager URL.
"$CLI" config init --overwrite 2>/dev/null || true
"$CLI" config add board_manager.additional_urls \
  https://www.pjrc.com/teensy/package_teensy_index.json

# Install board cores.
"$CLI" core update-index
"$CLI" core install arduino:avr
"$CLI" core install teensy:avr

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
  if "$CLI" compile --library "$SCRIPT_DIR" --fqbn "$fqbn" "$sketch" >/dev/null 2>&1; then
    echo "OK"
    PASS=$((PASS + 1))
  else
    echo "FAILED"
    "$CLI" compile --library "$SCRIPT_DIR" --fqbn "$fqbn" "$sketch" 2>&1 | tail -20
    FAIL=$((FAIL + 1))
  fi
}

compile arduino:avr:leonardo "$SCRIPT_DIR/examples/BasicControl/BasicControl.ino"
compile arduino:avr:leonardo "$SCRIPT_DIR/examples/DiagnosticProtocol/DiagnosticProtocol.ino"
compile arduino:avr:leonardo "$SCRIPT_DIR/examples/WaitComplete/WaitComplete.ino"
compile arduino:avr:leonardo "$SCRIPT_DIR/examples/UartControl/UartControl.ino"
compile teensy:avr:teensy41  "$SCRIPT_DIR/examples/TeensyBasicControl/TeensyBasicControl.ino"

echo ""
echo "$PASS passed, $FAIL failed"

if [ "$FAIL" -ne 0 ]; then
  exit 1
fi
