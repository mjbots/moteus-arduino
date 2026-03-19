# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Arduino library for controlling mjbots moteus brushless servo controllers over CAN-FD. Header-only C++ library targeting Arduino boards with MCP2517FD/MCP2518FD CAN-FD hardware (e.g., CANBed FD). Depends on the ACAN2517FD library.

## Build & Test

This is an Arduino library (no Makefile/CMake). To compile examples:

```bash
# Using arduino-cli (install board core first)
arduino-cli compile --fqbn <board_fqbn> examples/BasicControl/BasicControl.ino
```

There is no formal test suite. The three examples in `examples/` serve as integration tests and must be verified on real hardware with a moteus controller.

## Architecture

All source is header-only under `src/`, in the `mjbots::moteus` namespace:

- **Moteus.h** — Primary user-facing `Moteus` class. Three patterns per operation:
  - `Make*()` — construct a CAN frame (for batching via `Transport::Cycle`)
  - `Set*()` — blocking: send command and wait for response
  - `Begin*()` — async: send command, poll later with `Poll()`
- **moteus_protocol.h** — Register definitions, mode enums, command/query structs (PositionMode, VFOCMode, CurrentMode, etc.)
- **moteus_multiplex.h** — CAN-FD payload serialization with configurable resolution (Int8/Int16/Int32/Float)
- **moteus_transport.h** — Abstract `Transport` interface; CAN-FD frame structure with bus routing and BRS support
- **moteus_optional.h** — Minimal C++11 `Optional<T>` (no STL dependency)
- **moteus_tokenizer.h** — String tokenizer for diagnostic protocol responses

## Key Patterns

- **CAN ID encoding**: `[destination | source<<8 | reply_bit<<15 | can_prefix<<16]`
- **Query format**: Commands automatically append a register query (configurable via `Query::Format`) so command+status is a single round trip
- **BRS disabled by default** (`disable_brs=true`) for hardware compatibility
- **ACAN2517FD interrupt workaround**: Library manually polls the CAN controller rather than relying on interrupts
- **Memory-conscious**: Avoids STL, uses raw `char*` allocation, careful with SRAM on constrained boards
- **NaN sentinel**: Unset `Optional<double>` fields use NaN; the multiplex layer skips NaN values during encoding
