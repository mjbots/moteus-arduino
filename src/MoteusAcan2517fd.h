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
/// Convenience header for using moteus with the ACAN2517FD library
/// (MCP2517FD / MCP2518FD external CAN-FD controllers over SPI).
///
/// Usage:
///   #include <moteus_acan2517fd.h>
///
/// Requires the ACAN2517FD library to be installed.

#pragma once

#include <ACAN2517FD.h>
#include "Moteus.h"

using Moteus = MoteusController<ACAN2517FD>;
