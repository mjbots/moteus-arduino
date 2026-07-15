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
/// The stm32duino Arduino core treats the HAL FDCAN module as opt-in:
/// it is only compiled when HAL_FDCAN_MODULE_ENABLED is defined via a
/// "hal_conf_extra.h" found on the include path.  Shipping this file
/// with the library makes MoteusStm32Fdcan.h link without requiring
/// every sketch to provide its own copy.
///
/// Note: if your sketch supplies its own hal_conf_extra.h, it takes
/// precedence over this one — in that case add
/// "#define HAL_FDCAN_MODULE_ENABLED" to it yourself.
///
/// This header has no effect on non-STM32 platforms; nothing includes
/// it there.

#pragma once

#define HAL_FDCAN_MODULE_ENABLED
