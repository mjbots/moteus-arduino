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
/// Convenience header for using moteus with STM32 on-board FDCAN
/// hardware via the STM32 HAL.  Supports STM32H7, G4, and G0
/// families.
///
/// Two usage modes:
///
///  1. Default board configuration:
///       MoteusStm32FdCan can;
///       can.begin();  // configures FDCAN1 with board-appropriate defaults
///
///  2. Custom hardware:
///       FDCAN_HandleTypeDef hfdcan;
///       // ... user configures GPIO, clocks, timing, starts hfdcan ...
///       MoteusStm32FdCan can(&hfdcan);
///
/// Usage:
///   #include <MoteusStm32Fdcan.h>
///
/// Requires STM32 HAL headers with FDCAN support (typically provided
/// by an STM32 Arduino core, but the adapter itself uses only the
/// STM32 HAL, not Arduino APIs).

#pragma once

#include "stm32_def.h"

#if defined(STM32H7xx)
#include "stm32h7xx_hal_fdcan.h"
#elif defined(STM32G4xx)
#include "stm32g4xx_hal_fdcan.h"
#elif defined(STM32G0xx)
#include "stm32g0xx_hal_fdcan.h"
#else
#error "MoteusStm32Fdcan: unsupported STM32 family. Provide a pre-initialized FDCAN_HandleTypeDef or add support for your family."
#endif

#include "Moteus.h"

/// Adapter that wraps the STM32 HAL FDCAN peripheral to provide the
/// interface expected by MoteusController.
class MoteusStm32FdCan {
 public:
  /// Default constructor — uses an internal FDCAN_HandleTypeDef.
  /// Call begin() to configure and start FDCAN1 with board-appropriate
  /// defaults (1 Mbps, FD, no BRS).
  MoteusStm32FdCan() : hfdcan_(&hfdcan_storage_), owns_handle_(true) {}

  /// Construct with a user-provided, already-initialized handle.
  /// The caller is responsible for configuring GPIO, clocks, timing,
  /// filters, and calling HAL_FDCAN_Start() before using this adapter.
  MoteusStm32FdCan(FDCAN_HandleTypeDef* hfdcan)
      : hfdcan_(hfdcan), owns_handle_(false) {}

  /// Initialize and start FDCAN1 with board-appropriate defaults.
  /// Only valid when using the default constructor.
  /// Returns true on success.
  bool begin() {
    if (!owns_handle_) { return false; }
    initGpio();
    return initFdcan();
  }

  void poll() {
    // Detect bus-off and recover by restarting the peripheral.
    FDCAN_ProtocolStatusTypeDef status = {};
    if (HAL_FDCAN_GetProtocolStatus(hfdcan_, &status) == HAL_OK
        && status.BusOff) {
      HAL_FDCAN_AbortTxRequest(hfdcan_, 0xFF);
      HAL_FDCAN_Stop(hfdcan_);
      HAL_FDCAN_Start(hfdcan_);
    }
  }

  bool available() {
    return HAL_FDCAN_GetRxFifoFillLevel(hfdcan_, FDCAN_RX_FIFO0) > 0;
  }

  bool receive(CANFDMessage& msg) {
    if (!available()) { return false; }

    FDCAN_RxHeaderTypeDef rx = {};
    uint8_t buf[64] = {};
    if (HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx, buf) != HAL_OK) {
      return false;
    }

    msg.id  = rx.Identifier;
    msg.ext = (rx.IdType == FDCAN_EXTENDED_ID);
    msg.len = dlcToLen(rx.DataLength);
    ::memcpy(msg.data, buf, msg.len);

    if (rx.FDFormat == FDCAN_FD_CAN) {
      msg.type = (rx.BitRateSwitch == FDCAN_BRS_ON)
                     ? CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH
                     : CANFDMessage::CANFD_NO_BIT_RATE_SWITCH;
    } else {
      msg.type = CANFDMessage::CAN_DATA;
    }
    return true;
  }

  bool tryToSend(const CANFDMessage& msg) {
    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = msg.id;
    tx.IdType              = msg.ext ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = lenToDlc(msg.len);
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = (msg.type == CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH)
                                 ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    tx.FDFormat            = (msg.type == CANFDMessage::CAN_DATA)
                                 ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(
        hfdcan_, &tx, const_cast<uint8_t*>(msg.data)) == HAL_OK;
  }

 private:
  FDCAN_HandleTypeDef hfdcan_storage_ = {};
  FDCAN_HandleTypeDef* hfdcan_;
  bool owns_handle_;

  // ------------------------------------------------------------------
  // GPIO initialization — board-specific pin assignments for FDCAN1.
  // ------------------------------------------------------------------

  static void initGpio() {
    GPIO_InitTypeDef gpio = {};
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

#if defined(STM32H7xx)
    // Portenta H7: FDCAN1_RX = PB8 (AF9), FDCAN1_TX = PH13 (AF9)
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_8;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin       = GPIO_PIN_13;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOH, &gpio);

#elif defined(STM32G4xx)
    // Nucleo-G474RE: FDCAN1_RX = PB8 (AF9), FDCAN1_TX = PB9 (AF9)
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_8;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin       = GPIO_PIN_9;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &gpio);

#elif defined(STM32G0xx)
    // Nucleo-G0B1RE: FDCAN1_RX = PA11 (AF3), FDCAN1_TX = PA12 (AF3)
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_11;
    gpio.Alternate = GPIO_AF3_FDCAN1;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin       = GPIO_PIN_12;
    gpio.Alternate = GPIO_AF3_FDCAN1;
    HAL_GPIO_Init(GPIOA, &gpio);
#endif
  }

  // ------------------------------------------------------------------
  // FDCAN peripheral initialization — family-specific clock and config.
  //
  // All families: 1 Mbps nominal, FD mode, BRS disabled.
  // Timing: prescaler chosen so bit time = 16 TQ
  //         (1 sync + 13 seg1 + 2 seg2, SJW = 2)
  // ------------------------------------------------------------------

  bool initFdcan() {
    // Enable FDCAN peripheral clock.
#if defined(STM32H7xx)
    // H7: FDCAN kernel clock from PLL.  Assumes 80 MHz PLL output.
    __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PLL);
    __HAL_RCC_FDCAN_CLK_ENABLE();
#elif defined(STM32G4xx)
    // G4: FDCAN clock from PCLK1.  Assumes 170 MHz.
    __HAL_RCC_FDCAN_CLK_ENABLE();
#elif defined(STM32G0xx)
    // G0: FDCAN clock from PCLK1.  Assumes 64 MHz.
    __HAL_RCC_FDCAN_CLK_ENABLE();
#endif

    hfdcan_->Instance                    = FDCAN1;
    hfdcan_->Init.FrameFormat            = FDCAN_FRAME_FD_NO_BRS;
    hfdcan_->Init.Mode                   = FDCAN_MODE_NORMAL;
    hfdcan_->Init.AutoRetransmission     = DISABLE;
    hfdcan_->Init.TransmitPause          = DISABLE;
    hfdcan_->Init.ProtocolException      = DISABLE;

    // Nominal bit timing: target 1 Mbps with 16 TQ per bit.
    //   1 sync + 13 seg1 + 2 seg2 = 16 TQ
    //   prescaler = kernel_clock_MHz / (16 * 1 Mbps)
#if defined(STM32H7xx)
    // 80 MHz / 16 = 5 MHz per TQ → prescaler = 5
    hfdcan_->Init.NominalPrescaler       = 5;
#elif defined(STM32G4xx)
    // 170 MHz / 16 = 10.625 → not exact at 16 TQ.
    // Use 10 TQ instead: 1 sync + 7 seg1 + 2 seg2 = 10 TQ
    // 170 MHz / 10 = 17 → prescaler = 17, but 170/17 = 10 MHz → 1 Mbps.
    // Actually: prescaler 10, 17 TQ: 1 sync + 12 seg1 + 4 seg2 = 17 TQ
    //   170 / 17 / 10 = 1 MHz.  Use this for better sampling point.
    hfdcan_->Init.NominalPrescaler       = 10;
#elif defined(STM32G0xx)
    // 64 MHz / 16 = 4 → prescaler = 4
    hfdcan_->Init.NominalPrescaler       = 4;
#endif

    hfdcan_->Init.NominalSyncJumpWidth   = 2;

#if defined(STM32G4xx)
    // 170 MHz with prescaler 10: 17 TQ per bit.
    // 1 sync + 12 seg1 + 4 seg2 = 17 TQ.
    hfdcan_->Init.NominalTimeSeg1        = 12;
    hfdcan_->Init.NominalTimeSeg2        = 4;
#else
    // H7 (80 MHz / 5) and G0 (64 MHz / 4): 16 TQ per bit.
    // 1 sync + 13 seg1 + 2 seg2 = 16 TQ.
    hfdcan_->Init.NominalTimeSeg1        = 13;
    hfdcan_->Init.NominalTimeSeg2        = 2;
#endif

    // Data phase timing — same as nominal (BRS disabled, but HAL
    // requires valid values).
    hfdcan_->Init.DataPrescaler          = hfdcan_->Init.NominalPrescaler;
    hfdcan_->Init.DataSyncJumpWidth      = hfdcan_->Init.NominalSyncJumpWidth;
    hfdcan_->Init.DataTimeSeg1           = hfdcan_->Init.NominalTimeSeg1;
    hfdcan_->Init.DataTimeSeg2           = hfdcan_->Init.NominalTimeSeg2;

    hfdcan_->Init.StdFiltersNbr          = 0;
    hfdcan_->Init.ExtFiltersNbr          = 0;
    hfdcan_->Init.TxFifoQueueMode        = FDCAN_TX_FIFO_OPERATION;

    // The H7 HAL has additional message RAM configuration fields
    // that don't exist on G4/G0 (which have a fixed 212-word RAM).
#if defined(STM32H7xx)
    hfdcan_->Init.MessageRAMOffset       = 0;
    hfdcan_->Init.RxFifo0ElmtsNbr       = 8;
    hfdcan_->Init.RxFifo0ElmtSize       = FDCAN_DATA_BYTES_64;
    hfdcan_->Init.RxFifo1ElmtsNbr       = 0;
    hfdcan_->Init.RxFifo1ElmtSize       = FDCAN_DATA_BYTES_64;
    hfdcan_->Init.RxBuffersNbr          = 0;
    hfdcan_->Init.RxBufferSize          = FDCAN_DATA_BYTES_64;
    hfdcan_->Init.TxEventsNbr           = 0;
    hfdcan_->Init.TxBuffersNbr          = 0;
    hfdcan_->Init.TxFifoQueueElmtsNbr   = 8;
    hfdcan_->Init.TxElmtSize            = FDCAN_DATA_BYTES_64;
#endif

    // G4/G0 have a ClockDivider field that H7 does not.
#if defined(STM32G4xx) || defined(STM32G0xx)
    hfdcan_->Init.ClockDivider           = FDCAN_CLOCK_DIV1;
#endif

    if (HAL_FDCAN_Init(hfdcan_) != HAL_OK) { return false; }

    // Accept all frames (standard + extended) into RX FIFO0.
    if (HAL_FDCAN_ConfigGlobalFilter(
            hfdcan_,
            FDCAN_ACCEPT_IN_RX_FIFO0,
            FDCAN_ACCEPT_IN_RX_FIFO0,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK) {
      return false;
    }

    return HAL_FDCAN_Start(hfdcan_) == HAL_OK;
  }

  /// Convert an STM32 HAL DLC code to a byte count.
  static uint8_t dlcToLen(uint32_t dlc) {
    switch (dlc) {
      case FDCAN_DLC_BYTES_0:  return 0;
      case FDCAN_DLC_BYTES_1:  return 1;
      case FDCAN_DLC_BYTES_2:  return 2;
      case FDCAN_DLC_BYTES_3:  return 3;
      case FDCAN_DLC_BYTES_4:  return 4;
      case FDCAN_DLC_BYTES_5:  return 5;
      case FDCAN_DLC_BYTES_6:  return 6;
      case FDCAN_DLC_BYTES_7:  return 7;
      case FDCAN_DLC_BYTES_8:  return 8;
      case FDCAN_DLC_BYTES_12: return 12;
      case FDCAN_DLC_BYTES_16: return 16;
      case FDCAN_DLC_BYTES_20: return 20;
      case FDCAN_DLC_BYTES_24: return 24;
      case FDCAN_DLC_BYTES_32: return 32;
      case FDCAN_DLC_BYTES_48: return 48;
      default:                 return 64;
    }
  }

  /// Convert a byte count to an STM32 HAL DLC code.
  static uint32_t lenToDlc(uint8_t len) {
    switch (len) {
      case 0:  return FDCAN_DLC_BYTES_0;
      case 1:  return FDCAN_DLC_BYTES_1;
      case 2:  return FDCAN_DLC_BYTES_2;
      case 3:  return FDCAN_DLC_BYTES_3;
      case 4:  return FDCAN_DLC_BYTES_4;
      case 5:  return FDCAN_DLC_BYTES_5;
      case 6:  return FDCAN_DLC_BYTES_6;
      case 7:  return FDCAN_DLC_BYTES_7;
      case 8:  return FDCAN_DLC_BYTES_8;
      case 12: return FDCAN_DLC_BYTES_12;
      case 16: return FDCAN_DLC_BYTES_16;
      case 20: return FDCAN_DLC_BYTES_20;
      case 24: return FDCAN_DLC_BYTES_24;
      case 32: return FDCAN_DLC_BYTES_32;
      case 48: return FDCAN_DLC_BYTES_48;
      default: return FDCAN_DLC_BYTES_64;
    }
  }
};

using Moteus = MoteusController<MoteusStm32FdCan>;
