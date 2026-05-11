/*====================================================================
<config.hpp>
書き込み前にここでIDと通信周期、スロット割り当てを設定してください。
serial_bridge (ROS2) 側は DEVICE_ID と int16配列のスロット数(PC->MCU=24, MCU->PC=17)
に合わせて動作します。

Copyright (c) 2026.
====================================================================*/

#pragma once

#include <stdint.h>

// ================= 基本設定 =================

// IDの設定（ROS側からマイコンを識別するために使用）
// すべてのマイコンで異なる値にすること
#define DEVICE_ID 0x01

// MCU -> PC 送信周期 (ms)
#define TX_PERIOD_MS 20

// PC -> MCU コマンドタイムアウト (ms)
// この時間を超えて受信が無い場合はフェイルセーフ(stop)
#define CMD_TIMEOUT_MS 500

// ================= デフォルト値 =================

#define DEFAULT_VOLTAGE_SUPPLY 24.0f
#define DEFAULT_VOLTAGE_LIMIT 12.0f
#define DEFAULT_VELOCITY_LIMIT 1000.0f

// ================= Slot Index (PC -> MCU) =================
// Rx_16Data[0..23]
#define RX_DEBUG 0
#define RX_ENABLE 1
#define RX_MODE 2
#define RX_TARGET_VELOCITY 3
#define RX_TARGET_ANGLE 4
#define RX_VOLTAGE_LIMIT 5

// ================= Slot Index (MCU -> PC) =================
// Tx_16Data[0..16]
#define TX_DEBUG 0
#define TX_ANGLE 1
#define TX_VELOCITY 2
#define TX_TARGET 3
#define TX_MODE 4
#define TX_VOLTAGE_LIMIT 5

// ================= Unit scaling =================
// 受信側: int16 -> float
// - velocity: 0.1 rad/s
// - angle: 0.1 deg
// - voltage_limit: 0.1 V
#define TARGET_VELOCITY_SCALE 0.1f
#define TARGET_ANGLE_SCALE 0.1f
#define VOLTAGE_LIMIT_SCALE 0.1f
