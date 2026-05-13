/*====================================================================
<frame_data.hpp>
・シリアル通信のフレームデータ定義ヘッダーファイル

serial_bridge (PC側) との関係:
- PC -> MCU は 24 x int16 (serial_bridge/include/serial_bridge/config.hpp の kTx16Num)
- MCU -> PC は 17 x int16 (kRx16Num)

注意:
- esp32_serial_bridge と同じ命名に合わせて
  Tx_16Data = MCU -> PC
  Rx_16Data = PC -> MCU
  としています。

Copyright (c) 2026.
====================================================================*/

#pragma once
#include <stdint.h>

#define Tx16NUM 17 // MCU -> PC
#define Rx16NUM 24 // PC -> MCU

extern volatile int16_t Tx_16Data[Tx16NUM];
/*
0: debug/reserved
1: angle (0.1 deg)
2: velocity (0.1 rad/s)
3: target (unit depends on mode)
4: mode (0=velocity, 1=angle)
5: voltage_limit (0.1 V)
6..16: reserved
*/

extern volatile int16_t Rx_16Data[Rx16NUM];
/*
0: debug/reserved
1: enable (0=stop, 1=run)
2: mode (0=velocity, 1=angle)
3: target_velocity (0.1 rad/s)
4: target_angle (0.1 deg)
5: voltage_limit (0.1 V)
6..23: reserved
*/
