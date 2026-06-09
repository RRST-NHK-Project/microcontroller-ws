/*====================================================================
<frame_data.hpp>
・シリアル通信のフレームデータ定義ヘッダーファイル

serial_bridge (PC側) との関係:
- PC -> MCU は 24 x int16
- MCU -> PC も 24 x int16

注意:
- esp32_serial_bridge と同じ命名に合わせて
  Tx_16Data = MCU -> PC
  Rx_16Data = PC -> MCU
  としています。

Copyright (c) 2026.
====================================================================*/

#pragma once
#include <stdint.h>

#define Tx16NUM 24 // MCU -> PC
#define Rx16NUM 24 // PC -> MCU

extern volatile int16_t Tx_16Data[Tx16NUM];
/*
0: debug/reserved
1: angle (0.1 deg)
2: velocity (0.1 rad/s)
3: target (unit depends on mode)
4: mode (0=velocity, 1=angle)
5: voltage_limit (0.1 V)
6: rpm (1 rpm, signed)
7: velocity_limit (0.1 rad/s)
8: current_limit (0.1 A)
9: velocity PID P (0.001)
10: velocity PID I (0.001)
11: velocity PID D (0.001)
12: velocity PID output_ramp (1.0)
13: velocity LPF Tf (ms)
14: angle P gain (0.01)
15..23: reserved
*/

extern volatile int16_t Rx_16Data[Rx16NUM];
/*
0: debug/reserved
1: enable (0=stop, 1=run)
2: mode (0=velocity, 1=angle)
3: target_velocity (1 rpm)
4: target_angle (0.1 deg)
5: voltage_limit (0.1 V)
6: param_apply_mask (bit-field)
7: velocity_limit (0.1 rad/s)
8: current_limit (0.1 A)
9: velocity PID P (0.001)
10: velocity PID I (0.001)
11: velocity PID D (0.001)
12: velocity PID output_ramp (1.0)
13: velocity LPF Tf (ms)
14: angle P gain (0.01)
15..23: reserved
*/
