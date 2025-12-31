/*====================================================================
<>
・
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once
#include <stdint.h>

#define Tx16NUM 8 // 送受信するint16データの数

extern volatile int16_t Tx_16Data[Tx16NUM];
extern volatile int16_t Rx_16Data[Tx16NUM];