#pragma once
#include "board.h"

#ifdef __cplusplus
#include <atomic>
/* Encoder state — written by encoder.cpp ISR path, read by gui/lv_port_indev.cpp */
extern std::atomic<int> g_enc_diff;
extern std::atomic<int> g_enc_btn;
#endif
