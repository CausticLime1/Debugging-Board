#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* C-callable shims — written by the atomic ISR bridge, called from exti_user.h */
void enc_report_cw(void);
void enc_report_ccw(void);
void enc_report_btn(void);

struct _lv_indev_t;
struct _lv_indev_t * lv_port_indev_init(void);

#ifdef __cplusplus
}

#include <atomic>
extern std::atomic<int> g_enc_diff;
extern std::atomic<int> g_enc_btn;
#endif
