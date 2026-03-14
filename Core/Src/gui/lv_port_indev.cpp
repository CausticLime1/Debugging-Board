#include <atomic>
#include "lvgl.h"
#include "lv_port_indev.h"

std::atomic<int> g_enc_diff{0};
std::atomic<int> g_enc_btn{0};

static void encoder_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    data->enc_diff = static_cast<int16_t>(g_enc_diff.exchange(0, std::memory_order_relaxed));
    data->state    = g_enc_btn.exchange(0, std::memory_order_relaxed)
                       ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

void enc_report_cw(void)  { g_enc_diff.fetch_add( 1, std::memory_order_relaxed); }
void enc_report_ccw(void) { g_enc_diff.fetch_add(-1, std::memory_order_relaxed); }
void enc_report_btn(void) { g_enc_btn.store(1, std::memory_order_relaxed); }

lv_indev_t * lv_port_indev_init(void)
{
    lv_indev_t *enc = lv_indev_create();
    lv_indev_set_type(enc, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_read_cb(enc, encoder_read_cb);
    return enc;
}
