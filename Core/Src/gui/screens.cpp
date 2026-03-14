#include "lvgl.h"
#include "screens.h"

/* ---- Menu ---- */

static const char * const items[] = {
    "Item 1", "Item 2", "Item 3", "Item 4"
};
static constexpr int ITEM_COUNT = 4;
static int s_focus_idx = 0;

static void item_event_cb(lv_event_t *e)
{
    int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    s_focus_idx = idx;
    screen_detail_load(idx);
}

void screen_menu_load(void)
{
    lv_obj_t *scr  = lv_obj_create(nullptr);
    lv_obj_t *list = lv_list_create(scr);
    lv_obj_set_size(list, LV_PCT(100), LV_PCT(100));
    lv_obj_center(list);

    lv_obj_t *focus_btn = nullptr;
    for (int i = 0; i < ITEM_COUNT; i++) {
        lv_obj_t *btn = lv_list_add_button(list, nullptr, items[i]);
        lv_obj_add_event_cb(btn, item_event_cb, LV_EVENT_CLICKED,
                            reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        if (i == s_focus_idx) focus_btn = btn;
    }
    if (focus_btn) lv_group_focus_obj(focus_btn);

    lv_scr_load_anim(scr, LV_SCR_LOAD_ANIM_NONE, 0, 0, true);
}

/* ---- Detail ---- */

static void back_cb(lv_event_t * /*e*/)
{
    screen_menu_load();
}

void screen_detail_load(int idx)
{
    lv_obj_t *scr = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000080), LV_PART_MAIN);

    lv_obj_t *lbl = lv_label_create(scr);
    lv_label_set_text_fmt(lbl, "Detail: Item %d", idx + 1);
    lv_obj_center(lbl);

    lv_obj_t *btn = lv_button_create(scr);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_label_set_text(lv_label_create(btn), "Back");
    lv_obj_add_event_cb(btn, back_cb, LV_EVENT_CLICKED, nullptr);

    lv_scr_load_anim(scr, LV_SCR_LOAD_ANIM_NONE, 0, 0, true);
}
