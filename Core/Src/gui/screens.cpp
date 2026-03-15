#include "lvgl.h"
#include "screens.h"

static void screen_theme_load(); // forward declaration

/* ---- Theme customization state ---- */

struct ColorOption { const char *name; lv_palette_t palette; };

static constexpr ColorOption color_options[] = {
    {"Blue",   LV_PALETTE_BLUE},
    {"Red",    LV_PALETTE_RED},
    {"Green",  LV_PALETTE_GREEN},
    {"Purple", LV_PALETTE_PURPLE},
    {"Orange", LV_PALETTE_ORANGE},
    {"Teal",   LV_PALETTE_TEAL},
};
static constexpr int COLOR_COUNT = sizeof(color_options) / sizeof(color_options[0]);

static int s_press_color_idx = 0;

/* ---- Menu ---- */

static const char * const items[] = {
    "Item 1", "Item 2", "Item 3", "Customize Theme"
};
static constexpr int ITEM_COUNT = 4;
static int s_focus_idx = 0;

static void item_event_cb(lv_event_t *e)
{
    int idx = (int)(uintptr_t)lv_event_get_user_data(e);
    s_focus_idx = idx;
    if (idx == ITEM_COUNT - 1)
        screen_theme_load();
    else
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
        lv_obj_add_event_cb(btn, item_event_cb, LV_EVENT_RELEASED, (void*)(uintptr_t)i);
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
    lv_obj_add_event_cb(btn, back_cb, LV_EVENT_RELEASED, nullptr);

    lv_scr_load_anim(scr, LV_SCR_LOAD_ANIM_NONE, 0, 0, true);
}

/* ---- Theme customization ---- */

void screens_apply_theme(void)
{
    lv_theme_t *th = lv_theme_default_init(
        lv_display_get_default(),
        lv_palette_main(color_options[s_press_color_idx].palette),
        lv_palette_main(LV_PALETTE_CYAN),
        true, &lv_font_montserrat_18);
    lv_display_set_theme(lv_display_get_default(), th);
}

static void color_event_cb(lv_event_t *e)
{
    int idx = (int)(uintptr_t)lv_event_get_user_data(e);
    if (idx >= 0) {
        s_press_color_idx = idx;
        screens_apply_theme();
    }
    screen_menu_load();
}

static void screen_theme_load()
{
    lv_obj_t *scr  = lv_obj_create(nullptr);
    lv_obj_t *list = lv_list_create(scr);
    lv_obj_set_size(list, LV_PCT(100), LV_PCT(100));
    lv_obj_center(list);

    lv_list_add_text(list, "Highlight Color");

    for (int i = 0; i < COLOR_COUNT; i++) {
        lv_obj_t *btn = lv_list_add_button(list, nullptr, color_options[i].name);
        lv_obj_add_event_cb(btn, color_event_cb, LV_EVENT_RELEASED, (void*)(uintptr_t)i);

        // Left color swatch
        lv_obj_set_style_border_color(btn, lv_palette_main(color_options[i].palette), LV_PART_MAIN);
        lv_obj_set_style_border_width(btn, 4, LV_PART_MAIN);
        lv_obj_set_style_border_side(btn, LV_BORDER_SIDE_LEFT, LV_PART_MAIN);

        // Tick mark on currently active selection
        if (i == s_press_color_idx)
            lv_obj_set_style_bg_color(btn,
                lv_palette_darken(color_options[i].palette, 3), LV_PART_MAIN);
    }

    // Back button — returns without changing color (idx = -1 signals no change)
    lv_obj_t *back = lv_list_add_button(list, nullptr, "Back");
    lv_obj_add_event_cb(back, color_event_cb, LV_EVENT_RELEASED, (void*)(uintptr_t)(unsigned)-1);

    lv_scr_load_anim(scr, LV_SCR_LOAD_ANIM_NONE, 0, 0, true);
}
