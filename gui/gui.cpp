#include "lvgl.h"
#include "stm32h7xx_hal.h"
#include "gui.h"
#include "lv_port.h"

namespace {

/* ---- Forward declarations ---- */

void screen_menu_load();
void screen_detail_load(int idx);
void screen_theme_load();

/* ---- Theme customization state ---- */

struct ColorOption { const char *name; lv_palette_t palette; };

constexpr ColorOption color_options[] = {
    {"Blue",   LV_PALETTE_BLUE},
    {"Red",    LV_PALETTE_RED},
    {"Green",  LV_PALETTE_GREEN},
    {"Purple", LV_PALETTE_PURPLE},
    {"Orange", LV_PALETTE_ORANGE},
    {"Teal",   LV_PALETTE_TEAL},
};
constexpr int COLOR_COUNT = (int)(sizeof(color_options) / sizeof(color_options[0]));

int s_press_color_idx = 0;

/* ---- Flash persistence ---- */

constexpr uint32_t NV_BASE = 0x080E0000U;

void nv_save()
{
    __attribute__((aligned(32))) uint32_t buf[8] = {
        (uint32_t)s_press_color_idx, ~0u, ~0u, ~0u, ~0u, ~0u, ~0u, ~0u
    };
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks     = FLASH_BANK_1,
        .Sector    = FLASH_SECTOR_7,
        .NbSectors = 1,
    };
    uint32_t erase_error;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &erase_error);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, NV_BASE, (uint32_t)buf);
    HAL_FLASH_Lock();
}

void screens_load_prefs()
{
    uint32_t stored = *(volatile uint32_t *)NV_BASE;
    if (stored < (uint32_t)COLOR_COUNT)
        s_press_color_idx = (int)stored;
}

void screens_apply_theme()
{
    lv_theme_t *th = lv_theme_default_init(
        lv_display_get_default(),
        lv_palette_main(color_options[s_press_color_idx].palette),
        lv_palette_main(LV_PALETTE_CYAN),
        true, &lv_font_montserrat_18);
    lv_display_set_theme(lv_display_get_default(), th);
}

/* ---- Menu ---- */

constexpr const char * const items[] = {
    "Item 1", "Item 2", "Item 3", "Customize Theme"
};
constexpr int ITEM_COUNT = (int)(sizeof(items) / sizeof(items[0]));

int s_focus_idx = 0;

void item_event_cb(lv_event_t *e)
{
    int idx = (int)(uintptr_t)lv_event_get_user_data(e);
    s_focus_idx = idx;
    if (idx == ITEM_COUNT - 1)
        screen_theme_load();
    else
        screen_detail_load(idx);
}

/* ---- Detail ---- */

void back_cb(lv_event_t * /*e*/)
{
    screen_menu_load();
}

/* ---- Theme customization ---- */

void color_event_cb(lv_event_t *e)
{
    int idx = (int)(uintptr_t)lv_event_get_user_data(e);
    if (idx >= 0) {
        s_press_color_idx = idx;
        nv_save();
        screens_apply_theme();
    }
    screen_menu_load();
}

void screen_theme_load()
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

void screen_menu_load()
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

} // namespace

/* ---- Public API: LVGL init + task loop ---- */

void gui_init()
{
    lv_init();
    lv_tick_set_cb(HAL_GetTick);

    lv_port_disp_init();
    screens_load_prefs();
    screens_apply_theme();

    lv_indev_t *enc = lv_port_indev_init();
    lv_group_t *grp = lv_group_create();
    lv_group_set_default(grp);
    lv_indev_set_group(enc, grp);

    screen_menu_load();
}

void gui_task_run()
{
    lv_timer_handler();
}
