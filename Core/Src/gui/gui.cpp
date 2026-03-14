#include "lvgl.h"
#include "stm32h7xx_hal.h"
#include "gui.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "screens.h"

void gui_init(void)
{
    lv_init();
    lv_tick_set_cb(HAL_GetTick); // ms-accurate tick from SysTick HAL

    lv_port_disp_init();

    lv_indev_t *enc = lv_port_indev_init();
    lv_group_t *grp = lv_group_create();
    lv_group_set_default(grp);
    lv_indev_set_group(enc, grp);

    screen_menu_load();
}

void gui_task_run(void)
{
    lv_timer_handler();
}
