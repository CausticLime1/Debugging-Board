#include "lvgl.h"
#include "src/drivers/display/st7789/lv_st7789.h"
#include "lv_port_disp.h"
#include "stm32h7xx_hal.h"

/* Display control GPIO */
#define DISP_CS_PORT    GPIOD
#define DISP_CS_PIN     GPIO_PIN_4
#define DISP_DC_PORT    GPIOD
#define DISP_DC_PIN     GPIO_PIN_5
#define DISP_RST_PORT   GPIOD
#define DISP_RST_PIN    GPIO_PIN_6

extern SPI_HandleTypeDef hspi3;

/*
 * Draw buffers: placed in RAM_D2 via linker section so DMA1 can reach them
 * without cache coherency issues. Two buffers of 320×24 pixels = 15,360 bytes
 * each, 30,720 bytes total inside the 32K RAM_D2 region.
 */
static uint16_t __attribute__((section(".dma_buf"), aligned(32))) lv_buf1[320 * 24];
static uint16_t __attribute__((section(".dma_buf"), aligned(32))) lv_buf2[320 * 24];

static lv_display_t *g_disp = nullptr;

/* --------------------------------------------------------------------------
 * send_cmd_cb — synchronous polling SPI for commands and parameters.
 * Called by the LVGL ST7789 driver for init sequences and address-window cmds.
 * -------------------------------------------------------------------------- */
static void disp_send_cmd(lv_display_t * /*disp*/,
                           const uint8_t *cmd, size_t csz,
                           const uint8_t *param, size_t psz)
{
    HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS_PIN, GPIO_PIN_RESET); // CS↓
    HAL_GPIO_WritePin(DISP_DC_PORT, DISP_DC_PIN, GPIO_PIN_RESET); // DC=CMD
    HAL_SPI_Transmit(&hspi3, const_cast<uint8_t *>(cmd), static_cast<uint16_t>(csz), HAL_MAX_DELAY);
    if (param && psz) {
        HAL_GPIO_WritePin(DISP_DC_PORT, DISP_DC_PIN, GPIO_PIN_SET); // DC=DATA
        HAL_SPI_Transmit(&hspi3, const_cast<uint8_t *>(param), static_cast<uint16_t>(psz), HAL_MAX_DELAY);
    }
    HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS_PIN, GPIO_PIN_SET); // CS↑
}

/* --------------------------------------------------------------------------
 * disp_send_color — non-blocking DMA SPI for pixel data.
 * CS is held low; released in HAL_SPI_TxCpltCallback after DMA drains.
 * -------------------------------------------------------------------------- */
static void disp_send_color(lv_display_t * /*disp*/,
                             const uint8_t *cmd, size_t csz,
                             uint8_t *param, size_t psz)
{
    HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS_PIN, GPIO_PIN_RESET); // CS↓
    HAL_GPIO_WritePin(DISP_DC_PORT, DISP_DC_PIN, GPIO_PIN_RESET); // DC=CMD
    HAL_SPI_Transmit(&hspi3, const_cast<uint8_t *>(cmd), static_cast<uint16_t>(csz), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(DISP_DC_PORT, DISP_DC_PIN, GPIO_PIN_SET); // DC=DATA
    // Start DMA — CS released and flush_ready() signalled in TxCpltCallback
    HAL_SPI_Transmit_DMA(&hspi3, param, static_cast<uint16_t>(psz));
}

/* --------------------------------------------------------------------------
 * HAL_SPI_TxCpltCallback — overrides the __weak HAL symbol.
 * Called from DMA1_Stream4 IRQ context when pixel DMA transfer completes.
 * Must be extern "C" so the HAL (compiled as C) can call it by name.
 * -------------------------------------------------------------------------- */
extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3) {
        // HAL waits for SPI_FLAG_TXC before calling this callback,
        // so the shift register is already drained — CS can rise immediately.
        HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS_PIN, GPIO_PIN_SET); // CS↑
        lv_display_flush_ready(g_disp);
    }
}

/* --------------------------------------------------------------------------
 * lv_port_disp_init — call once from gui_init().
 * -------------------------------------------------------------------------- */
void lv_port_disp_init(void)
{
    // Hardware reset
    HAL_GPIO_WritePin(DISP_RST_PORT, DISP_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(DISP_RST_PORT, DISP_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(120);

    /*
     * Create the LVGL ST7789 display object.
     * The panel is physically 240×320 portrait; we configure it as 320×240
     * landscape. The LVGL ST7789 driver handles the MADCTL axis swap.
     * If the image appears mirrored, add LV_LCD_FLAG_MIRROR_X / MIRROR_Y.
     */
    g_disp = lv_st7789_create(320, 240, LV_LCD_FLAG_NONE,
                               disp_send_cmd, disp_send_color);

    // Most ST7789 panels require display inversion for correct colors.
    lv_st7789_set_invert(g_disp, true);

    // Set MADCTL to match the old library's rotation=1 (landscape, MY|MV = 0xA0).
    // Physical panel is 240×320 portrait; MV swaps axes so LVGL's 320-wide rows map
    // to the 320-row physical VRAM. MY flips the page direction to match rotation 1.
    // VARIANT A (try first — matches old library exactly):
    lv_lcd_generic_mipi_set_address_mode(g_disp, false, true, true, false);
    // VARIANT B (if A is upside-down/mirrored — rotation 3 style, MX|MV = 0x60):
    // lv_lcd_generic_mipi_set_address_mode(g_disp, true, false, true, false);

    lv_display_set_buffers(g_disp,
                           lv_buf1, lv_buf2,
                           sizeof(lv_buf1),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
}
