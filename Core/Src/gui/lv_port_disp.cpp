#include "lvgl.h"
#include "src/drivers/display/st7789/lv_st7789.h"
#include "lv_port_disp.h"
#include "board.h"

extern SPI_HandleTypeDef hspi3;

namespace {

/*
 * Draw buffers in RAM_D1 (.dma_buf linker section, after the 64KB LVGL heap).
 * CPU and DMA1 both access AXI SRAM via D1-domain paths — no cross-domain
 * write buffer between them, so DMA reads always see the CPU's latest writes.
 * Two buffers × 320×24 pixels × 2 bytes = 30,720 bytes.
 */
uint16_t __attribute__((section(".dma_buf"), aligned(32))) lv_buf1[320 * 24];
uint16_t __attribute__((section(".dma_buf"), aligned(32))) lv_buf2[320 * 24];

lv_display_t *g_disp = nullptr;

/* Synchronous polling SPI — used for init sequences and address-window commands. */
void disp_send_cmd(lv_display_t * /*disp*/,
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

/* Non-blocking DMA SPI — CS held low until TxCpltCallback. */
void disp_send_color(lv_display_t * /*disp*/,
                     const uint8_t *cmd, size_t csz,
                     uint8_t *param, size_t psz)
{
    HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS_PIN, GPIO_PIN_RESET); // CS↓
    HAL_GPIO_WritePin(DISP_DC_PORT, DISP_DC_PIN, GPIO_PIN_RESET); // DC=CMD
    HAL_SPI_Transmit(&hspi3, const_cast<uint8_t *>(cmd), static_cast<uint16_t>(csz), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(DISP_DC_PORT, DISP_DC_PIN, GPIO_PIN_SET); // DC=DATA
    HAL_SPI_Transmit_DMA(&hspi3, param, static_cast<uint16_t>(psz));
}

} // namespace

/* Overrides __weak HAL symbol — must be extern "C" for the HAL to find it.
 * Called from DMA IRQ context when the pixel transfer completes. */
extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3) {
        HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS_PIN, GPIO_PIN_SET); // CS↑
        lv_display_flush_ready(g_disp);
    }
}

void lv_port_disp_init(void)
{
    HAL_GPIO_WritePin(DISP_RST_PORT, DISP_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(DISP_RST_PORT, DISP_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(120);

    /*
     * Panel is physically 240×320 portrait; configured as 320×240 landscape.
     * MADCTL MY|MV (0xA0) matches the old library's rotation=1.
     */
    g_disp = lv_st7789_create(320, 240, LV_LCD_FLAG_NONE, disp_send_cmd, disp_send_color);
    lv_st7789_set_invert(g_disp, true);
    lv_lcd_generic_mipi_set_address_mode(g_disp, false, true, true, false);

    lv_display_set_buffers(g_disp, lv_buf1, lv_buf2, sizeof(lv_buf1),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
}
