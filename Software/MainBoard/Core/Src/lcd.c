/*
 * lcd.c
 *
 *  Created on: Feb 2, 2026
 *      Author: ryans
 */

#include "lcd.h"
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef LCD_SPI_HANDLE;

static void LCD_WriteCommand(uint8_t cmd) {
    LCD_DC_CMD();
    LCD_CS_LOW();
    HAL_SPI_Transmit(&LCD_SPI_HANDLE, &cmd, 1, 10);
    LCD_CS_HIGH();
}

static void LCD_WriteData(uint8_t data) {
    LCD_DC_DATA();
    LCD_CS_LOW();
    HAL_SPI_Transmit(&LCD_SPI_HANDLE, &data, 1, 10);
    LCD_CS_HIGH();
}

void LCD_Init(void) {
    LCD_RST_LOW();
    HAL_Delay(50);
    LCD_RST_HIGH();
    HAL_Delay(50);

    LCD_WriteCommand(0x01); // Software Reset
    HAL_Delay(100);
    LCD_WriteCommand(0x11); // Sleep Out
    HAL_Delay(50);
    LCD_WriteCommand(0x29); // Display On
    LCD_WriteCommand(0x3A); // Pixel Format
    LCD_WriteData(0x55);    // 16-bit color

    /* CHANGE: Orientation set to Portrait (240x320) */
    LCD_WriteCommand(0x36);
    LCD_WriteData(0x08);    // Was 0x48 (Landscape). 0x08 is usually Portrait BGR.
}

void LCD_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    LCD_WriteCommand(0x2A); // Column Addr
    LCD_WriteData(x0 >> 8); LCD_WriteData(x0 & 0xFF);
    LCD_WriteData(x1 >> 8); LCD_WriteData(x1 & 0xFF);

    LCD_WriteCommand(0x2B); // Row Addr
    LCD_WriteData(y0 >> 8); LCD_WriteData(y0 & 0xFF);
    LCD_WriteData(y1 >> 8); LCD_WriteData(y1 & 0xFF);

    LCD_WriteCommand(0x2C); // Write RAM
}

void LCD_Clear(uint16_t color) {
    /* CHANGE: Swapped limits to 240x320 */
    LCD_SetAddressWindow(0, 0, 239, 319);
    LCD_DC_DATA();
    LCD_CS_LOW();

    uint8_t data[2] = {color >> 8, color & 0xFF};

    /* CHANGE: Loop count for 240x320 */
    for(long i=0; i<(240*320); i++) {
        HAL_SPI_Transmit(&LCD_SPI_HANDLE, data, 2, 10);
    }
    LCD_CS_HIGH();
}

/* ... DrawChar and DrawNumber functions remain the same ... */
/* (They will automatically work within the new window) */
const uint8_t SimpleFont[11][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x00, 0x00, 0x00, 0x00}  // Space
};

void LCD_DrawChar(uint16_t x, uint16_t y, uint8_t num, uint16_t color, uint16_t bg) {
    if(num > 10) return;
    uint8_t scale = 4;

    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = SimpleFont[num][i];
        for (uint8_t j = 0; j < 8; j++) {
            uint16_t drawColor = (line & (1 << j)) ? color : bg;
            LCD_SetAddressWindow(x + (i*scale), y + (j*scale), x + (i*scale) + scale - 1, y + (j*scale) + scale - 1);
            LCD_DC_DATA();
            LCD_CS_LOW();
            for(int k=0; k < (scale*scale); k++) {
                 uint8_t cData[2] = {drawColor >> 8, drawColor & 0xFF};
                 HAL_SPI_Transmit(&LCD_SPI_HANDLE, cData, 2, 10);
            }
            LCD_CS_HIGH();
        }
    }
}

void LCD_DrawNumber(uint16_t x, uint16_t y, int32_t number) {
    char buf[12];
    sprintf(buf, "%ld  ", number);
    for(int i=0; i<strlen(buf); i++) {
        if(buf[i] >= '0' && buf[i] <= '9') {
            LCD_DrawChar(x + (i*25), y, buf[i] - '0', WHITE, BLACK);
        } else {
            LCD_DrawChar(x + (i*25), y, 10, WHITE, BLACK);
        }
    }
}
