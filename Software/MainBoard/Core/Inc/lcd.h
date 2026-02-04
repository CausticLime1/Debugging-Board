/*
 * lcd.h
 *
 *  Created on: Feb 2, 2026
 *      Author: ryans
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "main.h"

/* --- Hardware Configuration --- */
#define LCD_SPI_HANDLE      hspi3

/* UPDATED PINOUT: CS=PD4, DC=PD5, RST=PD6 */
#define LCD_CS_PORT         GPIOD
#define LCD_CS_PIN          GPIO_PIN_4

#define LCD_DC_PORT         GPIOD
#define LCD_DC_PIN          GPIO_PIN_5

#define LCD_RST_PORT        GPIOD
#define LCD_RST_PIN         GPIO_PIN_6

/* --- Macros (No changes needed) --- */
#define LCD_CS_LOW()        HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()       HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_DC_CMD()        HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_DATA()       HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET)
#define LCD_RST_LOW()       HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET)
#define LCD_RST_HIGH()      HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET)

/* Colors */
#define BLACK   0x0000
#define WHITE   0xFFFF
/* ... other defines ... */

/* Function Prototypes */
void LCD_Init(void);
void LCD_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void LCD_DrawChar(uint16_t x, uint16_t y, uint8_t num, uint16_t color, uint16_t bg);
void LCD_DrawNumber(uint16_t x, uint16_t y, int32_t number);
void LCD_Clear(uint16_t color);

#endif /* INC_LCD_H_ */
