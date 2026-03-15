#pragma once
#include "stm32h7xx_hal.h"

/* --- Display SPI control lines (SPI3) --- */
#define DISP_CS_PORT    GPIOD
#define DISP_CS_PIN     GPIO_PIN_4
#define DISP_DC_PORT    GPIOD
#define DISP_DC_PIN     GPIO_PIN_5
#define DISP_RST_PORT   GPIOD
#define DISP_RST_PIN    GPIO_PIN_6

/* --- Rotary encoder (EXTI15_10, TIM7 debounce) --- */
#define ENC_A_PORT      GPIOD
#define ENC_A_PIN       GPIO_PIN_13
#define ENC_B_PORT      GPIOD
#define ENC_B_PIN       GPIO_PIN_14
#define ENC_BUTTON_PORT GPIOD
#define ENC_BUTTON_PIN  GPIO_PIN_15

/* --- Auxiliary button (EXTI11) --- */
#define BUTTON_PORT     GPIOD
#define BUTTON_PIN      GPIO_PIN_11

/* --- LEDs --- */
#define LED1_PORT   GPIOE
#define LED1_PIN    GPIO_PIN_15
#define LED2_PORT   GPIOE
#define LED2_PIN    GPIO_PIN_14
#define LED3_PORT   GPIOA
#define LED3_PIN    GPIO_PIN_10

#define TOGGLE_LED(x)   HAL_GPIO_TogglePin(x##_PORT, x##_PIN)

#ifdef __cplusplus
extern "C" {
#endif

void tim7_PeriodElapsedCallback_action(void);

#ifdef __cplusplus
}
#endif
