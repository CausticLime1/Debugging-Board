#include "lv_port_indev.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_cortex.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_tim.h"
#include <stdint.h>

#define GPIO_DESC(port, pin)	(port),(pin)
#define DEBUG_LED_gpio				GPIO_DESC(GPIOE, GPIO_PIN_14)
#define LED1					        GPIO_DESC(GPIOE, GPIO_PIN_15)
#define LED2					        DEBUG_LED_gpio
#define LED3					        GPIO_DESC(GPIOA, GPIO_PIN_10)
#define toggle_LED(ledx)		  HAL_GPIO_TogglePin(ledx)

#define BUTTON_gpio           GPIO_DESC(GPIOD, GPIO_PIN_11)
#define ENC_A_gpio            GPIO_DESC(GPIOD, GPIO_PIN_13)
#define ENC_B_gpio            GPIO_DESC(GPIOD, GPIO_PIN_14)
#define ENC_BUTTON_gpio       GPIO_DESC(GPIOD, GPIO_PIN_15)
#define BUTTON_pin            GPIO_PIN_11
#define ENC_A_pin             GPIO_PIN_13
#define ENC_B_pin             GPIO_PIN_14
#define ENC_BUTTON_pin        GPIO_PIN_15

#define APB1_TICKS_PER_US     275u // cubemx for TIM7

#define ENCODER_KNOB_DEBOUNCE_US    100u
#define ENCODER_BUTTON_DEBOUNCE_US  10000u
#define ENC_BUTTON_PRESSED_STATE    GPIO_PIN_RESET
#define ENC_BUTTON_RELEASED_STATE   GPIO_PIN_SET

typedef enum { Enc_None, Enc_Knob, Enc_Button } Encoder_Action;

extern TIM_HandleTypeDef htim7;

volatile Encoder_Action encoder_action = Enc_None;

void tim7_start_oneshot_us(uint16_t us);

const int8_t qdr_table[16] = {
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0
};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == BUTTON_pin) return;
  if (encoder_action != Enc_None) return;
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

  if (GPIO_Pin == ENC_A_pin || GPIO_Pin == ENC_B_pin) {
    encoder_action = Enc_Knob;
    tim7_start_oneshot_us(ENCODER_KNOB_DEBOUNCE_US);
  } else if (GPIO_Pin == ENC_BUTTON_pin) {
    encoder_action = Enc_Button;
    tim7_start_oneshot_us(ENCODER_BUTTON_DEBOUNCE_US);
  }
}

void tim7_start_oneshot_us(uint16_t us) {
  __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
  __HAL_TIM_SET_COUNTER(&htim7, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim7, (us > 0) ? (us * APB1_TICKS_PER_US - 1) : 0);
  __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim7);
}

void tim7_PeriodElapsedCallback_action() {
  static uint8_t prev_ab = 0xFF;
  static int8_t knob_accum = 0;
  static uint8_t prev_button_state = 0xFF;

  HAL_TIM_Base_Stop_IT(&htim7);

  if (encoder_action == Enc_Knob) {
    uint8_t a = (HAL_GPIO_ReadPin(ENC_A_gpio));
    uint8_t b = (HAL_GPIO_ReadPin(ENC_B_gpio));
    uint8_t curr_ab = (a << 1) | b;
    if (prev_ab == 0xFF) prev_ab = curr_ab;

    int8_t knob_delta = qdr_table[(prev_ab << 2) | curr_ab];
    prev_ab = curr_ab;

    if (knob_delta != 0) {
      if ((knob_accum > 0 && knob_delta < 0) || (knob_accum < 0 && knob_delta > 0))
        knob_accum = 0;
      knob_accum += knob_delta;
    }
    if (knob_accum >= 4) {
      knob_accum = 0;
      enc_report_cw();
      toggle_LED(LED3);
    } else if (knob_accum <= -4) {
      knob_accum = 0;
      enc_report_ccw();
      toggle_LED(LED2);
    }
  } else if (encoder_action == Enc_Button) {
    uint8_t button_state = HAL_GPIO_ReadPin(ENC_BUTTON_gpio);

    if (prev_button_state == 0xFF) {
      prev_button_state = button_state;
    } else {
      if ((prev_button_state == ENC_BUTTON_PRESSED_STATE) &&
          (button_state == ENC_BUTTON_RELEASED_STATE)) {
        enc_report_btn();
        toggle_LED(LED1);
      }
      prev_button_state = button_state;
    }
  }
  
  encoder_action = Enc_None;
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
