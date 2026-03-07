#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_cortex.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_tim.h"
#include <stdint.h>

#define GPIO_DESC(port, pin)	(port),(pin)
#define DEBUG_LED_P				    GPIO_DESC(GPIOE, GPIO_PIN_14)
#define LED1					        GPIO_DESC(GPIOE, GPIO_PIN_15)
#define LED2					        DEBUG_LED_P
#define LED3					        GPIO_DESC(GPIOA, GPIO_PIN_10)
#define toggle_LED(ledx)		  HAL_GPIO_TogglePin(ledx)

#define ENCODER_KNOB_DEBOUNCE_US    500u
#define ENCODER_BUTTON_DEBOUNCE_US  10000u

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_11) return;
  if (encoder_action != Enc_None) return;
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

  if (GPIO_Pin == GPIO_PIN_13 || GPIO_Pin == GPIO_PIN_14) {
    encoder_action = Enc_Knob;
    tim7_start_oneshot_us(ENCODER_KNOB_DEBOUNCE_US);
  } else if (GPIO_Pin == GPIO_PIN_15) {
    encoder_action = Enc_Button;
    tim7_start_oneshot_us(ENCODER_BUTTON_DEBOUNCE_US);
  }
}

void tim7_start_oneshot_us(uint16_t us) {
  __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
  __HAL_TIM_SET_COUNTER(&htim7, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim7, (us > 0) ? (us - 1) : 0);
  __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim7);
}