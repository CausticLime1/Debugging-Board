#include "board.h"
#include "lv_port_indev.h"
#include "stm32h7xx_hal.h"

extern TIM_HandleTypeDef htim7;

namespace {

constexpr uint32_t      APB1_TICKS_PER_US          = 275u;
constexpr uint32_t      ENCODER_KNOB_DEBOUNCE_US   = 100u;
constexpr uint32_t      ENCODER_BUTTON_DEBOUNCE_US = 10000u;
constexpr GPIO_PinState ENC_BUTTON_PRESSED_STATE   = GPIO_PIN_RESET;
constexpr GPIO_PinState ENC_BUTTON_RELEASED_STATE  = GPIO_PIN_SET;

enum class Encoder_Action { None, Knob, Button, Aux };

volatile Encoder_Action encoder_action = Encoder_Action::None;

const int8_t qdr_table[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0,
};

void tim7_start_oneshot_us(uint16_t us)
{
    __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim7, (us > 0u) ? (us * APB1_TICKS_PER_US - 1u) : 0u);
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim7);
}

} // namespace

/* Overrides __weak HAL symbol — must be extern "C" for the HAL to find it. */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != ENC_A_PIN   && GPIO_Pin != ENC_B_PIN &&
        GPIO_Pin != ENC_BUTTON_PIN && GPIO_Pin != BUTTON_PIN)
        return;
    if (encoder_action != Encoder_Action::None)
        return;

    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

    if (GPIO_Pin == ENC_A_PIN || GPIO_Pin == ENC_B_PIN) {
        encoder_action = Encoder_Action::Knob;
        tim7_start_oneshot_us(ENCODER_KNOB_DEBOUNCE_US);
    } else if (GPIO_Pin == ENC_BUTTON_PIN) {
        encoder_action = Encoder_Action::Button;
        tim7_start_oneshot_us(ENCODER_BUTTON_DEBOUNCE_US);
    } else {
        encoder_action = Encoder_Action::Aux;
        tim7_start_oneshot_us(ENCODER_BUTTON_DEBOUNCE_US);
    }
}

void tim7_PeriodElapsedCallback_action(void)
{
    static uint8_t prev_ab           = 0xFF;
    static int8_t  knob_accum        = 0;
    static uint8_t prev_button_state = 0xFF;

    HAL_TIM_Base_Stop_IT(&htim7);

    if (encoder_action == Encoder_Action::Knob) {
        uint8_t a       = (uint8_t)HAL_GPIO_ReadPin(ENC_A_PORT, ENC_A_PIN);
        uint8_t b       = (uint8_t)HAL_GPIO_ReadPin(ENC_B_PORT, ENC_B_PIN);
        uint8_t curr_ab = (uint8_t)((a << 1) | b);
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
            TOGGLE_LED(LED3);
        } else if (knob_accum <= -4) {
            knob_accum = 0;
            enc_report_ccw();
            TOGGLE_LED(LED2);
        }
    } else if (encoder_action == Encoder_Action::Button) {
        uint8_t button_state = (uint8_t)HAL_GPIO_ReadPin(ENC_BUTTON_PORT, ENC_BUTTON_PIN);

        if (prev_button_state == 0xFF) {
            prev_button_state = button_state;
        } else {
            if (prev_button_state == ENC_BUTTON_PRESSED_STATE &&
                button_state      == ENC_BUTTON_RELEASED_STATE) {
                enc_report_btn();
                TOGGLE_LED(LED1);
            }
            prev_button_state = button_state;
        }
    }
    // Enc_Aux (BUTTON_PIN): placeholder, no action yet

    encoder_action = Encoder_Action::None;
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
