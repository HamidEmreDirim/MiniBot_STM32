/*
 * motorDriver.c
 *
 * Created on: Oct 27, 2025
 * Author: emre
 */

#include "motorDriver.h"

void Motor_Init(void) {
    // 1. Start all PWM Channels
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // M2 L
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // M2 R
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // M1 R
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // M1 L

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // M3 L
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // M3 R
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // M4 R
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // M4 L

    // 2. IMPORTANT: Enable Main Output (MOE) for Advanced Timer 1
    // Without this, M3 and M4 will not spin.
    __HAL_TIM_MOE_ENABLE(&htim1);
}

void Motor_Move(uint8_t id, int direction, uint32_t intensity, int use_percent) {
    if (id == 0 || id > MOTOR_COUNT) return;

    // Adjust ID to 0-index
    const MotorPWM *m = &s_motors[id - 1];
    uint32_t arr = ARR(m->htim);

    // 1. Calculate Duty Cycle
    uint32_t duty;
    if (use_percent) {
        if (intensity > 100u) intensity = 100u;
        duty = (uint32_t)((intensity * (uint64_t)arr + 50u) / 100u);
    } else {
        duty = CLAMP_U32(intensity, 0u, arr);
    }

    // 2. Set BOTH Enables HIGH (As per your Logic: 1, 1, PWM, 0)
    // We enable the driver regardless of direction.
    HAL_GPIO_WritePin(m->en_fwd_port, m->en_fwd_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->en_rev_port, m->en_rev_pin, GPIO_PIN_SET);

    // 3. Apply PWM based on Direction
    if (direction == 1) {
        // FORWARD: Fwd PWM = Duty, Rev PWM = 0
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_rev, 0u);
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_fwd, duty);
    } else {
        // BACKWARD: Fwd PWM = 0, Rev PWM = Duty
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_fwd, 0u);
        __HAL_TIM_SET_COMPARE(m->htim, m->ch_rev, duty);
    }
}
