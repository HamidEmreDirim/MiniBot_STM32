/*
 * motorDriver.h
 *
 * Created on: Oct 27, 2025
 * Author: emre
 */

#ifndef SRC_MOTORDRIVER_H_
#define SRC_MOTORDRIVER_H_

#include "main.h" // Required for GPIO Pin Definitions and HAL types

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

// ---- Struct Update: Includes Enable Pins for Dual-Enable Logic ----
typedef struct {
    TIM_HandleTypeDef *htim;
    // Forward (Left) Channel & Enable
    uint32_t ch_fwd;
    GPIO_TypeDef *en_fwd_port;
    uint16_t en_fwd_pin;

    // Reverse (Right) Channel & Enable
    uint32_t ch_rev;
    GPIO_TypeDef *en_rev_port;
    uint16_t en_rev_pin;
} MotorPWM;

/* * MAPPING CONFIGURATION
 * Based on Test Code: Left=Forward, Right=Reverse
 */
static const MotorPWM s_motors[] = {
    /* M1: Left(CH4, PD12), Right(CH3, PD14) */
    { &htim3, TIM_CHANNEL_4, M1_L_EN_GPIO_Port, M1_L_EN_Pin, TIM_CHANNEL_3, M1_R_EN_GPIO_Port, M1_R_EN_Pin },

    /* M2: Left(CH1, PD15), Right(CH2, PD13) */
    { &htim3, TIM_CHANNEL_1, M2_L_EN_GPIO_Port, M2_L_EN_Pin, TIM_CHANNEL_2, M2_R_EN_GPIO_Port, M2_R_EN_Pin },

    /* M3: Left(CH1, PD10), Right(CH2, PB13) */
    { &htim1, TIM_CHANNEL_1, M3_L_EN_GPIO_Port, M3_L_EN_Pin, TIM_CHANNEL_2, M3_R_EN_GPIO_Port, M3_R_EN_Pin },

    /* M4: Left(CH4, PB12), Right(CH3, PD11) */
    { &htim1, TIM_CHANNEL_4, M4_L_EN_GPIO_Port, M4_L_EN_Pin, TIM_CHANNEL_3, M4_R_EN_GPIO_Port, M4_R_EN_Pin },
};

#define MOTOR_COUNT   ((uint8_t)(sizeof(s_motors)/sizeof(s_motors[0])))
#define ARR(_htim)    (__HAL_TIM_GET_AUTORELOAD((_htim)))
#define CLAMP_U32(x, lo, hi) (( (x) < (lo) ) ? (lo) : ( (x) > (hi) ? (hi) : (x) ))

void Motor_Move(uint8_t id, int direction, uint32_t intensity, int use_percent);
void Motor_Init(void);

#endif /* SRC_MOTORDRIVER_H_ */
