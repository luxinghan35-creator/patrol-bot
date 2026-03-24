//
// Created by luxin on 2026/3/21.
//
#include "encoder.h"
#include "tim.h"

void R3X_Encoder_Init(void) {
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 启动左轮
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // 启动右轮
}

// 读取速度并自动清零，外部直接拿走数据，不需要管底层定时器
int16_t R3X_Get_Left_Speed(void) {
    int16_t speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    return speed;
}

int16_t R3X_Get_Right_Speed(void) {
    int16_t speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    return speed;
}