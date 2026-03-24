//
// Created by luxin on 2026/3/21.
//
#include "encoder.h"
#include "tim.h"

/**
 * @brief   硬件编码器启动实现
 * @retval  None
 */
void R3X_Encoder_Init(void) {
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 启动左轮
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // 启动右轮
}

/**
 * @brief   左轮速度读取与清零实现
 * @retval  int16_t 采样周期内的脉冲变化量
 */
int16_t R3X_Get_Left_Speed(void) {
    int16_t speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);// 1. 读取当前定时器计数值，强转为有符号 16 位以利用补码特性解析正反转
    __HAL_TIM_SET_COUNTER(&htim3, 0);// 2. 读取完成后立即将计数器清零，为下一周期的增量采样做好准备
    return speed;
}
/**
 * @brief   右轮速度读取与清零实现
 * @retval  int16_t 采样周期内的脉冲变化量
 */
int16_t R3X_Get_Right_Speed(void) {
    int16_t speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    return speed;
}