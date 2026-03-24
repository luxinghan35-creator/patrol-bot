//
// Created by luxin on 2026/1/31.
//

#ifndef PATROLBOT_MOTOR_DRIVER_H
#define PATROLBOT_MOTOR_DRIVER_H
#include "main.h"
/**
 * @brief 电机索引枚举
 */
typedef enum {MOTOR_FL=0, MOTOR_FR=1, MOTOR_RL=2, MOTOR_RR=3} MotorID_t;

/**
 * @brief 电机初始化
 * @retval none
 */
void R3X_Init(void);
/**
 * @brief 设置电机速度
 * @param id 电机编号 [(Motor_FL, Motor_FR, Motor_RL, Motor_RR),（分别对应左前，右前，左后，右后）]
 * @param speed 速度值，范围-1000到1000，正值前进，负值后退，0停止
 * @retval none
 */
void R3X_Set_Speed(MotorID_t id, int16_t speed);
/**
 * @brief 差速驱动控制
 * @param speed_L 左侧电机速度，范围-1000到1000，正值前进，负值后退，0停止
 * @param speed_R 右侧电机速度，范围-1000到1000，正值前进，负值后退，0停止
 * @retval none
 */
void R3X_Drive_Tank(int16_t speed_L, int16_t speed_R);
#endif //PATROLBOT_MOTOR_DRIVER_H