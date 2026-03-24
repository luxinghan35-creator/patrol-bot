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

void R3X_Init(void);

void R3X_Set_Speed(MotorID_t id, int16_t speed);

void R3X_Drive_Tank(int16_t speed_L, int16_t speed_R);
#endif //PATROLBOT_MOTOR_DRIVER_H