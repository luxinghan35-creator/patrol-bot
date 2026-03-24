//
// Created by luxin on 2026/3/24.
//
#include "chassis_app.h"
#include "motor_driver.h"
#include "encoder.h"
#include "pid.h"
#include "system_def.h"
#include "cmsis_os.h"

// 【核心修复】PID 结构体必须是静态/全局变量，保证其 I 和 D 状态在循环中绝对持久化！
static PID_Controller yaw_pid;
static int base_pwm = 500;
static int rear_offset = 30;

void Chassis_App_Init(void) {
    R3X_Encoder_Init();
    PID_Init(&yaw_pid, 10.0f, 2.0f);
    osDelay(500); // 必须在此处等待传感器上电稳定
    target_yaw = current_yaw;
}

void Chassis_App_TaskLoop(void) {
    if (system_alarm_flag == 1) {
        R3X_Set_Speed(MOTOR_FL, 0);
        R3X_Set_Speed(MOTOR_RL, 0);
        R3X_Set_Speed(MOTOR_FR, 0);
        R3X_Set_Speed(MOTOR_RR, 0);
        return; // 直接拦截，绝不执行后续算法
    }

    int16_t speed_L = R3X_Get_Left_Speed();
    int16_t speed_R = R3X_Get_Right_Speed();

    float correction = PID_Calc_Yaw(&yaw_pid, current_yaw, target_yaw);

    int left_pwm  = base_pwm - (int)correction;
    int right_pwm = base_pwm + (int)correction;

    R3X_Set_Speed(MOTOR_FL, left_pwm);
    R3X_Set_Speed(MOTOR_RL, left_pwm - rear_offset);
    R3X_Set_Speed(MOTOR_FR, right_pwm);
    R3X_Set_Speed(MOTOR_RR, right_pwm - rear_offset);
}