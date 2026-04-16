//
// Created by luxin on 2026/3/24.
//
#include "chassis_app.h"
#include "motor_driver.h"
#include "encoder.h"
#include "pid.h"
#include "system_def.h"
#include "cmsis_os.h"
#include "track_app.h"

static PID_Controller yaw_pid;
static int cruise_base_pwm = 250; // 标准巡航速度
static int rear_offset = 30;      // 维持后轮补偿

// 声明外部变量（确保与 track_app.c 共享）
extern uint8_t is_lost_line;
extern float last_valid_error;
extern volatile float current_yaw;
extern volatile float target_yaw;

void Chassis_App_Init(void) {
    R3X_Encoder_Init();
    // P=10.0, I=0.0, D=2.0, max_i=100.0, max_out=150.0
    // 这里的 max_out=150 决定了 PID 纠偏的最大差速，配合 250 基速很安全
    PID_Init(&yaw_pid, 10.0f, 0.0f, 2.0f, 100.0f, 150.0f);
    osDelay(500);
    target_yaw = current_yaw;
}

void Chassis_App_TaskLoop(void) {
    // 1. 【熔断防线】
    if (system_alarm_flag == 1) {
        R3X_Set_Speed(MOTOR_FL, 0); R3X_Set_Speed(MOTOR_RL, 0);
        R3X_Set_Speed(MOTOR_FR, 0); R3X_Set_Speed(MOTOR_RR, 0);
        return;
    }

    // 2. 【丢线自旋状态】
    // 此时 TrackTask 不会发 target_yaw 增量，这里执行开环原地搜线
    if (is_lost_line == 1) {
        if (last_valid_error <= 0.0f) {
            // 向左扫
            R3X_Set_Speed(MOTOR_FL, -200); R3X_Set_Speed(MOTOR_RL, -200);
            R3X_Set_Speed(MOTOR_FR, 200);  R3X_Set_Speed(MOTOR_RR, 200);
        } else {
            // 向右扫
            R3X_Set_Speed(MOTOR_FL, 200);  R3X_Set_Speed(MOTOR_RL, 200);
            R3X_Set_Speed(MOTOR_FR, -200); R3X_Set_Speed(MOTOR_RR, -200);
        }
        return;
    }

    // 3. 【常规巡航：内环 PID 执行】
    // 计算 PID 修正量
    float correction = PID_Calc_Yaw(&yaw_pid, current_yaw, target_yaw);

    // ==========================================
    // 【核心改动：动态基速分发】
    // 根据 TrackTask 算出的质心误差，动态调整“油门”大小
    // ==========================================
    int current_base = cruise_base_pwm;
    float abs_err = (last_valid_error >= 0) ? last_valid_error : -last_valid_error;

    if (abs_err > 3.0f) {
        // 误差极大（只有 1、2 号或 7、8 号亮）：降速至 80，全神贯注转向
        current_base = 80;
    } else if (abs_err > 1.5f) {
        // 中等偏离：降速至 160
        current_base = 160;
    }

    // 4. 最终 PWM 混合
    int left_pwm  = current_base - (int)correction;
    int right_pwm = current_base + (int)correction;

    // 输出到硬件
    R3X_Set_Speed(MOTOR_FL, left_pwm);
    R3X_Set_Speed(MOTOR_RL, left_pwm - rear_offset);
    R3X_Set_Speed(MOTOR_FR, right_pwm);
    R3X_Set_Speed(MOTOR_RR, right_pwm - rear_offset);
}