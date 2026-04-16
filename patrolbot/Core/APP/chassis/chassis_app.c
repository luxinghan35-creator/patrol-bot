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
static int base_pwm = 250;
static int rear_offset = 30; // 维持后轮补偿防打滑

void Chassis_App_Init(void) {
    R3X_Encoder_Init();
    PID_Init(&yaw_pid, 10.0f, 0.0f, 2.0f, 100.0f, 150.0f);
    osDelay(500);
    target_yaw = current_yaw;
}

void Chassis_App_TaskLoop(void) {
    extern uint8_t is_lost_line;
    extern float last_valid_error;
    extern volatile float current_yaw;
    extern volatile float target_yaw;

    // 1. 【最高安全防线：超温熔断】
    if (system_alarm_flag == 1) {
        R3X_Set_Speed(MOTOR_FL, 0); R3X_Set_Speed(MOTOR_RL, 0);
        R3X_Set_Speed(MOTOR_FR, 0); R3X_Set_Speed(MOTOR_RR, 0);
        return;
    }

    // 2. 【状态机强控：全白丢线开环自旋保护】
    if (is_lost_line == 1) {
        if (last_valid_error <= 0.0f) {
            R3X_Set_Speed(MOTOR_FL, -250);
            R3X_Set_Speed(MOTOR_RL, -250 - rear_offset);
            R3X_Set_Speed(MOTOR_FR, 250);
            R3X_Set_Speed(MOTOR_RR, 250 - rear_offset);
        } else {
            R3X_Set_Speed(MOTOR_FL, 250);
            R3X_Set_Speed(MOTOR_RL, 250 - rear_offset);
            R3X_Set_Speed(MOTOR_FR, -250);
            R3X_Set_Speed(MOTOR_RR, -250 - rear_offset);
        }
        return;
    }

    // 3. 【常规巡航：带降维打击的内环闭环】
    int16_t speed_L = R3X_Get_Left_Speed();
    int16_t speed_R = R3X_Get_Right_Speed();

    float correction = PID_Calc_Yaw(&yaw_pid, current_yaw, target_yaw);

    // 【动态基础推力降维】对抗偏离时的前冲惯性
    int current_base_pwm = base_pwm;
    float abs_err = last_valid_error;
    if (abs_err < 0) abs_err = -abs_err;

    if (abs_err > 3.0f) {
        // 极限偏离，砍掉 60% 的直线动力，优先供能给差速打方向
        current_base_pwm = 100;
    } else if (abs_err > 1.5f) {
        current_base_pwm = 160;
    }

    // 4. 动力分配与补偿输出
    int left_pwm  = current_base_pwm - (int)correction;
    int right_pwm = current_base_pwm + (int)correction;

    R3X_Set_Speed(MOTOR_FL, left_pwm);
    R3X_Set_Speed(MOTOR_RL, left_pwm - rear_offset);
    R3X_Set_Speed(MOTOR_FR, right_pwm);
    R3X_Set_Speed(MOTOR_RR, right_pwm - rear_offset);
}