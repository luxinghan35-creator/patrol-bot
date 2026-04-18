//
// Created by luxin on 2026/3/24.
//
#include "chassis_app.h"
#include "motor_driver.h"
#include "encoder.h"
#include "pid.h"
#include "system_def.h"  // 提供 current_yaw, target_yaw, system_alarm_flag
#include "cmsis_os.h"
#include "track_app.h"   // 引入全新的 Getter 通行证！

// ==========================================================
// 物理常量宏定义 (消灭魔法数字)
// ==========================================================
#define CRUISE_BASE_PWM   250   // 直线最高巡航基础速度
#define REAR_WHEEL_OFFSET 30    // 维持重型底盘后轮补偿防打滑
#define SPEED_DROP_MID    160   // 中度偏离时的降维速度
#define SPEED_DROP_MAX    80    // 极限偏离时的断崖式降速

static PID_Controller yaw_pid;

void Chassis_App_Init(void) {
    R3X_Encoder_Init();
    // 限制 I 积分墙为 100，限制最大差速输出为 150，保护减速箱
    PID_Init(&yaw_pid, 10.0f, 0.0f, 2.0f, 100.0f, 150.0f);
    osDelay(500);
    target_yaw = current_yaw;
}

void Chassis_App_TaskLoop(void) {
    // 1. 【最高物理防线】：超温一刀切
    if (system_alarm_flag == 1) {
        R3X_Set_Speed(MOTOR_FL, 0); R3X_Set_Speed(MOTOR_RL, 0);
        R3X_Set_Speed(MOTOR_FR, 0); R3X_Set_Speed(MOTOR_RR, 0);
        osDelay(20);
        return;
    }

    // 2. 【协作路权避让】：领航员在强控过直角弯，内环死锁避让！
    if (Track_Is_Turning() == 1) {
        osDelay(10); // 绝对防饿死看门狗
        return;
    }

    // 3. 【丢线自旋开环保护】
    if (Track_Is_Lost() == 1) {
        if (Track_Get_Error() <= 0.0f) {
            R3X_Set_Speed(MOTOR_FL, -200); R3X_Set_Speed(MOTOR_RL, -200);
            R3X_Set_Speed(MOTOR_FR, 200);  R3X_Set_Speed(MOTOR_RR, 200);
        } else {
            R3X_Set_Speed(MOTOR_FL, 200);  R3X_Set_Speed(MOTOR_RL, 200);
            R3X_Set_Speed(MOTOR_FR, -200); R3X_Set_Speed(MOTOR_RR, -200);
        }
        osDelay(10);
        return;
    }

    // 4. 【常规闭环降速纠偏】(核心降维打击区)
    float correction = PID_Calc_Yaw(&yaw_pid, current_yaw, target_yaw);

    int current_base = CRUISE_BASE_PWM;
    float err = Track_Get_Error();
    float abs_err = (err >= 0) ? err : -err;

    // 工业级动态推力分配：误差越大，越要收油门！
    if (abs_err > 3.0f) {
        current_base = SPEED_DROP_MAX;
    } else if (abs_err > 1.5f) {
        current_base = SPEED_DROP_MID;
    }

    int left_pwm  = current_base - (int)correction;
    int right_pwm = current_base + (int)correction;

    R3X_Set_Speed(MOTOR_FL, left_pwm);
    R3X_Set_Speed(MOTOR_RL, left_pwm - REAR_WHEEL_OFFSET);
    R3X_Set_Speed(MOTOR_FR, right_pwm);
    R3X_Set_Speed(MOTOR_RR, right_pwm - REAR_WHEEL_OFFSET);

    osDelay(10); // 常规巡航出口，绝不饿死 CPU
}