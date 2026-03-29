//
// Created by luxin on 2026/3/21.
//
#include "pid.h"

void PID_Init(PID_Controller *pid, float p, float i, float d, float max_i, float max_out) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;

    pid->integral = 0.0f;
    pid->last_error = 0.0f;

    pid->max_integral = max_i;
    pid->max_output = max_out;
}

float PID_Calc_Yaw(PID_Controller *pid, float current_yaw, float target_yaw) {
    float error = target_yaw - current_yaw;

    // 1. 欧拉角数学跃变处理 (防止在 ±180度 边界时发生反向死亡旋转)
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }

    // 2. 比例项 (P): 决定当下该出多大力
    float p_out = pid->Kp * error;

    // 3. 积分项 (I): 累积历史误差，专治机械静摩擦导致的微小偏移
    pid->integral += error;

    // 【核心防御】：积分抗饱和 (Anti-Windup)
    // 一旦累计的误差超过物理界限，强制切断，防止被卡住脱困后产生报复性暴走
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }
    float i_out = pid->Ki * pid->integral;

    // 4. 微分项 (D): 预测未来，提前刹车防止车头左右摇摆超调
    float d_out = pid->Kd * (error - pid->last_error);
    pid->last_error = error;

    // 5. 汇总总输出
    float correction = p_out + i_out + d_out;

    // 【核心防御】：总输出硬限幅
    // 保护底盘执行层，防止极端偏差下算出的补偿量过大，导致一侧车轮被瞬间拉反转
    if (correction > pid->max_output) {
        correction = pid->max_output;
    } else if (correction < -pid->max_output) {
        correction = -pid->max_output;
    }

    return correction;
}