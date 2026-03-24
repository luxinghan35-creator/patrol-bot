//
// Created by luxin on 2026/3/21.
//
#include "pid.h"

void PID_Init(PID_Controller *pid, float p, float d) {
    pid->Kp = p;
    pid->Kd = d;
    pid->last_error = 0.0f;
}

float PID_Calc_Yaw(PID_Controller *pid, float current_yaw, float target_yaw) {
    float error = target_yaw - current_yaw;

    // 数学跃变处理
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }

    float correction = pid->Kp * error + pid->Kd * (error - pid->last_error);
    pid->last_error = error;

    return correction;
}