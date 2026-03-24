//
// Created by luxin on 2026/3/21.
//

#ifndef PATROLBOT_PID_H
#define PATROLBOT_PID_H
typedef struct {
    float Kp;
    float Kd;
    float last_error;
} PID_Controller;

void PID_Init(PID_Controller *pid, float p, float d);
float PID_Calc_Yaw(PID_Controller *pid, float current_yaw, float target_yaw);
#endif //PATROLBOT_PID_H