//
// Created by luxin on 2026/3/21.
//

#ifndef PATROLBOT_PID_H
#define PATROLBOT_PID_H
typedef struct {
    float Kp;           // 比例系数：决定响应速度 (弹簧弹力)
    float Ki;           // 积分系数：消除稳态静差 (记忆力)
    float Kd;           // 微分系数：抑制超调震荡 (阻尼器)

    float integral;     // 误差累积量
    float last_error;   // 上一次周期的误差

    float max_integral; //积分器物理限幅阈值 (防 windup 灾难)
    float max_output;   // 控制器总输出物理限幅阈值 (防 PWM 溢出)
} PID_Controller;

/**
 * @brief  初始化 PID 控制器参数与限幅墙
 */
void PID_Init(PID_Controller *pid, float p, float i, float d, float max_i, float max_out);

/**
 * @brief  执行偏航角 (Yaw) 闭环解算
 * @note   内置 ±180 度数学跃变处理，专为陀螺仪欧拉角设计。
 */
float PID_Calc_Yaw(PID_Controller *pid, float current_yaw, float target_yaw);
#endif //PATROLBOT_PID_H