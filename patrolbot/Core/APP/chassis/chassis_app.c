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

/** * @brief 底盘偏航角 PID 控制器实例
 * @note  【核心架构】必须为 static 全局静态持久化变量，严禁放在循环内部，防止积分(I)与微分(D)状态在每次循环中被重置丢失。
 */
static PID_Controller yaw_pid;
static int base_pwm = 250; //基础占空比
static int rear_offset = 30; //后轮相对于前轮的速度补偿，防止打滑

/**
 * @brief   底盘控制系统初始化
 * @details
 * 1. 唤醒底层双路轮速编码器外设
 * 2. 实例化航向闭环 PID 控制器并装载 Kp, Kd 参数
 * 3. 阻塞等待 500ms 确保陀螺仪上电完成及零偏校准
 * 4. 锁定开机航向为系统目标航向 (target_yaw)
 *
 * @retval  None
 */
void Chassis_App_Init(void) {
    R3X_Encoder_Init();
    PID_Init(&yaw_pid, 10.0f, 0.0f, 2.0f, 100.0f, 150.0f);
    osDelay(500); // 必须在此处等待传感器上电稳定
    target_yaw = current_yaw;
}

/**
 * @brief   底盘控制核心任务单次扫描循环
 * @details
 * 包含两大核心工序：
 * 1. 【安全熔断层】原子级查询 system_alarm_flag，若触发则直接将四个独立电机的 PWM 强行灌零锁死，直接 return 跳过后续所有算法计算。
 * 2. 【运动控制层】读取当前航向角，通过 PID 算法解算航向偏差修正量，并将其通过混控逻辑分配至四轮差速底盘。
 *
 * @attention 严禁在此函数内加入大于控制周期的阻塞延时，否则将破坏 PID 积分环节的 dt (时间微分) 计算，导致系统震荡失控。
 * @retval    None
 */
void Chassis_App_TaskLoop(void) {
    // 0. 声明跨任务共享的全局变量 (这些由 Track_App 在低优先级任务中异步更新)
    extern uint8_t is_lost_line;
    extern float last_valid_error;
    extern volatile float current_yaw;
    extern volatile float target_yaw;

    // 1. 【最高优先级防线】：系统超温熔断拦截
    if (system_alarm_flag == 1) {
        R3X_Set_Speed(MOTOR_FL, 0);
        R3X_Set_Speed(MOTOR_RL, 0);
        R3X_Set_Speed(MOTOR_FR, 0);
        R3X_Set_Speed(MOTOR_RR, 0);
        return; // 发现异常直接物理掐断，绝不执行后续运动控制
    }

    // 2. 【状态机强控】：直角弯 / 被踹飞导致全白丢线
    if (is_lost_line == 1) {
        // 瞬间剥夺常规 PID 路权，直接开环暴力自旋！
        // 此时控制指令从 MotionTask 下发，独占电机，绝对不会发生竞态震荡。
        if (last_valid_error <= 0.0f) {
            // 丢线前黑线在左侧 -> 必须向左暴力自旋
            // 左转：左侧履带倒车，右侧履带前进
            R3X_Set_Speed(MOTOR_FL, -250);
            R3X_Set_Speed(MOTOR_RL, -250 - rear_offset);
            R3X_Set_Speed(MOTOR_FR, 250);
            R3X_Set_Speed(MOTOR_RR, 250 - rear_offset);
        } else {
            // 丢线前黑线在右侧 -> 必须向右暴力自旋
            R3X_Set_Speed(MOTOR_FL, 250);
            R3X_Set_Speed(MOTOR_RL, 250 - rear_offset);
            R3X_Set_Speed(MOTOR_FR, -250);
            R3X_Set_Speed(MOTOR_RR, -250 - rear_offset);
        }
        return; // 强控生效，直接 return，跳过后面的常规 PID 解算！
    }

    // 3. 【常规巡航】：内环老司机接管
    // 闭环数据采集 (暂存备用)
    int16_t speed_L = R3X_Get_Left_Speed();
    int16_t speed_R = R3X_Get_Right_Speed();

    // 执行带抗积分饱和的姿态 PID 解算
    float correction = PID_Calc_Yaw(&yaw_pid, current_yaw, target_yaw);

    // 4. 动力分配与 PWM 输出
    // 推演：如果目标在左边(需左转)，Yaw应增大，Target > Current，correction 为正。
    // 左转需要左轮减速，右轮加速。所以 left_pwm 是减，right_pwm 是加。极性完美。
    int left_pwm  = base_pwm - (int)correction;
    int right_pwm = base_pwm + (int)correction;

    // 5. 后轮相对于前轮的速度补偿，防止打滑
    R3X_Set_Speed(MOTOR_FL, left_pwm);
    R3X_Set_Speed(MOTOR_RL, left_pwm - rear_offset);
    R3X_Set_Speed(MOTOR_FR, right_pwm);
    R3X_Set_Speed(MOTOR_RR, right_pwm - rear_offset);
}