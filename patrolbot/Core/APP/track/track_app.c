//
// Created by luxin on 2026/3/29.
//
/**
 * @file    track_app.c
 * @brief   R3X 外环循迹状态机实现 (纯正 I2C 总线版本)
 */

#include "track_app.h"
#include "i2c.h"
#include "chassis_app.h"
#include "cmsis_os.h"
#include "motor_driver.h"

// 硬件配置 (根据手册 7.6 / 7.7)
#define GW_GRAY_ADDR 0x98
#define REG_DIGITAL  0xDD
#define REG_PING     0xAA

// 任务句柄（用于强控时点穴）
extern osThreadId_t MotionTaskHandle;

// 算法参数
static float Kp_outer = -0.6f;     // 外环比例系数
float last_valid_error = 0.0f;    // 历史残余误差
uint8_t is_lost_line = 0;         // 丢线标志位

// ==========================================================
// 1. 底层数据接口 (遵循官方 Demo & 手册)
// ==========================================================
static uint8_t Fetch_Sensor_Raw(void) {
    uint8_t data = 0;
    // 官方 Demo 强调：先写 0xDD 再读。HAL_I2C_Mem_Read 封装了此过程
    if (HAL_I2C_Mem_Read(&hi2c1, GW_GRAY_ADDR, REG_DIGITAL, I2C_MEMADD_SIZE_8BIT, &data, 1, 10) == HAL_OK) {
        // 手册 2.4：白场=1，黑场=0。此处取反，使 1=黑线，0=白底
        return (uint8_t)(~data);
    }
    return 0x55; // 通信失败返回杂波，靠惯性滑行
}

// ==========================================================
// 2. 加权质心纠错算法 (重构同仁 xunji.c 的核心)
// ==========================================================
static float Calculate_Line_Error(uint8_t sensor_bits) {
    // 权重定义：从左到右 (传感器 1-8) 对应误差 -4 到 4
    // 这样 4、5 号中间位置误差刚好为 0
    float weights[8] = {-4.5f, -3.0f, -1.5f, -0.5f, 0.5f, 1.5f, 3.0f, 4.5f};
    float sum = 0;
    int count = 0;

    for (int i = 0; i < 8; i++) {
        if (sensor_bits & (1 << i)) {
            sum += weights[i];
            count++;
        }
    }
    // 如果有探头扫到线，计算平均质心偏移；否则维持现状
    return (count > 0) ? (sum / (float)count) : last_valid_error;
}

// ==========================================================
// 3. 初始化 (同步网络诊断)
// ==========================================================
void Track_App_Init(void) {
    uint8_t ping_val = 0;
    osDelay(500); // 等待传感器内部 MCU 初始化
    // 遵循手册 7.13，Ping 不通不发车
    while (ping_val != 0x66) {
        HAL_I2C_Mem_Read(&hi2c1, GW_GRAY_ADDR, REG_PING, I2C_MEMADD_SIZE_8BIT, &ping_val, 1, 10);
        osDelay(50);
    }
}

// ==========================================================
// 4. 核心任务循环 (重构状态机)
// ==========================================================
void Track_App_TaskLoop(void) {
    uint8_t sensor_val = Fetch_Sensor_Raw();

    // 情况 A：全白丢线
    if (sensor_val == 0) {
        is_lost_line = 1;
        return;
    }

    // 情况 B：突遇左直角弯 (参考同仁Turning检测)
    // 只要左侧三路压满，判定为直角
    if ((sensor_val & 0b00000111) == 0b00000111) {
        osThreadSuspend(MotionTaskHandle); // 1. 物理点穴

        // 2. 入弯主动反接制动 (解决 800px/s 惯性)
        R3X_Set_Speed(MOTOR_FL, -500); R3X_Set_Speed(MOTOR_RL, -500);
        R3X_Set_Speed(MOTOR_FR, -500); R3X_Set_Speed(MOTOR_RR, -500);
        osDelay(40);

        // 3. 盲转逃逸 (快速甩出直角顶点)
        R3X_Set_Speed(MOTOR_FL, 700); R3X_Set_Speed(MOTOR_RL, 700);
        R3X_Set_Speed(MOTOR_FR, -400); R3X_Set_Speed(MOTOR_RR, -400);
        osDelay(120);

        // 4. 中心捕获 (死等 4,5 号对正)
        while (!(Fetch_Sensor_Raw() & 0b00011000)) {
            osDelay(5);
        }

        // 5. 退出锁死并对齐航向
        extern volatile float current_yaw, target_yaw;
        target_yaw = current_yaw;
        is_lost_line = 0;
        osThreadResume(MotionTaskHandle);
        return;
    }

    // 情况 C：突遇右直角弯 (极性对称)
    if ((sensor_val & 0b11100000) == 0b11100000) {
        osThreadSuspend(MotionTaskHandle);
        R3X_Set_Speed(MOTOR_FL, -500); R3X_Set_Speed(MOTOR_RL, -500);
        R3X_Set_Speed(MOTOR_FR, -500); R3X_Set_Speed(MOTOR_RR, -500);
        osDelay(40);
        R3X_Set_Speed(MOTOR_FL, -400); R3X_Set_Speed(MOTOR_RL, -400);
        R3X_Set_Speed(MOTOR_FR, 700); R3X_Set_Speed(MOTOR_RR, 700);
        osDelay(120);
        while (!(Fetch_Sensor_Raw() & 0b00011000)) osDelay(5);
        extern volatile float current_yaw, target_yaw;
        target_yaw = current_yaw;
        is_lost_line = 0;
        osThreadResume(MotionTaskHandle);
        return;
    }

    // 情况 D：常规质心巡航
    float error = Calculate_Line_Error(sensor_val);
    last_valid_error = error;

    extern volatile float current_yaw, target_yaw;
    // 刚刚扫回线，执行电子刹车 80ms
    if (is_lost_line) {
        osThreadSuspend(MotionTaskHandle);
        R3X_Set_Speed(MOTOR_FL, 0); R3X_Set_Speed(MOTOR_RL, 0);
        R3X_Set_Speed(MOTOR_FR, 0); R3X_Set_Speed(MOTOR_RR, 0);
        osDelay(80);
        target_yaw = current_yaw;
        is_lost_line = 0;
        osThreadResume(MotionTaskHandle);
    }

    // 串级叠加
    target_yaw += (error * Kp_outer);

    // 物理墙限幅：禁止目标航向与实际航向偏差超过 35 度，防止漂移报复
    if (target_yaw - current_yaw > 35.0f) target_yaw = current_yaw + 35.0f;
    if (target_yaw - current_yaw < -35.0f) target_yaw = current_yaw - 35.0f;

    // 归一化处理... (略)
}