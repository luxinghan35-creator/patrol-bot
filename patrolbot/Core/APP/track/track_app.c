//
// Created by luxin on 2026/3/29.
//
#include "track_app.h"
#include "gpio.h"
#include "chassis_app.h"
#include "motor_driver.h"

/** @brief 外环航向牵引速率 (注意：因为改成了累加制，这个值必须调小，否则转弯会极快) */
static float Kp_outer = 0.5f;

/** @brief 记忆最后一次有效偏差 */
static float last_valid_error = 0.0f;

/** @brief 状态机标志位：去掉了 static！允许 chassis_app.c 跨文件调用拦截 */
uint8_t is_lost_line = 0;

void Track_App_Init(void) {
    last_valid_error = 0.0f;
    is_lost_line = 0;
}

static uint8_t Read_Sensor_Array(void) {
    // 请务必替换为你真实的 GPIO 读取代码，以下为占位
    return 0;
}

void Track_App_TaskLoop(void) {
    uint8_t sensor_val = Read_Sensor_Array();
    float current_error = 0.0f;
    uint8_t valid_read = 1;

    // 1. 特征字典解析
    switch (sensor_val) {
        case 0b00011000: current_error = 0.0f;  break;
        case 0b00110000: current_error = -1.0f; break;
        case 0b00001100: current_error = 1.0f;  break;
        case 0b01100000: current_error = -2.0f; break;
        case 0b00000110: current_error = 2.0f;  break;
        case 0b11000000: current_error = -3.5f; break;
        case 0b10000000: current_error = -4.0f; break;
        case 0b00000011: current_error = 3.5f;  break;
        case 0b00000001: current_error = 4.0f;  break;
        case 0b00000000:
            valid_read = 0;
            break;
        default:
            current_error = last_valid_error;
            break;
    }

    if (valid_read) {
        // --- 正常巡线状态 ---
        is_lost_line = 0;
        last_valid_error = current_error;

        extern volatile float target_yaw;

        // 【核心修复】：以积分形式牵引目标航向，绝不破坏内环的绝对抗扰动能力！
        target_yaw += (current_error * Kp_outer);

        // 航向角越界保护 (防止长时间跑圈导致 float 失去精度)
        if (target_yaw > 180.0f) {
            target_yaw -= 360.0f;
        } else if (target_yaw < -180.0f) {
            target_yaw += 360.0f;
        }

    } else {
        // --- 彻底丢线，触发记忆强控 ---
        is_lost_line = 1;
        if (last_valid_error <= -3.0f) {
            R3X_Drive_Tank(-200, 200);
        } else if (last_valid_error >= 3.0f) {
            R3X_Drive_Tank(200, -200);
        }
    }
}