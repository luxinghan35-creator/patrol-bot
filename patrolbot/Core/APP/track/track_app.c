//
// Created by luxin on 2026/3/29.
//
/**
 * @file    track_app.c
 * @brief   R3X 外环循迹状态机实现 (纯正 I2C 总线版本)
 */

#include "track_app.h"
#include "i2c_bus.h"
#include "chassis_app.h"
#include "cmsis_os.h"
#include "motor_driver.h"

// 硬件地址
#define GRAY_SENSOR_ADDR 0x98

static uint8_t Read_Sensor_Array(void) {
    uint8_t raw_data = 0;

    // 1. 从寄存器 0xDD (根据你的芯片手册而定) 读取 1 字节状态
    if (R3X_I2C_Read_Reg(GRAY_SENSOR_ADDR, 0xDD, &raw_data, 1) == 0) {

        // 2. 读取成功：按位取反，确保 1=黑线，0=白底，并屏蔽多余的高位
        return (uint8_t)(~raw_data & 0x00FF);
    }

    // 3. 读取失败(断线/死机)：返回安全脏数据，防止上层状态机失控疯跑
    return 0x55;
}

static float Kp_outer = -0.5f;
float last_valid_error = 0.0f;
uint8_t is_lost_line = 0;

extern osThreadId_t MotionTaskHandle; // 引入内环任务句柄，用于夺权

void Track_App_Init(void) {
    last_valid_error = 0.0f;
    is_lost_line = 0;
    osDelay(500); // 传感器上电稳定时间
}

void Track_App_TaskLoop(void) {
    uint8_t sensor_val = Read_Sensor_Array();
    float current_error = 0.0f;
    uint8_t valid_read = 1;

    // 特征字典解析
    if (sensor_val == 0b00000000) {
        valid_read = 0; // 全白丢线
    }
    else if ((sensor_val & 0b01111110) == 0b01111110) {
        current_error = 0.0f; // 十字路口硬冲
    }
    // 2. 【强控拦截】左直角弯 / 锐角弯
    else if ((sensor_val & 0b11100000) == 0b11100000 || (sensor_val & 0b11000000) == 0b11000000) {
        current_error = 0.0f; // 强控期切断外环累加
        osThreadSuspend(MotionTaskHandle);

        // 【入弯主动反接制动】绞杀前冲直线动能
        R3X_Set_Speed(MOTOR_FL, -400); R3X_Set_Speed(MOTOR_RL, -400);
        R3X_Set_Speed(MOTOR_FR, -400); R3X_Set_Speed(MOTOR_RR, -400);
        osDelay(30);
        R3X_Set_Speed(MOTOR_FL, 0); R3X_Set_Speed(MOTOR_RL, 0);
        R3X_Set_Speed(MOTOR_FR, 0); R3X_Set_Speed(MOTOR_RR, 0);
        osDelay(20);

        // 【第一段：盲转逃逸】强行离开直角顶点
        R3X_Set_Speed(MOTOR_FL, -400); R3X_Set_Speed(MOTOR_RL, -400);
        R3X_Set_Speed(MOTOR_FR, 700);  R3X_Set_Speed(MOTOR_RR, 700);
        osDelay(100);

        // 【第二段：中心死区捕获】死等 4、5 号探头归位
        sensor_val = Read_Sensor_Array();
        while ((sensor_val & 0b00011000) == 0) {
            R3X_Set_Speed(MOTOR_FL, -400); R3X_Set_Speed(MOTOR_RL, -400);
            R3X_Set_Speed(MOTOR_FR, 700);  R3X_Set_Speed(MOTOR_RR, 700);
            osDelay(10);
            sensor_val = Read_Sensor_Array();
        }

        // 【出弯抱死防甩】按平旋转动能
        R3X_Set_Speed(MOTOR_FL, 0); R3X_Set_Speed(MOTOR_RL, 0);
        R3X_Set_Speed(MOTOR_FR, 0); R3X_Set_Speed(MOTOR_RR, 0);
        osDelay(40);

        // 【物理航向重置】防止旧账导致 PID 报复性疯转
        extern volatile float target_yaw;
        extern volatile float current_yaw;
        target_yaw = current_yaw;

        osThreadResume(MotionTaskHandle);
    }
    // 3. 【强控拦截】右直角弯 / 锐角弯
    else if ((sensor_val & 0b00000111) == 0b00000111 || (sensor_val & 0b00000011) == 0b00000011) {
        current_error = 0.0f;
        osThreadSuspend(MotionTaskHandle);

        // 【入弯主动反接制动】
        R3X_Set_Speed(MOTOR_FL, -400); R3X_Set_Speed(MOTOR_RL, -400);
        R3X_Set_Speed(MOTOR_FR, -400); R3X_Set_Speed(MOTOR_RR, -400);
        osDelay(30);
        R3X_Set_Speed(MOTOR_FL, 0); R3X_Set_Speed(MOTOR_RL, 0);
        R3X_Set_Speed(MOTOR_FR, 0); R3X_Set_Speed(MOTOR_RR, 0);
        osDelay(20);

        // 【第一段：盲转逃逸】
        R3X_Set_Speed(MOTOR_FL, 700);  R3X_Set_Speed(MOTOR_RL, 700);
        R3X_Set_Speed(MOTOR_FR, -400); R3X_Set_Speed(MOTOR_RR, -400);
        osDelay(100);

        // 【第二段：中心死区捕获】
        sensor_val = Read_Sensor_Array();
        while ((sensor_val & 0b00011000) == 0) {
            R3X_Set_Speed(MOTOR_FL, 700);  R3X_Set_Speed(MOTOR_RL, 700);
            R3X_Set_Speed(MOTOR_FR, -400); R3X_Set_Speed(MOTOR_RR, -400);
            osDelay(10);
            sensor_val = Read_Sensor_Array();
        }

        // 【出弯抱死防甩】
        R3X_Set_Speed(MOTOR_FL, 0); R3X_Set_Speed(MOTOR_RL, 0);
        R3X_Set_Speed(MOTOR_FR, 0); R3X_Set_Speed(MOTOR_RR, 0);
        osDelay(40);

        extern volatile float target_yaw;
        extern volatile float current_yaw;
        target_yaw = current_yaw;

        osThreadResume(MotionTaskHandle);
    }
    // 4. 【动态加权质心纠错巡航】彻底消灭视觉死角
    else {
        float weights[8] = {4.0f, 3.0f, 2.0f, 1.0f, -1.0f, -2.0f, -3.0f, -4.0f};
        float sum = 0.0f;
        int count = 0;

        for (int i = 0; i < 8; i++) {
            if (sensor_val & (1 << i)) {
                sum += weights[i];
                count++;
            }
        }

        if (count > 0) {
            current_error = sum / count;
        } else {
            current_error = last_valid_error;
        }
    }

    // ==========================================
    // 姿态分发与 IMU 串级同步
    // ==========================================
    if (valid_read) {
        extern volatile float target_yaw;
        extern volatile float current_yaw;

        if (is_lost_line == 1) {
            osThreadSuspend(MotionTaskHandle);

            // 刚扫到线时的高速自旋动能抱死
            R3X_Set_Speed(MOTOR_FL, 0); R3X_Set_Speed(MOTOR_RL, 0);
            R3X_Set_Speed(MOTOR_FR, 0); R3X_Set_Speed(MOTOR_RR, 0);
            osDelay(80);

            target_yaw = current_yaw;
            is_lost_line = 0;
            osThreadResume(MotionTaskHandle);
        }

        is_lost_line = 0;
        last_valid_error = current_error;

        // 【激活串级】外环误差转化为持续偏航角累加
        float yaw_step = current_error * Kp_outer;
        target_yaw += yaw_step;

        // 【抗积分风转锁】禁止目标角过度超前物理角度
        if (target_yaw - current_yaw > 30.0f) {
            target_yaw = current_yaw + 30.0f;
        } else if (target_yaw - current_yaw < -30.0f) {
            target_yaw = current_yaw - 30.0f;
        }

        // 环绕归一化
        if (target_yaw > 180.0f) target_yaw -= 360.0f;
        else if (target_yaw < -180.0f) target_yaw += 360.0f;

    } else {
        is_lost_line = 1;
    }
}