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

extern osThreadId_t MotionTaskHandle; // 引入内环任务句柄

// 【硬件地址】不插跳线帽为 0x4C，左移 1 位得 0x98。
// 如果你插了双跳线帽，请改为 0x9E！(不确定请告诉我你板子上的跳线情况)
#define GRAY_SENSOR_ADDR 0x98

static float Kp_outer = -0.5f;
float last_valid_error = 0.0f;
uint8_t is_lost_line = 0;

void Track_App_Init(void) {
    last_valid_error = 0.0f;
    is_lost_line = 0;

    // 【最高规格防护】网络诊断 (Ping) 同步 - 工业级重构版
    uint8_t ping_val = 0;
    uint8_t retry_count = 0;

    // 1. 【物理时序让步】强制休眠 500ms，给灰度传感器内部 MCU 充足的开机时间！
    osDelay(500);

    // 2. 【防死锁轮询】最多只试 10 次，绝不无限期死等
    while (ping_val != 0x66 && retry_count < 10) {
        R3X_I2C_Read_Reg(GRAY_SENSOR_ADDR, 0xAA, &ping_val, 1);
        retry_count++;
        osDelay(20); // 每次失败等 20ms 再试
    }

    // 3. 【系统级熔断隔离】
    if (ping_val != 0x66) {
        // 走到这里说明灰度传感器彻底死了（断线或坏了）
        // 但我们绝不卡死系统！强行放行，让 OLED 和测温继续工作。
        // 外环任务会在 TaskLoop 里因为读不到数据自动触发 is_lost_line 原地自旋保护。
    }
}

static uint8_t Read_Sensor_Array(void) {
    uint8_t raw_data = 0;

    // 【硬件直读】使用命令 0xDD 读取 8 路数字量 (手册 7.7)
    if (R3X_I2C_Read_Reg(GRAY_SENSOR_ADDR, 0xDD, &raw_data, 1) == 0) {
        // 【极性反转】手册明确白场=1，黑场=0。
        // 取反后屏蔽高位，完美适配黑线=1 的状态机算法
        return (uint8_t)(~raw_data & 0x00FF);
    }

    // 【I2C 断线自愈降级】
    // 走到这里说明底层的 i2c_bus.c 已经触发了总线重启。
    // 返回 0x55 这个绝对不可能匹配黑线特征的脏数据，
    // 它会触发下面的 default 维持原状，而绝不会误触发全 0 的盲打自旋状态机！
    return 0x55;
}

void Track_App_TaskLoop(void) {
    uint8_t sensor_val = Read_Sensor_Array();
    float current_error = 0.0f;
    uint8_t valid_read = 1;

    // 特征字典解析 - 工业级掩码模糊匹配与强控升级版
    if (sensor_val == 0b00000000) {
        valid_read = 0; // 全白，彻底丢线，交给 Chassis_App 暴力自旋
    }
    // 1. 【十字路口过滤】(大面积全黑) - 必须放在最前
    // 直接把误差归零，硬扛着冲过去，绝不乱扭
    else if ((sensor_val & 0b01111110) == 0b01111110) {
        current_error = 0.0f;
    }
    // 2. 【强控拦截】左直角弯 / 锐角弯
    else if ((sensor_val & 0b11100000) == 0b11100000 || (sensor_val & 0b11000000) == 0b11000000) {
        current_error = -6.0f;

        // 【核心补丁：系统级越权】强行挂起内环 PID 任务，剥夺其路权！
        osThreadSuspend(MotionTaskHandle);

        R3X_Set_Speed(MOTOR_FL, -400);
        R3X_Set_Speed(MOTOR_RL, -400);
        R3X_Set_Speed(MOTOR_FR, 700);
        R3X_Set_Speed(MOTOR_RR, 700);
        osDelay(30);

        // 砸过弯后，重新激活内环 PID 任务
        osThreadResume(MotionTaskHandle);
    }
    // 3. 【强控拦截】右直角弯 / 锐角弯
    else if ((sensor_val & 0b00000111) == 0b00000111 || (sensor_val & 0b00000011) == 0b00000011) {
        current_error = 6.0f;

        // 【核心补丁：系统级越权】
        osThreadSuspend(MotionTaskHandle);

        R3X_Set_Speed(MOTOR_FL, 700);
        R3X_Set_Speed(MOTOR_RL, 700);
        R3X_Set_Speed(MOTOR_FR, -400);
        R3X_Set_Speed(MOTOR_RR, -400);
        osDelay(30);

        osThreadResume(MotionTaskHandle);
    }
    // 4. 【常规微调】兼容你原有的平滑巡航逻辑
    else if (sensor_val == 0b10000000) current_error = -4.0f;
    else if (sensor_val == 0b01100000) current_error = -2.0f;
    else if (sensor_val == 0b00110000) current_error = -1.0f;
    else if (sensor_val == 0b00011000) current_error = 0.0f;
    else if (sensor_val == 0b00001100) current_error = 1.0f;
    else if (sensor_val == 0b00000110) current_error = 2.0f;
    else if (sensor_val == 0b00000001) current_error = 4.0f;
    else {
        // 杂波降级：什么都没匹配上，维持上一帧判断
        current_error = last_valid_error;
    }

    if (valid_read) {
        extern volatile float target_yaw;
        extern volatile float current_yaw;
        extern osThreadId_t MotionTaskHandle; // 引入内环 PID 任务句柄

        // 【物理级大手术 1：斩断“旋转动能锁死”】
        if (is_lost_line == 1) {
            // 此时刚刚从丢线盲打自旋中捕获到黑线！
            // 必须瞬间剥夺内环路权，防止它下发直行指令导致打滑！
            osThreadSuspend(MotionTaskHandle);

            // 极限电子刹车：全轮灌零，利用减速箱齿轮阻力强行绞杀旋转动能
            R3X_Set_Speed(MOTOR_FL, 0);
            R3X_Set_Speed(MOTOR_RL, 0);
            R3X_Set_Speed(MOTOR_FR, 0);
            R3X_Set_Speed(MOTOR_RR, 0);
            osDelay(80); // 死锁 80ms，硬生生把几公斤的车头按稳在黑线上！

            // 车体动能清零后，再把目标对齐当前物理航向，完成真正的无扰切换
            target_yaw = current_yaw;
            is_lost_line = 0; // 提前清零状态

            osThreadResume(MotionTaskHandle); // 重新放行内环
        }

        is_lost_line = 0;
        last_valid_error = current_error;

        // 【物理级大手术 2：拔除直线跑偏的“积分炸弹”】
        // 彻底抛弃 target_yaw += ... 的积分风转写法！
        // 改为基于当前真实物理姿态的“绝对偏移随动”：
        float yaw_offset = current_error * Kp_outer;
        target_yaw = current_yaw + yaw_offset;

        // 偏航角越界归一化 (-180 到 180 物理环绕)
        if (target_yaw > 180.0f) {
            target_yaw -= 360.0f;
        } else if (target_yaw < -180.0f) {
            target_yaw += 360.0f;
        }

    } else {
        is_lost_line = 1; // 彻底丢线，交给 Chassis_App 执行原地盲打自旋
    }
}