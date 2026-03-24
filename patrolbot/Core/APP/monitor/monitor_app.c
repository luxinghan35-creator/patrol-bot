//
// Created by luxin on 2026/3/24.
//
/**
 * @file    monitor_app.c
 * @brief   R3X 监控与预警业务逻辑实现
 * @details
 * - 核心职责：充当机器人的“视觉中枢”，实时监测变电站刀闸等设备的异常发热情况。
 * - 熔断机制：通过修改全局原子锁 (system_alarm_flag) 实现对底盘运动的最高级剥夺。
 *
 * @author  luxin
 * @date    2026/3/24
 */
#include "monitor_app.h"
#include "oled.h"
#include "gy906.h"
#include "system_def.h"
#include <stdio.h>

/**
 * @brief 全局跨任务熔断锁 (SIL 级别)
 * @note  0: 系统正常运行 | 1: 触发超温熔断，底盘强制锁死。必须使用 volatile 修饰以防止被编译器优化。
 */
volatile uint8_t system_alarm_flag = 0;

/**
 * @brief 监控与预警系统初始化
 * @details 此处为初始化OLED显示屏，当前配置为页寻址模式
 */
void Monitor_App_Init(void) {
    OLED_Init();
}

/**
 * @brief 监控业务层应用层初始化
* @details 包含三大工序：
 * 1. 【感知】：通过 I2C 总线读取 GY-906 目标温度。
 * 2. 【决策】：执行温度阈值判定，过滤通信错误码，控制 system_alarm_flag 状态翻转。
 * 3. 【交互】：根据系统状态锁，渲染并推送对应的 OLED 帧显存数据。
 * @retval  None
 */

void Monitor_App_TaskLoop(void) {
    float obj_temp = R3X_GY906_Read_ObjTemp();

    // 过滤掉 I2C 通信异常产生的 -999 错误码，只在真实高温区间触发物理锁死，温度为60-500℃
    if (obj_temp > 60.0f && obj_temp < 500.0f) {
        system_alarm_flag = 1;
    } else if (obj_temp < 50.0f) {
        system_alarm_flag = 0;//回落到安全数值一下后小车继续前进
    }

    OLED_NewFrame();
    char buf[32];
    /**
     * @brief   视觉层系统状态分发渲染逻辑
     * @details
     * - 警报状态 (Flag=1)：使用反色字体高亮闪烁 "!! ALARM !!" 与 "MOTORS LOCKED"。
     * - 正常状态 (Flag=0)：白底黑字刷新当前温度、偏航角及系统帧率。
     */
    if (system_alarm_flag == 1) {
        OLED_PrintASCIIString(0, 0, "!! ALARM !!", &afont16x8, OLED_COLOR_REVERSED);
        sprintf(buf, "TEMP: %d C", (int)obj_temp);
        OLED_PrintASCIIString(0, 20, buf, &afont16x8, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(0, 40, "MOTORS LOCKED", &afont16x8, OLED_COLOR_NORMAL);
    } else {
        sprintf(buf, "Temp: %d C", (int)obj_temp);
        OLED_PrintASCIIString(0, 0, buf, &afont16x8, OLED_COLOR_NORMAL);
        sprintf(buf, "Yaw: %d", (int)current_yaw);
        OLED_PrintASCIIString(0, 20, buf, &afont16x8, OLED_COLOR_NORMAL);
        sprintf(buf, "SYS OK | %lu", rx_frame_cnt);
        OLED_PrintASCIIString(0, 40, buf, &afont16x8, OLED_COLOR_NORMAL);
    }
    OLED_ShowFrame();
}