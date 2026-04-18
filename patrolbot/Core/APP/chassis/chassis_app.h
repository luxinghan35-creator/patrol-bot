//
// Created by luxin on 2026/3/24.
//

#ifndef PATROLBOT_CHASSIS_APP_H
#define PATROLBOT_CHASSIS_APP_H

#include "stdint.h"

// ==========================================================
// 外部接口：底盘任务生命周期管理
// ==========================================================

/**
 * @brief  底盘系统初始化 (包含编码器、PID 参数及目标航向对齐)
 * @note   必须在 RTOS 调度器启动前，或任务进入死循环前调用
 */
void Chassis_App_Init(void);

/**
 * @brief  底盘任务主循环
 * @note   包含超温熔断、路权避让、开环自旋及闭环姿态纠偏
 */
void Chassis_App_TaskLoop(void);

#endif // PATROLBOT_CHASSIS_APP_H