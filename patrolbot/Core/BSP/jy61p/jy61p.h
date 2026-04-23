//
// Created by luxin on 2026/4/23.
//
#ifndef PATROLBOT_JY61P_H
#define PATROLBOT_JY61P_H

#include "main.h"

// ==========================================================
// JY61P 姿态解算驱动 (完全解耦串口外设)
// ==========================================================

// 初始化函数：传入你希望绑定的硬件串口句柄 (例如 &huart3)
void JY61P_Init(UART_HandleTypeDef *huart);

// 必须放置在 main.c 的 HAL_UART_RxCpltCallback 中调用
void JY61P_RxCpltCallback(UART_HandleTypeDef *huart);

// 必须放置在 main.c 的 HAL_UART_ErrorCallback 中调用 (防止 ORE 死锁)
void JY61P_ErrorCallback(UART_HandleTypeDef *huart);

#endif // PATROLBOT_JY61P_H