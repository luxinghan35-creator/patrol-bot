//
// Created by luxin on 2026/3/29.
//

#ifndef PATROLBOT_TRACK_APP_H
#define PATROLBOT_TRACK_APP_H
#include "main.h"

/**
 * @brief 循迹外环初始化
 */
void Track_App_Init(void);

/**
 * @brief 循迹外环主业务逻辑 (必须在内环 PID 解算前调用)
 */
void Track_App_TaskLoop(void);

#endif //PATROLBOT_TRACK_APP_H