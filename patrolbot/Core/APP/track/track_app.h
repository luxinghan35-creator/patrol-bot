//
// Created by luxin on 2026/3/29.
//

#ifndef PATROLBOT_TRACK_APP_H
#define PATROLBOT_TRACK_APP_H

#include "main.h"

// 状态定义
typedef enum {
    TRACK_STATE_IDLE = 0,
    TRACK_STATE_CRUISE,    // 常规加权巡航
    TRACK_STATE_TURN_LEFT, // 强控左转
    TRACK_STATE_TURN_RIGHT,// 强控右转
    TRACK_STATE_LOST       // 丢线处理
} TrackState_t;

void Track_App_Init(void);
void Track_App_TaskLoop(void);

#endif