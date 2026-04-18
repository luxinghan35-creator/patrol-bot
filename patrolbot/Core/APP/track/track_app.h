//
// Created by luxin on 2026/3/29.
//

#ifndef PATROLBOT_TRACK_APP_H
#define PATROLBOT_TRACK_APP_H

#include "stdint.h"

// ==========================================================
// 物理奇点（硬件中断）与特征掩码宏定义
// ==========================================================
#define SENSOR_LEFT_TURN_TRIGGER   0b11100000  // 左直角强控触发线 (1,2,3压线)
#define SENSOR_RIGHT_TURN_TRIGGER  0b00000111  // 右直角强控触发线 (6,7,8压线)
#define SENSOR_CENTER_LOCK         0b00011000  // 甩尾中心捕获线   (4,5压线)

#define I2C_ERROR_THRESHOLD        10          // 连续读取失败的熔断阈值 (防死机)

// ==========================================================
// 外部接口
// ==========================================================
void Track_App_Init(void);
void Track_App_TaskLoop(void);

// 供底盘调用的 Getter 接口：当前是否处于强制转弯接管状态？
// 返回 1 表示外环强控中，内环需避让；返回 0 表示正常巡航。
uint8_t Track_Is_Turning(void);

#endif // PATROLBOT_TRACK_APP_H