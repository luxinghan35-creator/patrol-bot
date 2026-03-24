//
// Created by luxin on 2026/3/23.
//

#ifndef PATROLBOT_GY906_H
#define PATROLBOT_GY906_H
#include "main.h"

// 返回真实的摄氏度温度。如果总线出错，返回 -999.0f 作为异常报警标志
float R3X_GY906_Read_ObjTemp(void);
#endif //PATROLBOT_GY906_H