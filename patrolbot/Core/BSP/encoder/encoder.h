//
// Created by luxin on 2026/3/21.
//

#ifndef PATROLBOT_ENCODER_H
#define PATROLBOT_ENCODER_H
#include "main.h"
void R3X_Encoder_Init(void);
int16_t R3X_Get_Left_Speed(void);//左轮轮速
int16_t R3X_Get_Right_Speed(void);//右轮轮速
#endif //PATROLBOT_ENCODER_H