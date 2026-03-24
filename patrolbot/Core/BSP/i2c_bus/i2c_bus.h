//
// Created by luxin on 2026/3/23.
//

#ifndef PATROLBOT_I2C_H
#define PATROLBOT_I2C_H
#include "main.h"
// 返回 0 表示成功，返回 1 表示超时报错
uint8_t R3X_I2C_Read_Reg(uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t len);
uint8_t R3X_I2C_Write_Reg(uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t len);
void R3X_I2C_Reset(void);
#endif //PATROLBOT_I2C_H