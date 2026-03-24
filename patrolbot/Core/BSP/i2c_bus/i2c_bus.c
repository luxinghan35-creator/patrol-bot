//
// Created by luxin on 2026/3/23.
//
// 返回 0 表示成功，返回 1 表示超时报错
#include "i2c_bus.h"
#include "i2c.h" // 引用你的 hi2c1

#define I2C_TIMEOUT 10 // 极限超时时间 10ms，绝不允许死等占用 CPU

// 工业级自愈复位函数
void R3X_I2C_Reset(void) {
    // 暴力关断 I2C 外设并重新初始化，切断死锁状态
    HAL_I2C_DeInit(&hi2c1);
    MX_I2C1_Init();
}

uint8_t R3X_I2C_Read_Reg(uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t len) {
    // 调用 HAL 库带 Timeout 的接口
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);

    if (status != HAL_OK) {
        R3X_I2C_Reset(); // 发现掉线或卡死，立刻触发复位机制
        return 1;        // 向上层报错
    }
    return 0; // 读取完美成功
}

uint8_t R3X_I2C_Write_Reg(uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t len) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);

    if (status != HAL_OK) {
        R3X_I2C_Reset();
        return 1;
    }
    return 0;
}