//
// Created by luxin on 2026/3/23.
//
// 返回 0 表示成功，返回 1 表示超时报错
#include "i2c_bus.h"
#include "i2c.h" // 引用你的 hi2c1

#define I2C_TIMEOUT 50 // 极限超时时间 10ms，绝不允许死等占用 CPU

/**
 * @brief   自愈复位机制实现
 * @retval  None
 */
void R3X_I2C_Reset(void) {
    // 暴力关断 I2C 外设并重新初始化，切断死锁状态
    HAL_I2C_DeInit(&hi2c1);
    MX_I2C1_Init();
}

/**
 * @brief   带防死锁保护的 I2C 寄存器读取
 */
uint8_t R3X_I2C_Read_Reg(uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t len) {
    // 调用 HAL 库带 Timeout 的接口
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);

    if (status != HAL_OK) {
        R3X_I2C_Reset(); // 发现掉线或卡死，立刻触发复位机制
        return 1;        // 向上层报错
    }
    return 0; // 读取完美成功
}

/**
 * @brief   带防死锁保护的 I2C 寄存器写入
 */
uint8_t R3X_I2C_Write_Reg(uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t len) {
    // 1. 尝试以指定超时容忍度进行轮询写入
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
    // 2. 异常捕获与暴力恢复机制联动
    if (status != HAL_OK) {
        R3X_I2C_Reset();
        return 1;// 向上层业务 (如 GY906/OLED) 抛出错误代码 1，防止其误用随机脏数据
    }
    return 0;
}