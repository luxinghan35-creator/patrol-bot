//
// Created by luxin on 2026/3/23.
//
// 返回真实的摄氏度温度。如果总线出错，返回 -999.0f 作为异常报警标志
#include "gy906.h"
#include "i2c_bus.h" // 引用你的工业级 I2C 底层

#define GY906_ADDR 0xB4     // 0x5A 左移一位，STM32 HAL库要求
#define REG_OBJ_TEMP 0x07   // 目标物体温度的寄存器地址

float R3X_GY906_Read_ObjTemp(void) {
    uint8_t data[3]; // 接收 LSB, MSB, 和 PEC校验位

    // 调用我们写的带有超时自愈功能的底层接口
    if (R3X_I2C_Read_Reg(GY906_ADDR, REG_OBJ_TEMP, data, 3) == 0) {
        // 数据拼接
        uint16_t raw_temp = (data[1] << 8) | data[0];
        // GY-906 (MLX90614) 的官方温度换算公式
        float temp = (raw_temp * 0.02f) - 273.15f;
        return temp;
    }

    // 如果走到这里，说明 I2C 掉线或者卡死了（底层已经自动复位外设）
    return -999.0f;
}