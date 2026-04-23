#include "jy61p.h"
#include "system_def.h" // 必须包含，为了直接更新外部的 current_yaw

// --- 物理层私有状态 (加上 static，绝对禁止外部直接访问) ---
static UART_HandleTypeDef *jy_huart = NULL; // 记录绑定的底层串口
static uint8_t rx_byte = 0;                 // 接收邮筒
static uint8_t rx_buffer[11];               // 数据帧重组车间
static uint8_t rx_index = 0;                // 状态机指针
volatile uint32_t rx_frame_cnt = 0;           // 健康心跳包计数

void JY61P_Init(UART_HandleTypeDef *huart) {
    if (huart == NULL) return;

    jy_huart = huart;
    rx_index = 0;
    rx_frame_cnt = 0;

    // 【物理防御升级】：暴力清除开机瞬间积攒的乱码和溢出错误
    // 否则寄存器处于死锁状态，第一发子弹绝对上不了膛！
    __HAL_UART_CLEAR_OREFLAG(huart);

    // 重新上膛
    HAL_UART_Receive_IT(jy_huart, &rx_byte, 1);
}

void JY61P_RxCpltCallback(UART_HandleTypeDef *huart) {
    // 【核心防御】：如果触发中断的串口，不是我绑定的那个串口，立刻滚蛋！
    if (jy_huart == NULL || huart->Instance != jy_huart->Instance) return;

    // --- 纯正的工业级状态机 ---
    if (rx_index == 0) {
        if (rx_byte == 0x55) { rx_buffer[0] = rx_byte; rx_index = 1; }
    } else if (rx_index == 1) {
        if (rx_byte == 0x53) { rx_buffer[1] = rx_byte; rx_index = 2; }
        else { rx_index = 0; } // 绞杀加速度等无关帧
    } else {
        rx_buffer[rx_index++] = rx_byte;
        if (rx_index >= 11) {
            int16_t yaw_raw = (rx_buffer[7] << 8) | rx_buffer[6];
            current_yaw = ((float)yaw_raw / 32768.0f) * 180.0f; // 穿透更新系统变量
            rx_frame_cnt++;
            rx_index = 0;
        }
    }
    // 重新上膛，准备接下一发子弹
    HAL_UART_Receive_IT(jy_huart, &rx_byte, 1);
}

void JY61P_ErrorCallback(UART_HandleTypeDef *huart) {
    if (jy_huart == NULL || huart->Instance != jy_huart->Instance) return;

    // 暴力清除 Overrun Error (溢出错误)，防单片机死锁
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(jy_huart, &rx_byte, 1);
}