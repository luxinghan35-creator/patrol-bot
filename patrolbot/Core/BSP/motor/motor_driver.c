#include "motor_driver.h"
#include "main.h"
#include "tim.h"
//宏定义简化GPIO操作
#define SET_PIN(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define CLR_PIN(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
/**
 *  @brief 电机状态枚举
 */
typedef enum {MOTOR_STOP, MOTOR_FWD, MOTOR_REV} MotorState_t;
//初始化
void R3X_Init(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

/**
 * @brief （内部函数）设置电机方向引脚状态
 * @param id 电机编号
 * @param state 电机状态 (FWD/REV/STOP)
 */
static void Set_Direction(MotorID_t id, MotorState_t state) {
    switch (id) {
        //前桥左电机PB12,PB13
        case MOTOR_FL:
            if (state == MOTOR_FWD) {
                SET_PIN(FL_DIR1_GPIO_Port, FL_DIR1_Pin);CLR_PIN(FL_DIR2_GPIO_Port, FL_DIR2_Pin);
            } else if (state == MOTOR_REV) {
                CLR_PIN(FL_DIR1_GPIO_Port, FL_DIR1_Pin);SET_PIN(FL_DIR2_GPIO_Port, FL_DIR2_Pin);
            } else {
                CLR_PIN(FL_DIR1_GPIO_Port, FL_DIR1_Pin);
                CLR_PIN(FL_DIR2_GPIO_Port, FL_DIR2_Pin);
                // 停止时不改变方向引脚状态
            }
            break;
        //前桥右电机PB14,PB15
        case MOTOR_FR:
            if (state == MOTOR_FWD) {
                SET_PIN(FR_DIR1_GPIO_Port, FR_DIR1_Pin);CLR_PIN(FR_DIR2_GPIO_Port, FR_DIR2_Pin);
            } else if (state == MOTOR_REV) {
                CLR_PIN(FR_DIR1_GPIO_Port, FR_DIR1_Pin);SET_PIN(FR_DIR2_GPIO_Port, FR_DIR2_Pin);
            } else {
                CLR_PIN(FR_DIR2_GPIO_Port, FR_DIR2_Pin);CLR_PIN(FR_DIR1_GPIO_Port, FR_DIR1_Pin);
                // 停止时不改变方向引脚状态
            }
            break;
        //后桥左电机PB3,PB4(需禁用JTAG释放)
        case MOTOR_RL:
            if (state == MOTOR_FWD) {
                SET_PIN(RL_DIR1_GPIO_Port, RL_DIR1_Pin);CLR_PIN(RL_DIR2_GPIO_Port, RL_DIR2_Pin);
            } else if (state == MOTOR_REV) {
                CLR_PIN(RL_DIR1_GPIO_Port, RL_DIR1_Pin);SET_PIN(RL_DIR2_GPIO_Port, RL_DIR2_Pin);
            } else {
                CLR_PIN(RL_DIR1_GPIO_Port, RL_DIR1_Pin);CLR_PIN(RL_DIR2_GPIO_Port, RL_DIR2_Pin);
                // 停止时不改变方向引脚状态
            }
            break;
        //后桥右电机PB5,PB9
        case MOTOR_RR:
            if (state == MOTOR_FWD) {
                SET_PIN(RR_DIR1_GPIO_Port, RR_DIR1_Pin);CLR_PIN(RR_DIR2_GPIO_Port, RR_DIR2_Pin);
            } else if (state == MOTOR_REV) {
                CLR_PIN(RR_DIR1_GPIO_Port, RR_DIR1_Pin);SET_PIN(RR_DIR2_GPIO_Port, RR_DIR2_Pin);
            } else {
                CLR_PIN(RR_DIR1_GPIO_Port, RR_DIR1_Pin);CLR_PIN(RR_DIR2_GPIO_Port, RR_DIR2_Pin);
                // 停止时不改变方向引脚状态
            }
            break;
        default:
            break;
    }
}
//设置电机速度
void R3X_Set_Speed(MotorID_t id, int16_t speed) {
    MotorState_t state;
    if (speed > 0) {
        state = MOTOR_FWD;
    } else if (speed < 0) {
        state = MOTOR_REV;
        speed = -speed; //取绝对值
    } else {
        state = MOTOR_STOP;
    }
    if (speed > 1000) speed = 1000; //限幅
    Set_Direction(id, state);//设置方向
    //设置PWM占空比
    switch (id) {
        case MOTOR_FL: // 左前
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
            break;
        case MOTOR_RL: // 左后 (原来写成了FR！)
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);
            break;
        case MOTOR_FR: // 右前 (原来写成了RL！)
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed);
            break;
        case MOTOR_RR: // 右后
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
            break;
        default:
            break;
    }

}

void R3X_Drive_Tank(int16_t speed_L, int16_t speed_R) {
    R3X_Set_Speed(MOTOR_FL, speed_L);
    R3X_Set_Speed(MOTOR_RL, speed_L);
    R3X_Set_Speed(MOTOR_FR, speed_R);
    R3X_Set_Speed(MOTOR_RR, speed_R);
}

//
// Created by luxin on 2026/1/31.
