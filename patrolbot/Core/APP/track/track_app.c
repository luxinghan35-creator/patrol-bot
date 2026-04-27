//
// Created by luxin on 2026/3/29.
//
/**
 * @file    track_app.c
 * @brief   R3X 外环循迹状态机实现 (纯正 I2C 总线版本)
 */

#include "track_app.h"
#include "i2c.h"
#include "chassis_app.h"
#include "cmsis_os.h"
#include "motor_driver.h"
#include "system_def.h" // 用于获取 current_yaw, target_yaw, system_alarm_flag

// 硬件参数宏定义
#define GW_GRAY_ADDR 0x98
#define REG_DIGITAL  0xDD
#define REG_PING     0xAA

// 动力参数宏定义（甩尾时的固定 PWM）
#define PWM_TURN_FWD  400
#define PWM_TURN_REV -300
#define PWM_BRAKE     0

// ==========================================================
// 外部接口 (Getter 通行证)
// ==========================================================
void Track_App_Init(void);
void Track_App_TaskLoop(void);
uint8_t Track_Is_Turning(void);// 获取强控路权状态
uint8_t Track_Is_Lost(void);    // 获取是否全白丢线
float Track_Get_Error(void);    // 获取当前质心误差

// 局部私有变量（绝对不加 extern）
static float Kp_outer = -0.6f;
static float last_valid_error = 0.0f;
static uint8_t is_lost_line = 0;
static volatile uint8_t track_is_turning = 0;

uint8_t  Track_Is_Turning(void)
{
    return track_is_turning;
}

uint8_t Track_Is_Lost(void) {
    return is_lost_line;
}

float Track_Get_Error(void) {
    return last_valid_error;
}

// ==========================================================
// 底层防线：带单边调度锁、避让机制与视觉缓存的 I2C 读取
// ==========================================================
static uint8_t Fetch_Sensor_Raw(void) {
    static uint8_t last_sensor_data = SENSOR_CENTER_LOCK;
    static uint16_t i2c_error_count = 0;

    uint8_t data = 0;
    uint8_t retry = 0;
    HAL_StatusTypeDef res;

    // 单边防御策略：因为我们没动测温代码，所以冲突极有可能发生。
    do {
        vTaskSuspendAll(); // 关门：尝试独占总线
        res = HAL_I2C_Mem_Read(&hi2c1, GW_GRAY_ADDR, REG_DIGITAL, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
        xTaskResumeAll();  // 开门：读完立刻释放调度器

        if (res != HAL_OK) {
            // 如果撞车了，绝对不能死锁！主动延时 1ms，把 CPU 让给测温任务去完成读取
            osDelay(1);
            retry++;
        }
    } while (res != HAL_OK && retry < 3);

    // 状态更新与熔断机制
    if (res == HAL_OK) {
        last_sensor_data = (uint8_t)(~data);
        i2c_error_count = 0; // 成功则清零错误池
    } else {
        i2c_error_count++;
        // 如果连续 I2C_ERROR_THRESHOLD (比如10次) 彻底读不到，说明硬件断线
        // 强制返回 0x00，触发全白丢线，切断动力，防止带旧缓存疯跑！
        if (i2c_error_count > I2C_ERROR_THRESHOLD) {
            return 0x00;
        }
    }

    return last_sensor_data;
}

// ==========================================================
// 算法引擎：动态加权质心（消灭 256 种 if-else）
// ==========================================================
static float Calculate_Line_Error(uint8_t sensor_bits) {
    float weights[8] = {-4.5f, -3.0f, -1.5f, -0.5f, 0.5f, 1.5f, 3.0f, 4.5f};
    float sum = 0;
    int count = 0;

    for (int i = 0; i < 8; i++) {
        if (sensor_bits & (1 << i)) {
            sum += weights[i];
            count++;
        }
    }
    // 有线算平均质心，无线则靠视觉惯性滑行
    return (count > 0) ? (sum / (float)count) : last_valid_error;
}

// ==========================================================
// 系统初始化
// ==========================================================
void Track_App_Init(void) {
    uint8_t ping_val = 0;
    osDelay(500);
    // 强制握手，不握手不发车
    while (ping_val != 0x66) {
        HAL_I2C_Mem_Read(&hi2c1, GW_GRAY_ADDR, REG_PING, I2C_MEMADD_SIZE_8BIT, &ping_val, 1, 10);
        osDelay(50);
    }
}

// ==========================================================
// 决策层状态机主循环 (绝对不饿死看门狗)
// ==========================================================
void Track_App_TaskLoop(void) {
    // 1. 【全局安全熔断】：监控测温标志位 (假设 system_def.h 里有 extern volatile uint8_t system_alarm_flag)
    if (system_alarm_flag == 1) {
        track_is_turning = 0; // 交出路权，让内环去执行刹车
        osDelay(20);
        return;
    }

    uint8_t sensor_val = Fetch_Sensor_Raw();

    // 2. 【全白丢线保护】
    if (sensor_val == 0) {
        is_lost_line = 1;
        osDelay(10);
        return;
    }

    // 3. 【找回路线动能绞杀】
    if (is_lost_line == 1) {
        track_is_turning = 1; // 夺权制动
        R3X_Set_Speed(MOTOR_FL, PWM_BRAKE); R3X_Set_Speed(MOTOR_RL, PWM_BRAKE);
        R3X_Set_Speed(MOTOR_FR, PWM_BRAKE); R3X_Set_Speed(MOTOR_RR, PWM_BRAKE);
        osDelay(80);

        target_yaw = current_yaw; // 清空盲转时累积的旧账
        is_lost_line = 0;
        track_is_turning = 0;     // 归还路权
        osDelay(10);
        return;
    }

    // 4. 【突变特征拦截：左直角弯】
    if ((sensor_val & SENSOR_LEFT_TURN_TRIGGER) == SENSOR_LEFT_TURN_TRIGGER) {
        track_is_turning = 1;

        // a. 电子刹车吸收直线动能
        R3X_Set_Speed(MOTOR_FL, PWM_BRAKE); R3X_Set_Speed(MOTOR_RL, PWM_BRAKE);
        R3X_Set_Speed(MOTOR_FR, PWM_BRAKE); R3X_Set_Speed(MOTOR_RR, PWM_BRAKE);
        osDelay(60);

        // b. 左侧倒车，右侧前进，实现原地强力甩尾
        R3X_Set_Speed(MOTOR_FL, PWM_TURN_REV); R3X_Set_Speed(MOTOR_RL, PWM_TURN_REV);
        R3X_Set_Speed(MOTOR_FR, PWM_TURN_FWD); R3X_Set_Speed(MOTOR_RR, PWM_TURN_FWD);
        osDelay(150); // 盲转逃离顶点

        // c. 死锁等待出弯条件
        while (!(Fetch_Sensor_Raw() & SENSOR_CENTER_LOCK)) {
            if (system_alarm_flag == 1) { track_is_turning = 0; osDelay(10); return; } // 死锁时保持测温警觉

            // 维持固定甩尾动力
            R3X_Set_Speed(MOTOR_FL, PWM_TURN_REV); R3X_Set_Speed(MOTOR_RL, PWM_TURN_REV);
            R3X_Set_Speed(MOTOR_FR, PWM_TURN_FWD);  R3X_Set_Speed(MOTOR_RR, PWM_TURN_FWD);
            osDelay(10); // 绝对防饿死
        }

        // d. 退出修正
        target_yaw = current_yaw;
        track_is_turning = 0;
        osDelay(10);
        return;
    }

    // 5. 【突变特征拦截：右直角弯】
    if ((sensor_val & SENSOR_RIGHT_TURN_TRIGGER) == SENSOR_RIGHT_TURN_TRIGGER) {
        track_is_turning = 1;

        R3X_Set_Speed(MOTOR_FL, PWM_BRAKE); R3X_Set_Speed(MOTOR_RL, PWM_BRAKE);
        R3X_Set_Speed(MOTOR_FR, PWM_BRAKE); R3X_Set_Speed(MOTOR_RR, PWM_BRAKE);
        osDelay(60);

        R3X_Set_Speed(MOTOR_FL, PWM_TURN_FWD); R3X_Set_Speed(MOTOR_RL, PWM_TURN_FWD);
        R3X_Set_Speed(MOTOR_FR, PWM_TURN_REV); R3X_Set_Speed(MOTOR_RR, PWM_TURN_REV);
        osDelay(150);

        while (!(Fetch_Sensor_Raw() & SENSOR_CENTER_LOCK)) {
            if (system_alarm_flag == 1) { track_is_turning = 0; osDelay(10); return; }
            R3X_Set_Speed(MOTOR_FL, PWM_TURN_FWD); R3X_Set_Speed(MOTOR_RL, PWM_TURN_FWD);
            R3X_Set_Speed(MOTOR_FR, PWM_TURN_REV); R3X_Set_Speed(MOTOR_RR, PWM_TURN_REV);
            osDelay(10);
        }

        target_yaw = current_yaw;
        track_is_turning = 0;
        osDelay(10);
        return;
    }

    // 6. 【常规微调：数学引擎接管】
    float error = Calculate_Line_Error(sensor_val);
    last_valid_error = error;

    // 只下达目标航向指令，绝不碰 PWM
    target_yaw += (error * Kp_outer);

    // 【抗积分风转锁】物理墙：禁止目标角过度超前物理角度
    if (target_yaw - current_yaw > 35.0f) target_yaw = current_yaw + 35.0f;
    if (target_yaw - current_yaw < -35.0f) target_yaw = current_yaw - 35.0f;

    // 欧拉角环绕归一化
    if (target_yaw > 180.0f) target_yaw -= 360.0f;
    else if (target_yaw < -180.0f) target_yaw += 360.0f;

    osDelay(10); // 常规出口绝不饿死 CPU
}