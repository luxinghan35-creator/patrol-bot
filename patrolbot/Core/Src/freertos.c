/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_driver.h"
#include "tim.h"
#include "gpio.h"
#include <stdio.h>
#include "string.h"
#include "usart.h"
#include "i2c_bus.h"
#include "pid.h"
#include "encoder.h"
#include "gy906.h"
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// 0: 系统正常运行 | 1: 触发超温熔断，底盘锁死
volatile uint8_t system_alarm_flag = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotionTask */
osThreadId_t MotionTaskHandle;
const osThreadAttr_t MotionTask_attributes = {
  .name = "MotionTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMotionTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotionTask */
  MotionTaskHandle = osThreadNew(StartMotionTask, NULL, &MotionTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  extern volatile float current_yaw;
  extern volatile uint32_t rx_frame_cnt;

  // 1. 初始化 OLED (屏幕点亮)
  OLED_Init();
  /* Infinite loop */
  for(;;)
  {
    float obj_temp = R3X_GY906_Read_ObjTemp();

    // 【新增：业务判定逻辑】
    // 过滤掉 -999 等通信错误码，只在真实高温时触发报警
    if (obj_temp > 60.0f && obj_temp < 500.0f) {
      system_alarm_flag = 1; // 触发熔断
    } else if (obj_temp < 50.0f) {
      system_alarm_flag = 0; // 温度降下来后，自动解除熔断 (也可以设计为必须手动复位)
    }

    OLED_NewFrame();
    char buf[32];

    if (system_alarm_flag == 1) {
      // 【警报画面】：反色闪烁，极具视觉冲击力
      OLED_PrintASCIIString(0, 0, "!! ALARM !!", &afont16x8, OLED_COLOR_REVERSED);
      sprintf(buf, "TEMP: %d C", (int)obj_temp);
      OLED_PrintASCIIString(0, 20, buf, &afont16x8, OLED_COLOR_NORMAL);
      OLED_PrintASCIIString(0, 40, "MOTORS LOCKED", &afont16x8, OLED_COLOR_NORMAL);
    } else {
      // 【正常画面】：你原来的遥测界面
      sprintf(buf, "Temp: %d C", (int)obj_temp);
      OLED_PrintASCIIString(0, 0, buf, &afont16x8, OLED_COLOR_NORMAL);

      sprintf(buf, "Yaw: %d", (int)current_yaw);
      OLED_PrintASCIIString(0, 20, buf, &afont16x8, OLED_COLOR_NORMAL);

      sprintf(buf, "SYS OK | %lu", rx_frame_cnt);
      OLED_PrintASCIIString(0, 40, buf, &afont16x8, OLED_COLOR_NORMAL);
    }

    OLED_ShowFrame();

    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMotionTask */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
/**
* @brief Function implementing the MotionTask thread.
* @param argument: Not used
* @retval None
*/
/**
 * @brief  运动控制与里程计核心任务 (MotionTask)
 * @param  argument: FreeRTOS 任务传参 (未使用)
 * @retval None
 * @note   【硬件资源占用清单】
 * - 动力输出: TIM2 (CH1~CH4) 控制四路电机 PWM
 * - 左轮感知: TIM3 (Encoder Mode)
 * - 右轮感知: TIM1 (Encoder Mode)
 * - 姿态感知: MPU6050 (I2C 轮询读取 DMP 解算)
 * - 遥测链路: USART3 高速串口上报
 */
/* USER CODE END Header_StartMotionTask */
void StartMotionTask(void *argument)
{
  /* USER CODE BEGIN StartMotionTask */
  // 1. 底层硬件初始化
  R3X_Encoder_Init();

  // 2. 算法层初始化 (实例化偏航角控制器)
  PID_Controller yaw_pid;
  PID_Init(&yaw_pid, 10.0f, 2.0f); // 你的 Kp 和 Kd

  // 3. 基础参数
  int base_pwm = 500;
  int rear_offset = 30;
  int16_t speed_L = 0;
  int16_t speed_R = 0;

  extern volatile float current_yaw;
  extern volatile float target_yaw;

  osDelay(500);
  target_yaw = current_yaw;

  /* Infinite loop */
  for(;;)
  {

    // 【新增：最高优先级的物理熔断拦截】
    if (system_alarm_flag == 1) {
      // 强行切断所有动力
      R3X_Set_Speed(MOTOR_FL, 0);
      R3X_Set_Speed(MOTOR_RL, 0);
      R3X_Set_Speed(MOTOR_FR, 0);
      R3X_Set_Speed(MOTOR_RR, 0);
      osDelay(10);
      continue; // 直接跳过后续的 PID 算法和电机控制，陷入安全锁死循环
    }

    // A：感知层 (获取轮速)
    speed_L = R3X_Get_Left_Speed();
    speed_R = R3X_Get_Right_Speed();

    // B：算法层 (计算修正量)
    float correction = PID_Calc_Yaw(&yaw_pid, current_yaw, target_yaw);

    // C：决策层 (生成带补偿的差速)
    int left_pwm  = base_pwm - (int)correction;
    int right_pwm = base_pwm + (int)correction;

    // D：执行层 (调用BSP控制电机)
    // 你的 motor_driver.c 已经自带了 0-1000 的限幅和方向引脚控制
    // 所以直接把计算结果塞给它就行，不用再写一堆 if-else 和 HAL_GPIO_WritePin
    R3X_Set_Speed(MOTOR_FL, left_pwm);
    R3X_Set_Speed(MOTOR_RL, left_pwm - rear_offset);
    R3X_Set_Speed(MOTOR_FR, right_pwm);
    R3X_Set_Speed(MOTOR_RR, right_pwm - rear_offset);

    osDelay(10);
  }
  /* USER CODE END StartMotionTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

