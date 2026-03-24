/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_driver.h"
#include <stdio.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// GCC 编译器的 printf 重定向底层函数
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}
#endif

// JY61P 串口状态机专属变量
uint8_t rx_byte;
uint8_t rx_buffer[11];
uint8_t rx_index = 0;

// 【核心修改】必须加 volatile！强制每次都从内存读取最新姿态！
volatile float current_yaw = 0.0f;
volatile float target_yaw = 0.0f;

// 【新增探针】用于监视状态机是否健康解析
volatile uint32_t rx_frame_cnt = 0;

extern osSemaphoreId_t oled_dma_sem;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // 启动 JY61P 专属通道 (USART3) 的单字节中断接收
  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  R3X_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    // --- 工业级强逻辑状态机 ---
    if (rx_index == 0) {
      if (rx_byte == 0x55) { // 第一步：必须踩中 0x55 帧头
        rx_buffer[0] = rx_byte;
        rx_index = 1;
      }
    } else if (rx_index == 1) {
      if (rx_byte == 0x53) { // 第二步：必须是角度包 0x53
        rx_buffer[1] = rx_byte;
        rx_index = 2;
      } else {
        // 如果是 0x51(加速度), 0x52(角速度)，直接踢掉，重新找 0x55
        rx_index = 0;
      }
    } else {
      // 第三步：无脑接收剩余的 9 个字节
      rx_buffer[rx_index] = rx_byte;
      rx_index++;
      if (rx_index >= 11) {
        // 完美收齐 11 字节，解算角度
        int16_t yaw_raw = (rx_buffer[7] << 8) | rx_buffer[6];
        current_yaw = ((float)yaw_raw / 32768.0f) * 180.0f;
        rx_frame_cnt++; // 解析帧数暴涨，证明系统健康！
        rx_index = 0;
      }
    }
    // 永动机重启
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  }
}

// 防死锁抗体保持不变
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  }
}

// DMA 搬砖完成中断回调
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    // DMA 完工，释放信号量，瞬间唤醒挂起的 DefaultTask
    osSemaphoreRelease(oled_dma_sem);
  }
}

// I2C 错误中断回调（专门用来扑灭中断风暴）
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    // 【终极杀招】在停掉 DMA 之前，先暴力屏蔽 I2C1 的所有硬件中断！
    // 彻底掐死 TXE 标志位向 CPU 告状的通道，直接从物理层阻断风暴！
    __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF);

    // 然后再拔掉 DMA 的氧气管
    HAL_DMA_Abort(hi2c->hdmatx);

    // 信号量依然不释放。
    // 等 DefaultTask 里 1000ms 超时后，APP 层会调用 R3X_I2C_Reset()。
    // R3X_I2C_Reset() 里的 MX_I2C1_Init() 会自动把刚才关掉的中断重新打开，实现完美的硬件浴火重生。
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
