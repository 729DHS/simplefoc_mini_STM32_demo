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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5600.h"
#include "foc.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUF_SIZE    64      /* 串口接收缓冲区大小         */
#define FOC_TICK_HZ         1098.0f /* TIM2 溢出频率 (72M/65536) */
#define AS5600_READ_DIV     10      /* 每 10 次 tick 读一次 I2C  */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* ---- 电机控制器 ---- */
MotorController motor;

/* ---- AS5600 编码器 ---- */
volatile float    g_mech_angle_rad = 0.0f;  /* 机械角度 (rad)      */
volatile uint16_t g_raw_angle      = 0;     /* 原始角度 (0-4095)   */

/* ---- 定时标志 ---- */
volatile uint8_t  foc_tick   = 0;           /* TIM2 溢出标志       */
volatile uint16_t tick_count = 0;           /* 总 tick 计数        */

/* ---- UART 接收 ---- */
volatile uint8_t  uart_rx_byte;             /* 单字节接收缓冲       */
volatile char     uart_rx_buf[UART_RX_BUF_SIZE];
volatile uint8_t  uart_rx_idx   = 0;
volatile uint8_t  uart_cmd_ready = 0;

/* ---- 自动打印开关 ---- */
volatile uint8_t  auto_print_enabled = 1;   /* 1=每500ms自动打印 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void UART_ParseCommand(char *cmd);
static void UART_PrintStatus(void);
static void UART_SendString(const char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 初始化电机控制器: 7 极对, PWM 周期 65535
   * dt = AS5600_READ_DIV / FOC_TICK_HZ ≈ 0.0091s → ~110Hz 控制频率 */
  Motor_Init(&motor, 7, __HAL_TIM_GET_AUTORELOAD(&htim1),
             (float)AS5600_READ_DIV / FOC_TICK_HZ);

  /* 硬件使能 SimpleFOC Mini (PA11 = HIGH)
   * 注意: 软件使能 (motor.enabled = 1) 要等第一次读完传感器再开
   *       否则 target_position 会被锁定为 0 (还没读到编码器) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

  /* 设置伺服参数: 低增益起步，避免震荡 */
  Motor_SetVoltage(&motor, 0.4f);              /* 最大 40% 电压       */
  Motor_SetPID(&motor, 0.1f, 0.02f, 0.03f);   /* Kp Ki Kd, 2804 12N14P 低增益起步 */

  /* ---- 启动诊断: 确认初始化后的状态 ---- */
  {
    char init_diag[80];
    snprintf(init_diag, sizeof(init_diag),
      "INIT: mode=%d enabled=%d pp=%d vlim=%.2f dt=%.4f\r\n",
      (int)motor.mode, (int)motor.enabled, (int)motor.pole_pairs,
      (double)motor.voltage_limit, (double)motor.dt);
    UART_SendString(init_diag);
  }

  /* 启动 TIM1 三路 PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  /* 启动 TIM2 周期中断 */
  HAL_TIM_Base_Start_IT(&htim2);

  /* 启动 USART1 中断接收 */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart_rx_byte, 1);

  /* 打印帮助 */
  UART_SendString("\r\n");
  UART_SendString("========================================\r\n");
  UART_SendString("  SimpleFOC Position Servo v2.1\r\n");
  UART_SendString("========================================\r\n");
  UART_SendString("  模式:\r\n");
  UART_SendString("   M0 - 开环速度模式  M1 - 位置伺服模式\r\n");
  UART_SendString("  伺服命令:\r\n");
  UART_SendString("   T<deg>    目标角度    (如 T90, T-45)\r\n");
  UART_SendString("   V<0~1>    电压限幅    (如 V0.3)\r\n");
  UART_SendString("   Kp/Ki/Kd<n> PID 增益 (如 Kp0.5)\r\n");
  UART_SendString("   E1/E0     使能/禁用\r\n");
  UART_SendString("   P<n>      极对数      (如 P7)\r\n");
  UART_SendString("   S<rpm>    开环转速    (如 S60)\r\n");
  UART_SendString("  诊断:\r\n");
  UART_SendString("   ?         完整状态\r\n");
  UART_SendString("   D         编码器原始值 (持续打印5次)\r\n");
  UART_SendString("   C         切换自动状态打印 (开/关)\r\n");
  UART_SendString("   H         本帮助\r\n");
  UART_SendString("========================================\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ===== FOC 控制更新 (同步传感器速率, ~110Hz) ===== */
    if (foc_tick) {
      foc_tick = 0;
      tick_count++;

      /* 只在读到新传感器数据时才执行 PID + PWM 更新
       * 避免用旧角度重复计算导致积分震荡 */
      static uint16_t last_ctrl_tick = 0;
      if (tick_count - last_ctrl_tick >= AS5600_READ_DIV) {
        last_ctrl_tick = tick_count;

        /* 读取 AS5600 角度 */
        uint16_t raw = AS5600_ReadRawAngle(&hi2c1);
        if (raw != 0xFFFF) {
          g_raw_angle = raw;
          g_mech_angle_rad = (float)raw * 6.283185307f / 4096.0f;
          Motor_SetMechanicalAngle(&motor, g_mech_angle_rad);

          /* 首次读到有效传感器数据：使能电机 + 锁定当前位置
           * 防止 target_position=0 导致电机跳半圈
           * 同时强制设置模式为 SERVO (防御 Motor_Init 模式没有写入的情况) */
          static uint8_t first_read = 1;
          if (first_read) {
            first_read = 0;
            motor.mode            = MODE_POSITION_SERVO;
            motor.target_position = g_mech_angle_rad;
            motor.enabled         = 1;
            motor.electrical_angle = 0.0f;
            PID_Reset(&motor.pid);
            UART_SendString("BOOT: first sensor OK, servo locked.\r\n");
          }
        }

        /* 新角度 → 运行一次 PID → 更新 PWM */
        Motor_UpdatePWM(&motor, &htim1);
      }
    }

    /* ===== UART 命令处理 ===== */
    if (uart_cmd_ready) {
      uart_cmd_ready = 0;
      UART_ParseCommand((char *)uart_rx_buf);
      uart_rx_idx = 0;
      /* 注意: 不在这里调 HAL_UART_Receive_IT
       * 中断回调 HAL_UART_RxCpltCallback 已经持续在接收下一个字节 */
    }

    /* ===== 自动打印状态 (每 ~500ms, 可通过 'C' 命令开关) ===== */
    static uint16_t print_tick = 0;
    if (auto_print_enabled && tick_count - print_tick >= 550) {
      print_tick = tick_count;
      char buf[96];
      snprintf(buf, sizeof(buf),
        "> A:%4u deg:%6.1f err:%+5.1f tgt:%5.1f v:%.2f %s\r\n",
        g_raw_angle,
        (double)(g_mech_angle_rad * 57.29578f),
        (double)((motor.target_position - g_mech_angle_rad) * 57.29578f),
        (double)(motor.target_position * 57.29578f),
        motor.voltage_limit,
        motor.enabled ? (motor.mode == MODE_POSITION_SERVO ? "[SERVO]" : "[SPEED]") : "[OFF]"
      );
      UART_SendString(buf);
    }

    /* USER CODE END 3 */
  }
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ================================================================
 *                    HAL 回调函数
 * ================================================================ */

/**
 * @brief TIM 周期溢出回调 — 由 HAL_TIM_IRQHandler 调用
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    foc_tick = 1;         /* 通知主循环更新控制 */
  }
}

/**
 * @brief UART 接收完成回调 — 单字节接收模式
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    uint8_t byte = uart_rx_byte;

    /* 收到换行 → 命令就绪 */
    if (byte == '\n' || byte == '\r') {
      if (uart_rx_idx > 0 && !uart_cmd_ready) {
        uart_rx_buf[uart_rx_idx] = '\0';
        uart_cmd_ready = 1;
      }
    }
    /* 普通字符 → 写入缓冲区 (上次命令未处理完则丢弃) */
    else if (uart_rx_idx < UART_RX_BUF_SIZE - 1 && !uart_cmd_ready) {
      uart_rx_buf[uart_rx_idx++] = (char)byte;
    }

    /* 重新启用中断接收下一个字节 */
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart_rx_byte, 1);
  }
}

/* ================================================================
 *                    UART 辅助函数
 * ================================================================ */

/**
 * @brief 发送字符串到串口
 */
static void UART_SendString(const char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

/**
 * @brief 打印当前状态
 */
static void UART_PrintStatus(void)
{
  char buf[320];
  snprintf(buf, sizeof(buf),
    "--- Status ------------------\r\n"
    " Mode:     %s (%d)\r\n"
    " Enabled:  %d\r\n"
    " Voltage:  %.2f\r\n"
    " Pole Pairs: %d\r\n"
    " Raw Angle:  %u / 4095\r\n"
    " Mech Angle: %.2f rad (%.1f deg)\r\n"
    " Elec Angle: %.2f rad\r\n"
    " Target Pos: %.1f deg\r\n"
    " Pos Error:  %+.1f deg\r\n"
    " PID Kp=%.3f Ki=%.3f Kd=%.3f\r\n"
    " Ticks: %u\r\n"
    "-----------------------------\r\n",
    motor.mode == MODE_POSITION_SERVO ? "SERVO" : "SPEED",
    (int)motor.mode,
    motor.enabled,
    motor.voltage_limit,
    motor.pole_pairs,
    g_raw_angle,
    g_mech_angle_rad,
    (double)(g_mech_angle_rad * 57.29578f),
    motor.electrical_angle,
    (double)(motor.target_position * 57.29578f),
    (double)((motor.target_position - g_mech_angle_rad) * 57.29578f),
    motor.pid.Kp, motor.pid.Ki, motor.pid.Kd,
    tick_count
  );
  UART_SendString(buf);
}

/**
 * @brief 解析串口命令
 *
 *  协议: 单字母命令 + 可选参数，换行结束。大小写不敏感。
 *  @note atof() 始终可用 (stdlib), snprintf %f 需要 -u _printf_float
 */
static void UART_ParseCommand(char *cmd)
{
  /* 跳过前导空白 */
  while (*cmd == ' ' || *cmd == '\t') cmd++;
  if (*cmd == '\0') return;

  char op = *cmd++;
  char buf[64];  /* 通用反馈缓冲区 */

  switch (op) {

    /* ==================== 模式切换 ==================== */
    case 'M': case 'm': {
      int m = atoi(cmd);
      if (m == 0) {
        Motor_SetMode(&motor, MODE_OPENLOOP_SPEED);
        UART_SendString("OK Mode -> OPENLOOP SPEED\r\n");
      } else if (m == 1) {
        Motor_SetMode(&motor, MODE_POSITION_SERVO);
        UART_SendString("OK Mode -> POSITION SERVO\r\n");
      } else {
        UART_SendString("ERR M0=Speed M1=Servo\r\n");
      }
      break;
    }

    /* ==================== 目标角度 (伺服模式) ==================== */
    case 'T': case 't': {
      float deg = (float)atof(cmd);
      float rad = deg * 0.0174532925f;  /* deg → rad */
      Motor_SetTargetPosition(&motor, rad);
      snprintf(buf, sizeof(buf), "OK Target -> %.1f deg (%.2f rad)\r\n",
               (double)deg, (double)rad);
      UART_SendString(buf);
      break;
    }

    /* ==================== PID 参数 ==================== */
    case 'K': case 'k': {
      if (*cmd == 'p' || *cmd == 'P') {
        motor.pid.Kp = (float)atof(cmd + 1);
        snprintf(buf, sizeof(buf), "OK Kp=%.3f\r\n", (double)motor.pid.Kp);
        UART_SendString(buf);
      } else if (*cmd == 'i' || *cmd == 'I') {
        motor.pid.Ki = (float)atof(cmd + 1);
        snprintf(buf, sizeof(buf), "OK Ki=%.3f\r\n", (double)motor.pid.Ki);
        UART_SendString(buf);
      } else if (*cmd == 'd' || *cmd == 'D') {
        motor.pid.Kd = (float)atof(cmd + 1);
        snprintf(buf, sizeof(buf), "OK Kd=%.3f\r\n", (double)motor.pid.Kd);
        UART_SendString(buf);
      } else {
        UART_SendString("ERR Usage: Kp<n> Ki<n> Kd<n>\r\n");
      }
      PID_Reset(&motor.pid);
      break;
    }

    /* ==================== 转速 (开环模式) ==================== */
    case 'S': case 's': {
      float rpm = (float)atof(cmd);
      Motor_SetSpeed(&motor, rpm);
      snprintf(buf, sizeof(buf), "OK Speed -> %.0f RPM\r\n", (double)rpm);
      UART_SendString(buf);
      break;
    }

    /* ==================== 电压限幅 ==================== */
    case 'V': case 'v': {
      float v = (float)atof(cmd);
      Motor_SetVoltage(&motor, v);
      snprintf(buf, sizeof(buf), "OK Voltage -> %.2f\r\n", (double)v);
      UART_SendString(buf);
      break;
    }

    /* ==================== 使能/禁用 ==================== */
    case 'E': case 'e': {
      int en = atoi(cmd);
      Motor_Enable(&motor, (uint8_t)(en != 0));
      if (motor.enabled) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
        UART_SendString("OK Motor ENABLED\r\n");
      } else {
        uint16_t neutral = motor.pwm_period >> 1;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, neutral);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, neutral);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, neutral);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
        UART_SendString("OK Motor DISABLED (PWM=50%)\r\n");
      }
      break;
    }

    /* ==================== 极对数 ==================== */
    case 'P': case 'p': {
      int pp = atoi(cmd);
      if (pp >= 1 && pp <= 100) {
        motor.pole_pairs = (uint8_t)pp;
        snprintf(buf, sizeof(buf), "OK Pole pairs -> %d\r\n", pp);
        UART_SendString(buf);
      } else {
        UART_SendString("ERR Pole pairs must be 1-100\r\n");
      }
      break;
    }

    /* ==================== 切换自动打印 ==================== */
    case 'C': case 'c':
      auto_print_enabled = !auto_print_enabled;
      if (auto_print_enabled) {
        UART_SendString("OK Auto print ON (every 0.5s)\r\n");
      } else {
        UART_SendString("OK Auto print OFF\r\n");
      }
      break;

    /* ==================== 编码器诊断 ==================== */
    case 'D': case 'd': {
      UART_SendString("--- Encoder Diag (5 samples) ---\r\n");
      uint8_t magnet = AS5600_IsMagnetDetected(&hi2c1);
      snprintf(buf, sizeof(buf), " Magnet: %s\r\n", magnet ? "OK" : "MISSING!");
      UART_SendString(buf);
      for (int i = 0; i < 5; i++) {
        uint16_t raw = AS5600_ReadRawAngle(&hi2c1);
        float deg = (float)raw * 360.0f / 4096.0f;
        snprintf(buf, sizeof(buf),
                 " [%d] raw=%4u -> %.1f deg (0x%04X)\r\n",
                 i + 1, raw, (double)deg, raw);
        UART_SendString(buf);
        HAL_Delay(5);  /* 等 5ms 再读下一个 */
      }
      UART_SendString("--- End Diag ---\r\n");
      break;
    }

    /* ==================== 完整状态 ==================== */
    case '?':
      UART_PrintStatus();
      break;

    /* ==================== 帮助 ==================== */
    case 'H': case 'h':
      UART_SendString("M0/M1  T<deg>  V<n>  Kp/Ki/Kd<n>  S<rpm>\r\n");
      UART_SendString("E1/E0  P<n>    C(toggle_auto)  D(diag)  ?(status)\r\n");
      break;

    default:
      snprintf(buf, sizeof(buf), "ERR Unknown cmd '%c'. H for help\r\n", op);
      UART_SendString(buf);
      break;
  }
}

/* USER CODE END 4 */

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
