/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "Motor.h"
#include "niming.h"
#include "pid.h"
#include "Tracking.h"
#include "jy61.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern float speed1;
extern float speed2;
extern float Motor_Speet_Up;
extern float Mileage;

extern uint8_t Jy61_Rx;
extern float Jy61_X_Angle;
extern float Jy61_Y_Angle;
extern float Jy61_Z_Angle;
extern float Jy61_Angle_Temp;

extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t g_TaskSelect = 0;       /* 0=idle, 1-4=task */
uint8_t g_TaskRunning = 0;      /* 任务运行中标志 */
NavState g_NavState = NAV_STATE_IDLE;
uint8_t g_WaypointCount = 0;    /* 已通过的顶点数 */
uint8_t g_TargetWaypoints = 0;  /* 目标顶点数 */
uint8_t g_PointDetected = 0;    /* 顶点检测标志 */
uint8_t g_IndicateDone = 0;     /* 声光提示完成 */
uint8_t g_PrevAllBlack = 0;     /* 前一帧全黑状态（用于上升沿检测） */
uint32_t g_PointTick = 0;       /* 顶点检测时间戳 */
uint32_t g_TaskStartTick = 0;   /* 任务启动时间戳 */
const float g_BaseSpeed = 1.7f; /* 基础速度 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void System_Indicate(void);
static void Task_Start(uint8_t task);
static void Nav_Run(void);
static uint8_t Point_Detected(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  Motor_Init();
  PID_init();
  HAL_UART_Receive_IT(&huart3, &Jy61_Rx, 1);
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
  printf("System Ready. Task=%d\r\n", g_TaskSelect);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ---- 空闲：等待按键选择任务 ---- */
    if (g_TaskSelect == TASK_NONE && g_TaskRunning == 0)
    {
      /* LED慢闪表示空闲 */
      static uint32_t idleTick = 0;
      if (HAL_GetTick() - idleTick > 500)
      {
        HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
        idleTick = HAL_GetTick();
      }
      /* 检查按键 */
      if (g_TaskSelect != TASK_NONE)
      {
        HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
        printf("Task %d selected. Starting in 2s...\r\n", g_TaskSelect);
        HAL_Delay(2000);
        Task_Start(g_TaskSelect);
      }
    }

    /* ---- 任务运行中 ---- */
    if (1)
    {
      Nav_Run();
    }

    /* 调试输出 */
    static uint32_t dbgTick = 0;
    if (HAL_GetTick() - dbgTick > 200)
    {
      printf("T%d S%d WP%d/%d Z:%.1f S1:%.2f S2:%.2f\r\n",
             g_TaskSelect, g_NavState, g_WaypointCount, g_TargetWaypoints,
             Jy61_Z_Angle, speed1, speed2);
      dbgTick = HAL_GetTick();
    }

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* 顶点检测：6个传感器都看到黑线（上升沿检测，防止重复计数） */
static uint8_t Point_Detected(void)
{
  uint8_t allBlack = Tracking_All_Black();
  uint8_t result = 0;

  if (allBlack && !g_PrevAllBlack)
  {
    result = 1;
  }
  g_PrevAllBlack = allBlack;
  return result;
}

/* 声光提示：LED + 蜂鸣器 */
static void System_Indicate(void)
{
  for (int i = 0; i < 2; i++)
  {
    HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(150);
    HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    HAL_Delay(150);
  }
}

/* 根据任务号初始化 */
static void Task_Start(uint8_t task)
{
  g_NavState = NAV_STATE_LINE_FOLLOW;
  g_WaypointCount = 0;
  g_PointDetected = 0;
  g_IndicateDone = 0;
  g_PrevAllBlack = 0;
  g_TaskRunning = 1;
  g_TaskStartTick = HAL_GetTick();
  Mileage = 0;

  switch (task)
  {
    case TASK_1:
      /* A→B: 经过B点即停止 */
      g_TargetWaypoints = 1;
      break;
    case TASK_2:
      /* A→B→弧→C→D→弧→A: 经过B,C,D,A共4个点 */
      g_TargetWaypoints = 4;
      break;
    case TASK_3:
      /* A→C→弧→B→D→弧→A (8字): 经过C,B,D,A共4个点 */
      g_TargetWaypoints = 4;
      break;
    case TASK_4:
      /* 8字4圈: 每圈4个点 × 4圈 = 16个点 */
      g_TargetWaypoints = 16;
      break;
    default:
      g_TargetWaypoints = 0;
      g_TaskRunning = 0;
      return;
  }

  printf("Task %d started. Target waypoints: %d\r\n", task, g_TargetWaypoints);
}

/* 导航主循环 */
static void Nav_Run(void)
{
  float leftSpeed, rightSpeed;

  switch (NAV_STATE_LINE_FOLLOW)
  {
    case NAV_STATE_LINE_FOLLOW:
      /* 巡线行驶 */
      Tracking_LineFollow(g_BaseSpeed, &leftSpeed, &rightSpeed);
      Motor_PID_Go_SetSpeed(leftSpeed, rightSpeed);

      /* 检测顶点 */
      if (Point_Detected())
      {
        g_WaypointCount++;
        printf("Waypoint %d/%d reached!\r\n", g_WaypointCount, g_TargetWaypoints);
        g_NavState = NAV_STATE_POINT_INDICATE;
        g_IndicateDone = 0;
      }
      break;

    case NAV_STATE_POINT_INDICATE:
      /* 停车 + 声光提示 */
      Motor_PID_Go_SetSpeed(0, 0);
      if (!g_IndicateDone)
      {
        System_Indicate();
        g_IndicateDone = 1;

        /* 检查是否完成 */
        if (g_WaypointCount >= g_TargetWaypoints)
        {
          g_NavState = NAV_STATE_FINISH;
        }
        else
        {
          g_NavState = NAV_STATE_LINE_FOLLOW;
          /* 短暂前进以离开顶点区域，避免重复检测 */
          Motor_PID_Go_SetSpeed(g_BaseSpeed, g_BaseSpeed);
          HAL_Delay(200);
        }
      }
      break;

    case NAV_STATE_FINISH:
      /* 任务完成 */
      Motor_PID_Go_SetSpeed(0, 0);
      g_TaskRunning = 0;
      g_TaskSelect = TASK_NONE;
      for (int i = 0; i < 4; i++)
      {
        HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
      }
      printf("Task complete! Total waypoints: %d, Time: %lums\r\n",
             g_WaypointCount, HAL_GetTick() - g_TaskStartTick);
      g_NavState = NAV_STATE_IDLE;
      break;

    default:
      Motor_PID_Go_SetSpeed(0, 0);
      g_NavState = NAV_STATE_IDLE;
      break;
  }
}

/* JY61 串口接收回调 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    Jy61_RXData();
  }
  HAL_UART_Receive_IT(&huart3, &Jy61_Rx, 1);
}

/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif /* USE_FULL_ASSERT */
