/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "BMI088reg.h"
#include "Dwt_Timer.h"
#include "PID.h"
#include "Can_Bus.h"
#include "RS485.h"
#include "Configuration.h"



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
void A1_Init_Left(void);
void A1_Init_Right(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void User_Init(void)
{
	A1_Init_Left();
  A1_Init_Right();
	
//	PowerLimit.Flag = 0;
//	PowerLimit.RemainPower[2] = 60;
//	PowerLimit.Real_Power[2]  = 75;
//	PowerLimit.MaxSpeed       = CHASSIS_MAX_SPEED;//设定�??大转�??
//	Chassis.Position.Flag     = 1;

	Robot_Status.Body.Body_Upright_Status = NO;
  Robot_Status.Body.Body_Soar_Status = NO;
  Robot_Status.Leg.Leg_Init_Status = NO;
  Robot_Status.Leg.Leg_Height_Status = LEG_LOW;
  Robot_Status.Gimbal.Gimbal_Reposition_Status = NO;
  Robot_Status.INS.INS_Init_Status = NO;
  Robot_Status.INS.INS_Fail_Status = NO;


//  SpinTop.Speed = ROTATE_NORMAL_SPEED;
//	Command.SpeedMinish = 1;
//	Command.SpeedZoom = 1;
	

	//PID初始化分别为KP KI KD �??大输�?? 死区 积分范围 �??大输�?? �??大积分�?�（输出�??=�??大积分�??*KI�??
	//注意！！！！！！
	//积分范围只有在位置式PID计算的时候起作用 
		//底盘电机
	PID_Init(&Balance_Yaw_Position_pid,0.1,0,20.0,2.0f,0,50,10000,200,0.05f,POSITIONAL);//底盘跟随PD
	//PID_Init(&Balance_Yaw_Speed_pid,10,0,0,5,0,0,3,0,POSITIONAL);
	PID_Init(&Leg_theta_Harmonize_pid[0],150,0,0,3,0,0.05,2,1,0.001f,POSITIONAL);//关节电机抗劈叉PD
	//PID_Init(&Leg_theta_Harmonize_pid[1],0,0,0,10,0,0,4,0,POSITIONAL);//轮毂电机抗劈叉PD
	PID_Init(&Leg_ROLL_Compensate_pid[0],50,0,150,20,0,0,3.14,0,0.05f,POSITIONAL);
	PID_Init(&Leg_ROLL_Compensate_pid[1],50.0f,0.2,150,30,0,0.05f,3.14,15.0f,0.05f,POSITIONAL);
	PID_Init(&L0_pid[0],1.0f,0.0f,5,20,0,3,360,5.0f,0.05f,POSITIONAL);//腿长控制
	PID_Init(&L0_pid[1],1.0f,0.0f,5,20,0,3,360,5.0f,0.05f,POSITIONAL);
	
	 PID_Init(&TempCtrl_pid, 800, 0.02, 5.0f, 600, 0, 0, 100, 600,0.05, POSITIONAL);
}

void A1_Init_Left(void)
{
  A1_Transmit_Data_Left[0]  = 0xFE;
	A1_Transmit_Data_Left[1]  = 0xEE;
	A1_Transmit_Data_Left[2]  = 0xBB;
	A1_Transmit_Data_Left[3]  = 0x00;
	A1_Transmit_Data_Left[4]  = 0x05;
	A1_Transmit_Data_Left[5]  = 0xFF;
	A1_Transmit_Data_Left[6]  = 0x00;
	A1_Transmit_Data_Left[7]  = 0x00;
	A1_Transmit_Data_Left[8]  = 0x00;
	A1_Transmit_Data_Left[9]  = 0x00;
	A1_Transmit_Data_Left[10] = 0x00;
	A1_Transmit_Data_Left[11] = 0x00;
	A1_Transmit_Data_Left[12] = 0;
	A1_Transmit_Data_Left[13] = 0;
	A1_Transmit_Data_Left[14] = 0;
	A1_Transmit_Data_Left[15] = 0;
	A1_Transmit_Data_Left[16] = 0;
	A1_Transmit_Data_Left[17] = 0;
	A1_Transmit_Data_Left[18] = 0;
	A1_Transmit_Data_Left[19] = 0;
	A1_Transmit_Data_Left[20] = 0;
	A1_Transmit_Data_Left[21] = 0;
	A1_Transmit_Data_Left[22] = 0;
	A1_Transmit_Data_Left[23] = 0;
	A1_Transmit_Data_Left[24] = 0x00;
	A1_Transmit_Data_Left[25] = 0x00;
	A1_Transmit_Data_Left[26] = 0x00;
	A1_Transmit_Data_Left[27] = 0x00;
	A1_Transmit_Data_Left[28] = 0x00;
	A1_Transmit_Data_Left[29] = 0x00;
	A1_Transmit_Data_Left[30] = 0;
	A1_Transmit_Data_Left[31] = 0;
	A1_Transmit_Data_Left[32] = 0;
	A1_Transmit_Data_Left[33] = 0;
}

void A1_Init_Right(void)
{
  A1_Transmit_Data_Right[0]  = 0xFE;
	A1_Transmit_Data_Right[1]  = 0xEE;
	A1_Transmit_Data_Right[2]  = 0xBB;
	A1_Transmit_Data_Right[3]  = 0x00;
	A1_Transmit_Data_Right[4]  = 0x05;
	A1_Transmit_Data_Right[5]  = 0xFF;
	A1_Transmit_Data_Right[6]  = 0x00;
	A1_Transmit_Data_Right[7]  = 0x00;
	A1_Transmit_Data_Right[8]  = 0x00;
	A1_Transmit_Data_Right[9]  = 0x00;
	A1_Transmit_Data_Right[10] = 0x00;
	A1_Transmit_Data_Right[11] = 0x00;
	A1_Transmit_Data_Right[12] = 0;
	A1_Transmit_Data_Right[13] = 0;
	A1_Transmit_Data_Right[14] = 0;
	A1_Transmit_Data_Right[15] = 0;
	A1_Transmit_Data_Right[16] = 0;
	A1_Transmit_Data_Right[17] = 0;
	A1_Transmit_Data_Right[18] = 0;
	A1_Transmit_Data_Right[19] = 0;
	A1_Transmit_Data_Right[20] = 0;
	A1_Transmit_Data_Right[21] = 0;
	A1_Transmit_Data_Right[22] = 0;
	A1_Transmit_Data_Right[23] = 0;
	A1_Transmit_Data_Right[24] = 0x00;
	A1_Transmit_Data_Right[25] = 0x00;
	A1_Transmit_Data_Right[26] = 0x00;
	A1_Transmit_Data_Right[27] = 0x00;
	A1_Transmit_Data_Right[28] = 0x00;
	A1_Transmit_Data_Right[29] = 0x00;
	A1_Transmit_Data_Right[30] = 0;
	A1_Transmit_Data_Right[31] = 0;
	A1_Transmit_Data_Right[32] = 0;
	A1_Transmit_Data_Right[33] = 0;
}
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
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_CAN2_Init();
  MX_UART4_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init(168);
  HAL_Delay(50);
  CAN_Start();
  HAL_Delay(50);
  RS485_Start();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  
  while(BMI088_init(&hspi2,1) )
  {
      ;
  }
  Robot_Status.INS.INS_Init_Status = YES;
  HAL_Delay(50);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  if(htim->Instance == TIM5)
  {
	Status_Counter[0]++;
  }
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

#ifdef  USE_FULL_ASSERT
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
