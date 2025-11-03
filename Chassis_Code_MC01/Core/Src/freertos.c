/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Balance_Task.h"
#include "A1_Task.h"
#include "LK9025_Task.h"
#include "Monitor_Task.h"
#include "Communicate_Task.h"

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
osThreadId ImuTaskHandle;
osThreadId MonitorTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId A1_Tx_Task_Handle;
osThreadId LK9025_Tx_Task_Handle;
osThreadId OdomTaskHandle;
osThreadId CommunicateTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	taskENTER_CRITICAL();
	
 osThreadDef(Monitor_Task, Monitor_Task, osPriorityNormal,0,128);
  MonitorTaskHandle = osThreadCreate(osThread(Monitor_Task), NULL);
    taskEXIT_CRITICAL();
	
	INS_Init();
	taskENTER_CRITICAL();
    osThreadDef(INS_Task, INS_Task, osPriorityRealtime,0,512);
  ImuTaskHandle = osThreadCreate(osThread(INS_Task), NULL);
    
    osThreadDef(Balance_Task, Balance_Task, osPriorityAboveNormal,0,1024);
  ChassisTaskHandle = osThreadCreate(osThread(Balance_Task), NULL);
    
	osThreadDef(A1_Tx_Task, A1_Tx_Task, osPriorityNormal,0,256);
  A1_Tx_Task_Handle = osThreadCreate(osThread(A1_Tx_Task), NULL);
	
	osThreadDef(LK9025_Tx_Task, LK9025_Tx_Task, osPriorityNormal,0,256);
  LK9025_Tx_Task_Handle = osThreadCreate(osThread(LK9025_Tx_Task), NULL);
	
	osThreadDef(Communicate_Task, Communicate_Task, osPriorityBelowNormal,0,128);
  CommunicateTaskHandle = osThreadCreate(osThread(Communicate_Task ), NULL);
	
  User_Init();
  
  
	vTaskDelete(defaultTaskHandle);	//删锟斤拷锟斤拷始锟斤拷锟斤拷锟斤�?
	taskEXIT_CRITICAL();            //锟剿筹拷锟劫斤拷锟斤�?
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
