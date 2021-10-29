/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "motor.h"
#include "tim.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)

#define MOTOR_NUM 4
#define PID_CYCLE 50

#define NO 0
#define FRONT 1
#define BACK 2

#define imuRxBufferLength 100
#define msgsRxBufferLength 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
pid_instance m[MOTOR_NUM];
short count[MOTOR_NUM];
const float k[MOTOR_NUM] = {0.2084f, 0.1926f, 0.2084f, 0.2084f};
uint16_t pwm[MOTOR_NUM];
float speedActual[MOTOR_NUM];

uint8_t dir = NO;
float speedSet;
float ang = 0;

extern uint8_t imuRxBuffer[imuRxBufferLength];
extern uint8_t imuIndex;	

int timeStart = 0;
int timeEnd = 0; 
		
extern uint8_t msgsRxBuffer[msgsRxBufferLength];							
extern uint8_t msgsRxLen;

EventGroupHandle_t xCreatedEventMSGS = NULL;
EventGroupHandle_t xCreatedEventIMU = NULL;
EventGroupHandle_t xCreatedEventGOLD = NULL;
/* USER CODE END Variables */
osThreadId PIDHandle;
osThreadId IMUHandle;
osThreadId GOLDHandle;
osThreadId MSGSHandle;
osThreadId STARTHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

inline void AppObjCreate(void)
{
	xCreatedEventMSGS = xEventGroupCreate();
	xCreatedEventIMU = xEventGroupCreate();
  xCreatedEventGOLD = xEventGroupCreate();
}

inline void motorInit(void)
{
  for(int i=0;i<MOTOR_NUM;i++)
  {
    m[i].Kp = 0.55f;m[i].Ki = 0.25f;m[i].Kd = -0.15f;
    pid_init(&m[i], 0);
  }
	dir = FRONT;
}
/* USER CODE END FunctionPrototypes */

void vTaskPID(void const * argument);
void vTaskIMU(void const * argument);
void vTaskGOLD(void const * argument);
void vTaskMSGS(void const * argument);
void vTaskSTART(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  AppObjCreate();
  motorInit();
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
  /* definition and creation of PID */
  osThreadDef(PID, vTaskPID, osPriorityLow, 0, 128);
  PIDHandle = osThreadCreate(osThread(PID), NULL);

  /* definition and creation of IMU */
  osThreadDef(IMU, vTaskIMU, osPriorityBelowNormal, 0, 128);
  IMUHandle = osThreadCreate(osThread(IMU), NULL);

  /* definition and creation of GOLD */
  osThreadDef(GOLD, vTaskGOLD, osPriorityNormal, 0, 256);
  GOLDHandle = osThreadCreate(osThread(GOLD), NULL);

  /* definition and creation of MSGS */
  osThreadDef(MSGS, vTaskMSGS, osPriorityAboveNormal, 0, 128);
  MSGSHandle = osThreadCreate(osThread(MSGS), NULL);

  /* definition and creation of START */
  osThreadDef(START, vTaskSTART, osPriorityIdle, 0, 128);
  STARTHandle = osThreadCreate(osThread(START), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_vTaskPID */
/**
  * @brief  Function implementing the PID thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_vTaskPID */
void vTaskPID(void const * argument)
{
  /* USER CODE BEGIN vTaskPID */
  TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    count[0] = __HAL_TIM_GetCounter(&htim2);
    count[1] = __HAL_TIM_GetCounter(&htim3);
    count[2] = __HAL_TIM_GetCounter(&htim4);
    count[3] = __HAL_TIM_GetCounter(&htim5);
    for(int i=0;i<MOTOR_NUM;i++)
    {
      speedActual[i] = k[i] * count[i];
      pwm[i] = pid(&m[i], speedSet - speedActual[i]);
    }
    #ifdef PID_INFO
      printf("%f,%f",speedActual,speedSet);
    #endif

    #ifdef ENCODER_INFO
    for(int i=0;i<MOTOR_NUM;i++)
      printf("count%d:%d", i, count[i]);
    printf("Time:%d", xLastWakeTime - xTaskGetTickCount());
    #else
    __HAL_TIM_SetCounter(&htim2, 0);
    __HAL_TIM_SetCounter(&htim3, 0);
    __HAL_TIM_SetCounter(&htim4, 0);
    __HAL_TIM_SetCounter(&htim5, 0);
    #endif

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm[0]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm[1]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm[2]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm[3]);

    if (pwm[0] >= 0)
    {
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm[0]);
    }
    else
    {
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -pwm[0]);
    }
    if (pwm[1] >= 0)
    {
      HAL_GPIO_WritePin(AIN3_GPIO_Port, AIN3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(AIN4_GPIO_Port, AIN4_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm[1]);
    }
    else
    {
      HAL_GPIO_WritePin(AIN3_GPIO_Port, AIN3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(AIN4_GPIO_Port, AIN4_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -pwm[1]);
    }
    if (pwm[2] >= 0)
    {
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm[2]);
    }
    else
    {
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -pwm[2]);
    }
    if (pwm[3] >= 0)
    {
      HAL_GPIO_WritePin(BIN3_GPIO_Port, BIN3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(BIN4_GPIO_Port, BIN4_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm[3]);
    }
    else
    {
      HAL_GPIO_WritePin(BIN3_GPIO_Port, BIN3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BIN4_GPIO_Port, BIN4_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -pwm[3]);
    }

    osDelayUntil(&xLastWakeTime, PID_CYCLE);
  }
  /* USER CODE END vTaskPID */
}

/* USER CODE BEGIN Header_vTaskIMU */
/**
* @brief Function implementing the IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskIMU */
void vTaskIMU(void const * argument)
{
  /* USER CODE BEGIN vTaskIMU */
  EventBits_t uxBits;
  /* Infinite loop */
  for(;;)
  {
    uxBits = xEventGroupWaitBits(xCreatedEventIMU, BIT_0, pdTRUE, pdFALSE, portMAX_DELAY);			
		if(uxBits == BIT_0)
		{
			while(imuRxBuffer[imuIndex] == 0x55)
			{
				switch(imuRxBuffer[imuIndex + 1])
				{
					case 0x53:
            ang = ((imuRxBuffer[imuIndex + 8] << 8) | imuRxBuffer[imuIndex + 7])*0.0054932f;
            break;
					default:
            printf("IMU data error!");
						break;
				}
			}
    }
  }
  /* USER CODE END vTaskIMU */
}

/* USER CODE BEGIN Header_vTaskGOLD */
/**
* @brief Function implementing the GOLD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskGOLD */
void vTaskGOLD(void const * argument)
{
  /* USER CODE BEGIN vTaskGOLD */
  /* Infinite loop */
  for(;;)
  {
    vTaskDelete(NULL);
    osDelay(1);
  }
  /* USER CODE END vTaskGOLD */
}

/* USER CODE BEGIN Header_vTaskMSGS */
/**
* @brief Function implementing the MSGS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskMSGS */
void vTaskMSGS(void const * argument)
{
  /* USER CODE BEGIN vTaskMSGS */
  EventBits_t uxBits;
  /* Infinite loop */
  for(;;)
  {
    uxBits = xEventGroupWaitBits(xCreatedEventMSGS, BIT_1 | BIT_0, pdTRUE, pdFALSE, portMAX_DELAY);
		if(uxBits == BIT_0)
		{						
			memset(msgsRxBuffer, 0, msgsRxBufferLength);
    }	
    else if(uxBits == BIT_1)
    {
      memset(msgsRxBuffer, 0, msgsRxBufferLength);
    }
  }
  /* USER CODE END vTaskMSGS */
}

/* USER CODE BEGIN Header_vTaskSTART */
/**
* @brief Function implementing the START thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskSTART */
void vTaskSTART(void const * argument)
{
  /* USER CODE BEGIN vTaskSTART */
  /* Infinite loop */
  for(;;)
  {
    vTaskDelete(NULL);
    osDelay(1);
  }
  /* USER CODE END vTaskSTART */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
