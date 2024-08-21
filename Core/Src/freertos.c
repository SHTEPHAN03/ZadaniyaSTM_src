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
#include "usart.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define bitcheck(byte,nbit) (((byte) & (1<<(nbit)))>>nbit)
#define bitflip(byte,nbit)  ((byte) ^=  (1<<(nbit)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
	uint8_t numByte = 0;
	uint8_t byteArr[3];
	uint8_t i = 0;
	uint16_t znach = 0;

	uint8_t H, S, V;
	uint8_t Hi, a;
	uint8_t R, G, B;
	float Vmin, Vinc, Vdec;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LedTask */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Set_PWM_Duty_Cycle();
	void colorLib ()
 {
	H = znach/13;
	V = 85;
	S = 100;

	Hi = (H / 56) % 6;
	Vmin = ((100 - S) * V) / 100;
	a = (V - Vmin) * ((H % 56) / 56);
	Vinc = Vmin + a;
	Vdec = V - a;

	V = V * 255 / 100;
	Vdec = Vdec * 255 / 100;
	Vinc = Vinc * 255 / 100;
	Vmin = Vmin * 255 / 100;

	   if (Hi == 0) {
	      R = V;
	      G = Vinc;
	      B = Vmin;
	   }
	   else if (Hi == 1) {
	      R = Vdec;
	      G = V;
	      B = Vmin;
	   }
	   else if (Hi == 2) {
	      R = Vmin;
	      G = V;
	      B = Vinc;
	   }
	   else if (Hi == 3) {
	      R = Vmin;
	      G = Vdec;
	      B = V;
	   }
	   else if (Hi == 4) {
	      R = Vinc;
	      G = Vmin;
	      B = V;
	   }
	   else if (Hi == 5) {
	      R = V;
	      G = Vmin;
	      B = Vdec;
	   }

}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartLedTask(void *argument);

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

  /* creation of LedTask */
  LedTaskHandle = osThreadNew(StartLedTask, NULL, &LedTask_attributes);

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
  /* Infinite loop */
  for(;;)
  {
	  if (HAL_UART_Receive(&huart1, &numByte,1,50) == HAL_OK){
	  if (bitcheck(numByte, 7) == 0){
		  i = 0;
		  byteArr[i]=numByte;
	  }
	  else  {
		  i++;
		  byteArr[i]=numByte;
	  }
	  if (i==2){
		  if (bitcheck(byteArr[2],7) != bitcheck(byteArr[0],5)){
			  bitflip(byteArr[2],7);
	  }
		  znach=byteArr[2]*17;
	  }
	  }

	  osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  for(;;)
  {
	  colorLib();
	  Set_PWM_Duty_Cycle(&htim3, TIM_CHANNEL_1, R);
	  Set_PWM_Duty_Cycle(&htim3, TIM_CHANNEL_2, G);
	  Set_PWM_Duty_Cycle(&htim3, TIM_CHANNEL_3, B);
  }
    osDelay(1);
}
  /* USER CODE END StartLedTask */


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

