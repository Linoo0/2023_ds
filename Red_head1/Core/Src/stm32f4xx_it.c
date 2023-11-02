/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "packet.h"
#include "imu_data_decode.h"
#include "control.h"
#include "visual communication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern UART_HandleTypeDef huart4;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim3,htim8;
extern Visual_Data_ MID_1,A_1,B_1,C_1,D_1,MY_1;
extern Visual_Data_ A_,B_,C_,D_,MY_;
extern uint16_t yaw0,pitch0;
extern float follow_yaw,follow_pitch;
extern uint8_t visual_data[12];
extern uint8_t visual_data2[14];
uint8_t ID;
int16_t Acc[3];
int16_t Gyo[3];
int16_t Mag[3];
float Eular[3];
float Quat[4];
uint8_t KEY0=0,KEY1=0,KEY2=0,KEY3=0,KEY3_2=0,KEY3_Stop=0,KEY_RED=0,KEY_REDsign=0;
uint8_t Rxbuf_v[4];
uint8_t Txbuf_v[1];
uint16_t nut=0;
uint8_t V_SUGN=0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8)!=RESET)
	{
		
		KEY0=1;
		KEY1=KEY2=KEY3=KEY3_2=KEY_RED=0;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		
	}
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7)!=RESET)
	{
		KEY1=1;
		KEY0=KEY2=KEY3=KEY3_2=KEY_RED=0;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		
	}
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6)!=RESET)
	{
		KEY_RED=1;
		KEY3_Stop=1;
		KEY0=0,KEY1=0,KEY2=0,KEY3=0,KEY3_2=0;

	}
	
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(KEY_0_Pin);
  HAL_GPIO_EXTI_IRQHandler(KEY_1_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
//	get_raw_acc(Acc);
//	get_raw_gyo(Gyo);
//	get_raw_mag(Mag);
//	get_eular(Eular);
//	get_quat(Quat);
//	get_id(&ID);
//	gyro_control();
//	HAL_UART_Transmit(&huart4,Txbuf_v,1,0xff);
	//TT();
//	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6)==1)
//	{
//		nut++;
//		if(nut>=5000)
//		{
//			nut=5000;
//			KEY_REDsign=1;
//		}
//	}
	if(KEY0==1)
	{
		if(KEY3_Stop==0)
		{
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, yaw0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch0);
			Txbuf_v[0]=0;
		}
		else if(KEY3_Stop==1)
		{
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
			Txbuf_v[0]=1;
		}
	}
	else if(KEY1==1)
	{
		Back_Origin();
	}
	else if(KEY2==1)
	{
		Key_2();
		
	}
	else if(KEY3==1)
	{
			Black_Follow();
		//Black_Follow2();
	}
	else if(KEY3_2==1)
	{
		Black_Follow3();
	}
	else if(KEY_RED==1)
	{
		Black_Follow3();
	}
	else 
	{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
//		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, yaw0);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch0);
	}
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	//Find_Point();
  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10)!=RESET)
	{
		
		KEY2=1;
		KEY0=KEY1=KEY3=KEY3_2=KEY_RED=0;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		
	}
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11)!=RESET)
	{
		KEY3_Stop=1;
		Key_3();
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		
	}
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_2_Pin);
  HAL_GPIO_EXTI_IRQHandler(KEY_3_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
	//HAL_UART_Receive(&huart4,Rxbuf_v,4,0xffff);
	visual_data_receive();
	V_SUGN++;
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
