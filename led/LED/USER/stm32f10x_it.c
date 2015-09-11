/**
  ******************************************************************************
  * @file    Project/Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.0.0
  * @date    04/06/2009
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include <stdio.h>
extern int TimingDelay;
extern float Duty;
extern int Direction;
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void TimingDelay_Decrement(void)
{
		if (TimingDelay != 0x00)
		{
			TimingDelay--;
		}
}
void SysTick_Handler(void)
{
		TimingDelay_Decrement();
}
void Motor_duty_control()
{
		TIM_OCInitTypeDef TIM_OCInitStructure;
		if(Duty == 1)
		{
				/* PWM1 Mode configuration: Channel3 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = 66;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC3Init(TIM3, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel4 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = 66;
				TIM_OC4Init(TIM3, &TIM_OCInitStructure);
				
				
				TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
				TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM3, ENABLE);
				/* TIM3 enable counter */
				TIM_Cmd(TIM3, ENABLE);
		}else if(Duty == 2)
		{
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = 133;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC3Init(TIM3, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel4 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = 133;
				TIM_OC4Init(TIM3, &TIM_OCInitStructure);
				
				
				TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
				TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM3, ENABLE);
				/* TIM3 enable counter */
				TIM_Cmd(TIM3, ENABLE);
		}
		else if(Duty == 3)
		{
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = 200;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC3Init(TIM3, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel4 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = 200;
				TIM_OC4Init(TIM3, &TIM_OCInitStructure);
				
				
				TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
				TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM3, ENABLE);
				/* TIM3 enable counter */
				TIM_Cmd(TIM3, ENABLE);
		}
		else if(Duty == 4)
		{
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = 266;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC3Init(TIM3, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel4 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = 266;
				TIM_OC4Init(TIM3, &TIM_OCInitStructure);
				
				
				TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
				TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM3, ENABLE);
				/* TIM3 enable counter */
				TIM_Cmd(TIM3, ENABLE);
		}
		else if(Duty == 5)
		{
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = 333;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC3Init(TIM3, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel4 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = 333;
				TIM_OC4Init(TIM3, &TIM_OCInitStructure);
				
				
				TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
				TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM3, ENABLE);
				/* TIM3 enable counter */
				TIM_Cmd(TIM3, ENABLE);
		}
		else if(Duty == 6)
		{
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = 400;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC3Init(TIM3, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel4 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = 400;
				TIM_OC4Init(TIM3, &TIM_OCInitStructure);
				
				
				TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
				TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM3, ENABLE);
				/* TIM3 enable counter */
				TIM_Cmd(TIM3, ENABLE);
		}
		else if(Duty == 7)
		{
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = 466;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC3Init(TIM3, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel4 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = 466;
				TIM_OC4Init(TIM3, &TIM_OCInitStructure);
				
				
				TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
				TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM3, ENABLE);
				/* TIM3 enable counter */
				TIM_Cmd(TIM3, ENABLE);
		}
		else if(Duty == 8)
		{
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = 533;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC3Init(TIM3, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel4 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = 533;
				TIM_OC4Init(TIM3, &TIM_OCInitStructure);
				
				
				TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
				TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM3, ENABLE);
				/* TIM3 enable counter */
				TIM_Cmd(TIM3, ENABLE);
		}
		else if(Duty == 9)
		{
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = 599;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC3Init(TIM3, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel4 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = 599;
				TIM_OC4Init(TIM3, &TIM_OCInitStructure);
				
				
				TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
				TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM3, ENABLE);
				/* TIM3 enable counter */
				TIM_Cmd(TIM3, ENABLE);
		}
		else if(Duty == 10)
		{
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = 666;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC3Init(TIM3, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel4 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = 666;
				TIM_OC4Init(TIM3, &TIM_OCInitStructure);
				
				
				TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
				TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM3, ENABLE);
				/* TIM3 enable counter */
				TIM_Cmd(TIM3, ENABLE);
		}else if(Duty == 0)
		{
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = 0;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC3Init(TIM3, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel4 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = 0;
				TIM_OC4Init(TIM3, &TIM_OCInitStructure);
				
				
				TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
				TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM3, ENABLE);
				TIM_Cmd(TIM3, ENABLE);
		}
}
void Motor_dir_control()
{
		if(Direction == 0){
				GPIO_SetBits(GPIOB,GPIO_Pin_15);
				//GPIO_ResetBits(GPIOB,GPIO_Pin_14);
				GPIO_ResetBits(GPIOB,GPIO_Pin_10);
				GPIO_SetBits(GPIOB,GPIO_Pin_13);
				//GPIO_ResetBits(GPIOB,GPIO_Pin_12);
				GPIO_ResetBits(GPIOB,GPIO_Pin_11);
		}
		if(Direction == 1)
		{
				GPIO_ResetBits(GPIOB, GPIO_Pin_15);
				//GPIO_SetBits(GPIOB,GPIO_Pin_14);
				GPIO_SetBits(GPIOB,GPIO_Pin_10);
				GPIO_ResetBits(GPIOB,GPIO_Pin_13);
				//GPIO_SetBits(GPIOB,GPIO_Pin_12);
				GPIO_SetBits(GPIOB,GPIO_Pin_11);
		}
}

void TIM4_IRQHandler(void)
{
		vu16 capture=0;
		if(TIM_GetITStatus(TIM4,TIM_IT_CC1) != RESET)
		{
				Motor_duty_control();
				Motor_dir_control();
				capture = TIM_GetCapture1(TIM4);
				TIM_ClearITPendingBit(TIM4,TIM_IT_CC1);
		}
}

extern void Sensor_fusion(void);
extern void Fuzzify(void);
extern void GetCrispInputs(void);
extern void Evaluate(void);
extern void Defuzzify(void);
extern void ApplyCrispOutputs(void);
void TIM2_IRQHandler(void)
{
	vu16 capture=0;
	if(TIM_GetITStatus(TIM2,TIM_IT_CC1) != RESET)
	{
			Sensor_fusion();
			GetCrispInputs();
			Fuzzify(); // fuzzification
			Evaluate(); // RuleBase evalution
			Defuzzify(); // Defuzzification
			ApplyCrispOutputs();
			capture = TIM_GetCapture1(TIM2);
			TIM_ClearITPendingBit(TIM2,TIM_IT_CC1);
	}
}
#define CR1_CEN_Set                 ((uint16_t)0x0001)
#define CR1_CEN_Reset               ((uint16_t)0x03FE)
#define TIM1_FREQ  									100000
long Cycle,temp_cnt;
int pre_duty;
void EXTI1_IRQHandler(void)
{
		int i = 0;
		vu16 capture=0;

		if(EXTI_GetITStatus(EXTI_Line1) != RESET)
		{
			capture = TIM_GetCounter(TIM1);
			while(i < 100000)
					i++;	
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)) { //debounce
					if(Duty == pre_duty){						
						if(((TIM1->CR1 & CR1_CEN_Set) != ENABLE)){
								TIM_Cmd(TIM1,ENABLE);
								return;
						}
						if(capture > 10000 || capture < 250){
							TIM_SetCounter(TIM1,0);
							return;
						}	
						printf("capture = %d\n",capture);
						temp_cnt += capture;
						TIM_SetCounter(TIM1,0);
						Cycle++;
						
						printf("Cycle = %ld ,PRM_inv = %d ,RPM = %d\n",Cycle,(int)((double)temp_cnt/Cycle),(int)((1/(((double)temp_cnt/Cycle)/TIM1_FREQ))/13*60));
					}else{
						Cycle = 0;
						temp_cnt = 0;
						TIM_Cmd(TIM1,DISABLE);
						TIM_SetCounter(TIM1,0);
					}
			}
		}
		pre_duty = Duty;
		EXTI_ClearITPendingBit(EXTI_Line1);
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
