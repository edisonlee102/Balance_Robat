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
#include "stdlib.h"
#include "string.h"
#include "ofme_pid.h"
#include <oled.h>
extern int TimingDelay;
extern float Duty;
extern int Direction;
extern int iic_status;
unsigned char dir;
bool ble_lock;
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

volatile int count;
extern double compAngleY;
extern float Y_Angular_velocity,acc_z;
extern pid_s sPID;
#define TIME_PERIOD 100
#define ABS(X) (((X)>(0))?(X):(-1*X))
#define SIGN(X) (((X)>(0))?(1):(-1))
int counter_number_ex;
void Motor_duty_control()
{
		static float IF = 1;		
		int pwm_balance;
		TIM_OCInitTypeDef TIM_OCInitStructure;
		int counter_number,temp;
		if(count != 0)
		{
				//printf("@count = %d\n",count);
				IF = 1.0/(count+1);
		}else{
				IF = 1;
		}
		//printf("IF = %f\n",IF);
			if(ble_lock){
				pwm_balance = pid_proc(&sPID, compAngleY, Y_Angular_velocity);
				counter_number = ABS(pwm_balance);
				printf("@@@@@@@@@@@@@@@\n");
				if(dir == '1')
					counter_number+=400;
				if(dir == '5')
					counter_number+=350;
			}else{
				if(ABS(Y_Angular_velocity) < 10){ // if w not too big
					if(Duty != 0)
						counter_number = ABS(ABS(compAngleY)*Duty+40);
					else
						counter_number = 0;
				}else{// if w big
						if(compAngleY>10 && Y_Angular_velocity>0){
								temp = compAngleY*Duty-Y_Angular_velocity*10;
								counter_number = ABS(temp);
						}
						if(compAngleY<10 && Y_Angular_velocity>0){
								temp = ABS(compAngleY)*Duty+Y_Angular_velocity*5;
								counter_number = ABS(temp);
						}
						if(compAngleY>10 && Y_Angular_velocity<0){
								temp = compAngleY*Duty+Y_Angular_velocity*5;
								counter_number = temp;
						}
						if(compAngleY<10 && Y_Angular_velocity<0){
								temp = ABS(compAngleY)*Duty-Y_Angular_velocity*5;
								counter_number = ABS(temp);
						}
				}
				printf("counter_number = %d\n",counter_number);
				//counter_number *= 3;
				if(counter_number >= 666|| counter_number < 0)
				{
					//printf("Trim %d\n",counter_number);
					counter_number = 0;
				}
		}
		if((acc_z<=0) || (iic_status == 0))
			counter_number = 0;
		counter_number_ex = counter_number;
		//printf("counter_number = %d\n",counter_number);
		//printf("counter_number = %d ,Duty = %f,compAngleY = %lf,Y_Angular_velocity = %f,ABS(compAngleY) = %f\n",counter_number,Duty,compAngleY,Y_Angular_velocity,ABS(compAngleY));
		TIM_Cmd(TIM4, DISABLE);
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		
		TIM_OCInitStructure.TIM_Pulse = counter_number*IF;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC1Init(TIM4, &TIM_OCInitStructure);
		/* PWM1 Mode configuration: Channel2 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = counter_number*IF;
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		
		
		TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
		TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		
		TIM_ARRPreloadConfig(TIM4, ENABLE);
		/* TIM4 enable counter */
		TIM_Cmd(TIM4, ENABLE);
		#if 0
		if(Duty == 1)
		{
				/* PWM1 Mode configuration: Channel1 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = (int)66*IF;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC1Init(TIM4, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel2 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = (int)66*IF;
				TIM_OC2Init(TIM4, &TIM_OCInitStructure);
				
				
				TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
				TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM4, ENABLE);
				/* TIM4 enable counter */
				TIM_Cmd(TIM4, ENABLE);
		}else if(Duty == 2)
		{
				/* PWM1 Mode configuration: Channel1 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = (int)133*IF;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC1Init(TIM4, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel2 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = (int)133*IF;
				TIM_OC2Init(TIM4, &TIM_OCInitStructure);
				
				
				TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
				TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM4, ENABLE);
				/* TIM4 enable counter */
				TIM_Cmd(TIM4, ENABLE);
		}
		else if(Duty == 3)
		{
				/* PWM1 Mode configuration: Channel1 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = (int)200*IF;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC1Init(TIM4, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel2 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = (int)200*IF;
				TIM_OC2Init(TIM4, &TIM_OCInitStructure);
				
				
				TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
				TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM4, ENABLE);
				/* TIM4 enable counter */
				TIM_Cmd(TIM4, ENABLE);
		}
		else if(Duty == 4)
		{
				/* PWM1 Mode configuration: Channel1 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = (int)266*IF;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC1Init(TIM4, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel2 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = (int)266*IF;
				TIM_OC2Init(TIM4, &TIM_OCInitStructure);
				
				
				TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
				TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM4, ENABLE);
				/* TIM4 enable counter */
				TIM_Cmd(TIM4, ENABLE);
		}
		else if(Duty == 5)
		{
				/* PWM1 Mode configuration: Channel1 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = (int)333*IF;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC1Init(TIM4, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel2 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = (int)333*IF;
				TIM_OC2Init(TIM4, &TIM_OCInitStructure);
				
				
				TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
				TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM4, ENABLE);
				/* TIM4 enable counter */
				TIM_Cmd(TIM4, ENABLE);
		}
		else if(Duty == 6)
		{
				/* PWM1 Mode configuration: Channel1 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = (int)400*IF;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC1Init(TIM4, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel2 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = (int)400*IF;
				TIM_OC2Init(TIM4, &TIM_OCInitStructure);
				
				
				TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
				TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM4, ENABLE);
				/* TIM4 enable counter */
				TIM_Cmd(TIM4, ENABLE);
		}
		else if(Duty == 7)
		{
				/* PWM1 Mode configuration: Channel1 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = (int)466*IF;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC1Init(TIM4, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel2 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = (int)466*IF;
				TIM_OC2Init(TIM4, &TIM_OCInitStructure);
				
				
				TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
				TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM4, ENABLE);
				/* TIM4 enable counter */
				TIM_Cmd(TIM4, ENABLE);
		}
		else if(Duty == 8)
		{
				/* PWM1 Mode configuration: Channel1 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = (int)533*IF;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC1Init(TIM4, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel2 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = (int)533*IF;
				TIM_OC2Init(TIM4, &TIM_OCInitStructure);
				
				
				TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
				TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM4, ENABLE);
				/* TIM4 enable counter */
				TIM_Cmd(TIM4, ENABLE);
		}
		else if(Duty == 9)
		{
				/* PWM1 Mode configuration: Channel1 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = (int)599*IF;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC1Init(TIM4, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel2 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = (int)599*IF;
				TIM_OC2Init(TIM4, &TIM_OCInitStructure);
				
				
				TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
				TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM4, ENABLE);
				/* TIM4 enable counter */
				TIM_Cmd(TIM4, ENABLE);
		}
		else if(Duty == 10)
		{
				/* PWM1 Mode configuration: Channel1 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = (int)666*IF;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC1Init(TIM4, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel2 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = (int)666*IF;
				TIM_OC2Init(TIM4, &TIM_OCInitStructure);
				
				
				TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
				TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM4, ENABLE);
				/* TIM4 enable counter */
				TIM_Cmd(TIM4, ENABLE);
		}else if(Duty == 0)
		{
				/* PWM1 Mode configuration: Channel1 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
				TIM_OCInitStructure.TIM_Pulse = 0;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				TIM_OC1Init(TIM4, &TIM_OCInitStructure);
				/* PWM1 Mode configuration: Channel2 */
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = 0;
				TIM_OC2Init(TIM4, &TIM_OCInitStructure);
				
				
				TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
				TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
				
				TIM_ARRPreloadConfig(TIM4, ENABLE);
				TIM_Cmd(TIM4, ENABLE);
		}
		#endif
}

void Motor_dir_control()
{
		static int ignore = TIME_PERIOD,pre_dir;
	
		if(pre_dir == Direction){
			ignore--;
			if(ignore == 0){
						ignore = TIME_PERIOD;
						count = 0;
			}
		}else{
			ignore = TIME_PERIOD;
			count++;
		}
		pre_dir = Direction;
		//printf("count = %d\n",count);
		if(dir != 0 && ble_lock)
		{
					switch(dir)
				{
					case '1':
									GPIO_SetBits(GPIOB,GPIO_Pin_4);
									GPIO_ResetBits(GPIOB,GPIO_Pin_5);
									GPIO_SetBits(GPIOB,GPIO_Pin_12);
									GPIO_ResetBits(GPIOA,GPIO_Pin_15);
									printf("forward\n");
									break;
					case '2':
						
									break;
					case '3':
						
									break;
					case '4':
						
									break;
					case '5':
									GPIO_ResetBits(GPIOB, GPIO_Pin_4);
									GPIO_SetBits(GPIOB,GPIO_Pin_5);
									GPIO_ResetBits(GPIOB,GPIO_Pin_12);
									GPIO_SetBits(GPIOA,GPIO_Pin_15);
									printf("backward\n");
									break;
					case '6':
						
									break;
					case '7':
						
									break;
					case '8':
						
									break;
					default:
									printf("stop\n");
									dir = 0;
									break;
				}
		}
		if(Direction == 0 && !ble_lock){
				GPIO_SetBits(GPIOB,GPIO_Pin_4);
				GPIO_ResetBits(GPIOB,GPIO_Pin_5);
				GPIO_SetBits(GPIOB,GPIO_Pin_12);
				GPIO_ResetBits(GPIOA,GPIO_Pin_15);
		}
		if(Direction == 1 && !ble_lock)
		{
				GPIO_ResetBits(GPIOB, GPIO_Pin_4);
				GPIO_SetBits(GPIOB,GPIO_Pin_5);
				GPIO_ResetBits(GPIOB,GPIO_Pin_12);
				GPIO_SetBits(GPIOA,GPIO_Pin_15);
		}
}

void TIM4_IRQHandler(void)
{
		vu16 capture=0;
		if(TIM_GetITStatus(TIM4,TIM_IT_CC1) != RESET)
		{
				Motor_dir_control();
				Motor_duty_control();
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
	/*static int ignore = TIME_PERIOD,pre_dir;
	
		if(pre_dir == Direction){
			ignore--;
			if(ignore == 0){
					ignore = TIME_PERIOD;
					count = 0;
			}
		}else{
			ignore = TIME_PERIOD;
			count++;
		}
		pre_dir = Direction;
	printf("count = %d\n",count);*/
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

void USART3_IRQHandler(){
		static unsigned char GetData;
		ble_lock = 0;
		if(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=RESET)
				USART_ClearITPendingBit(USART3,USART_FLAG_TC);
		if((USART_GetFlagStatus(USART3,USART_FLAG_RXNE)!=RESET)){
				USART_ClearITPendingBit(USART3,USART_FLAG_RXNE);
				 GetData = USART_ReceiveData(USART3);
			if(GetData){
				if(GetData == '0')
					ble_lock = FALSE ;
				else if(GetData >= '1' && GetData <='8')
					ble_lock = TRUE ;
				printf("GetData = %c\n",GetData);
				dir = GetData;
			}
		}
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
