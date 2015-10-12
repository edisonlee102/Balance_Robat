#include "stm32f10x.h"
#include "led.h"
#include <stdio.h>
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "iic_analog.h" 
#include "MPU6050.h"
#include "stm32f10x_gpio.h"
#include "Robat.h"
#include <math.h>
#include "misc.h"
#include "stm32f10x_tim.h"
#include <oled.h>
#include "ofme_pid.h"
#include <string.h>
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read:
float acc_x,acc_y,acc_z,gy_x,gy_y,gy_z;
float roll, pitch;
float gyroXangle, gyroYangle; // Angle calculate using the gyro only 
float kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
struct Kalman kalmanX, kalmanY; // Create the Kalman instances
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double Cal_acc_x,Cal_acc_y,Cal_acc_z;
float X_Angular_velocity = 0;
float Y_Angular_velocity = 0;
float Duty; //duty cycle percentage
int Direction;    //1:CW 0:CCW
int iic_status;
pid_s sPID;	
#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.01745329251994329576923690768489
#define ACCEL_SENSITIVITY 16384  //32768/2 = 16384 LSB per g
#define GYRO_SENSITIVITY  16.384 //32768/2000 = 16.384 LSB per degree
#define CAL_COUNT         100    //sensor calibration count
#define PI (3.14159265)
//Edison fuzzy parameters
int FLC[] =
{
    2, 1, // Fuzzy model: 2 inputs, 1 output
    1, // Min Intersection
    2, // Additive Aggregation
    2 // Weighted average Defuzzification
};
int DataBase[] =
{
		7,
    // First input have 7 memberfunctions.
    -90, -90, -44, -25, 90,  4,  // NB -0
    -44, -25, -25, -15,  4,  9,  // NM -1
    -25, -8 , -8 ,   0,  5, 11,  // NS -2
    -8 , -0 ,  0 ,  8 , 11, 11,  // ZO -3
    0  ,  8 ,  8 ,  16, 11, 11,  // PS -4
    8  , 16 ,  16,  30, 11,  6,  // PM -5
    16 , 30 ,  90,  90,  6, 90,  // PB -6
		7,
    // Second input have 7 memberfunctions.
    -90, -90, -30, -20, 90, 9,  // NB -7
    -30, -20, -20, -10, 9 , 9,  // NM -8
    -20, -10, -10, 0  , 9 , 9,  // NS -9
    -10,  0 ,  0 , 10 , 9 , 9,  // Z0 -10
     0 ,  10,  10, 20 , 9 , 9,  // PS -11
     10,  20,  20, 30 , 9 , 9,  // PM -12
     20 , 30,  90, 90 , 9 , 90, // PB -13

		15,
    // Output have 11 singleton memberfunctions. //+back;-forward 
    2, 		// o/p_1 -14
    2 , 		// o/p_5 -15
    2 , 		// o/p_7 -16
    2 , 		// o/p_8 -17
		2 , 		// o/p_10 -18
    2 , 		// o/p_11 -19
    0 , 		// o/p_12 -20
    -2 , 		// o/p_13 -21
    -2 , 		// o/p_14 -22
    -2 , 		// o/p_15 -23
    -2 , 		// o/p_16 -24
	  -2 , 		// o/p_16 -25
		-2, 		// o/p_16 -26
		 1 , 		// o/p_16 -27
		-1, 		// o/p_16 -28
};// End DataBase
char RuleBase[] =
{
    /*w and roll direction opposite.*/
    49, // Rule Base contains 11 rules.
    0 , 7 ,255, 17,  255, // Rule 0: If x is NB and y is NB then z is o/p_3
    1 , 7 ,255, 18,  255, // Rule 1: If x is NB and y is NM then z is o/p_3
    2 , 7 ,255, 27,  255, // Rule 2: If x is NB and y is NS then z is o/p_3 <xxxx>
    3 , 7 ,255, 27,  255, // Rule 3: If x is NB and y is Z0 then z is o/p_1 <xxxx>
    4 , 7 ,255, 27,  255, // Rule 4: If x is NB and y is PS then z is o/p_1 <!!!!erase inertia>
    5 , 7 ,255, 28,  255, // Rule 5: If x is NB and y is PM then z is o/p_1 <!!!!erase inertia>
    6 , 7 ,255, 22,  255, // Rule 6: If x is NB and y is PB then z is o/p_1 <!!!!not erase inertia>
	
    0 , 8 ,255, 17,  255, // Rule 7: If x is NM and y is NB then z is o/p_4
    1 , 8 ,255, 18,  255, // Rule 8: If x is NM and y is NM then z is o/p_4
    2 , 8 ,255, 27,  255, // Rule 9: If x is NM and y is NS then z is o/p_3
    3 , 8 ,255, 27,  255, // Rule 9: If x is NM and y is NS then z is o/p_3
		4 , 8 ,255, 28,  255, // Rule 10: If x is NM and y is Z0 then z is o/p_3<!!!!erase inertia>
		5 , 8 ,255, 28,  255, // Rule 5: If x is NB and y is PM then z is o/p_1 <!!!!not erase inertia>
    6 , 8 ,255, 21,  255, // Rule 6: If x is NB and y is PB then z is o/p_1 <!!!!not erase inertia>
	
		0 , 9 ,255, 18,  255, // Rule 7: If x is NM and y is NB then z is o/p_4
    1 , 9 ,255, 19,  255, // Rule 8: If x is NM and y is NM then z is o/p_4
    2 , 9 ,255, 27,  255, // Rule 9: If x is NM and y is NS then z is o/p_3
    3 , 9 ,255, 27,  255, // Rule 9: If x is NM and y is NS then z is o/p_3
		4 , 9 ,255, 28,  255, // Rule 10: If x is NM and y is Z0 then z is o/p_3<!!!!erase inertia>
		5 , 9 ,255, 22,  255, // Rule 5: If x is NB and y is PM then z is o/p_1 <!!!!not erase inertia>
    6 , 9 ,255, 22,  255, // Rule 6: If x is NB and y is PB then z is o/p_1 <!!!!not erase inertia>

		0 , 10 ,255, 18,  255, // Rule 7: If x is NM and y is NB then z is o/p_4
    1 , 10 ,255, 19,  255, // Rule 8: If x is NM and y is NM then z is o/p_4
    2 , 10 ,255, 27,  255, // Rule 9: If x is NM and y is NS then z is o/p_3
    3 , 10 ,255, 20,  255, // Rule 9: If x is NM and y is NS then z is o/p_3
		4 , 10 ,255, 28,  255, // Rule 10: If x is NM and y is Z0 then z is o/p_3
		5 , 10 ,255, 21,  255, // Rule 5: If x is NB and y is PM then z is o/p_1 
    6 , 10 ,255, 22,  255, // Rule 6: If x is NB and y is PB then z is o/p_1 
		
		0 , 11 ,255, 18,  255, // Rule 7: If x is NM and y is NB then z is o/p_4 <!!!!not erase inertia>
    1 , 11 ,255, 19,  255, // Rule 8: If x is NM and y is NM then z is o/p_4
    2 , 11 ,255, 20,  255, // Rule 9: If x is NM and y is NS then z is o/p_3
    3 , 11 ,255, 28,  255, // Rule 9: If x is NM and y is NS then z is o/p_3 <!!!!erase inertia>
		4 , 11 ,255, 28,  255, // Rule 10: If x is NM and y is Z0 then z is o/p_3
		5 , 11 ,255, 21,  255, // Rule 5: If x is NB and y is PM then z is o/p_1 
    6 , 11 ,255, 23,  255, // Rule 6: If x is NB and y is PB then z is o/p_1 

		0 , 12 ,255, 19,  255, // Rule 7: If x is NM and y is NB then z is o/p_4 <!!!!not erase inertia>
    1 , 12 ,255, 27,  255, // Rule 8: If x is NM and y is NM then z is o/p_4 <!!!!not erase inertia>
    2 , 12 ,255, 28,  255, // Rule 9: If x is NM and y is NS then z is o/p_3 <!!!!erase inertia>
    3 , 12 ,255, 28,  255, // Rule 9: If x is NM and y is NS then z is o/p_3 <!!!!erase inertia>
		4 , 12 ,255, 21,  255, // Rule 10: If x is NM and y is Z0 then z is o/p_3
		5 , 12 ,255, 22,  255, // Rule 5: If x is NB and y is PM then z is o/p_1 
    6 , 12 ,255, 23,  255, // Rule 6: If x is NB and y is PB then z is o/p_1
		
		0 , 13 ,255, 19,  255, // Rule 7: If x is NM and y is NB then z is o/p_4 
    1 , 13 ,255, 27,  255, // Rule 8: If x is NM and y is NM then z is o/p_4 <!!!!not erase inertia>
    2 , 13 ,255, 28,  255, // Rule 9: If x is NM and y is NS then z is o/p_3 <!!!!erase inertia>
    3 , 13 ,255, 28,  255, // Rule 9: If x is NM and y is NS then z is o/p_3 <!!!!erase inertia>
		4 , 13 ,255, 21,  255, // Rule 10: If x is NM and y is Z0 then z is o/p_3
		5 , 13 ,255, 22,  255, // Rule 5: If x is NB and y is PM then z is o/p_1 
    6 , 13 ,255, 23,  255, // Rule 6: If x is NB and y is PB then z is o/p_1
}; // End RuleBase

#define MaxMbfs  255
#define MaxInputs  8
#define MaxOutputs 4
#define MaxTruth  90
#define Delimiter 255 /* Separator between IF-side, THEN-side, Rule. */
int NumOfInputs, NumOfOutputs;
int MbfDegree[MaxMbfs];
int CrispInput[MaxInputs];
int CrispOutput[MaxOutputs];
int dIndex, mIndex;
int j;
int NumOfRules;// MaxOutputsimum of 255 rules.
//*****************************************
// Function : SFIE()			  							*
// Initialize SFIE work area by reading   *
// fuzzy model parameters from DataBase	  *
//					  														*
//and RuleBase structures.		  					*
// Initialize I/O and mbfs. arrays zeros  *
//*****************************************
void SFIE()
{
    // Find out from FLC array for inputs, outputs, and rules there will be.
    NumOfInputs = FLC[0];
    NumOfOutputs = FLC[1];
		NumOfRules = RuleBase[0];
    // Zero out all input, output, and membership function array locations.
    for (j = 0; j < MaxMbfs; j++) MbfDegree[j] = 0;
    for (j = 0; j < MaxInputs; j++) CrispInput[j] = 0;
    for (j = 0; j < MaxOutputs; j++) CrispOutput[j] = 0;
    return;
}

//*****************************************/
// Function : Fuzzify()			  						*/
// Determine degree of membership of new  */
// crisp inputs in all input mbfs.	  		*/
//*****************************************/
void Fuzzify()
{
    int NumOfMbfs;
    int k,tmp1,tmp2;
    dIndex = 0; /* Index to step through the DataBase. */
    mIndex = 0; /* Index to point into the MbfDegree array. */
    /* Looping to Calculate inputs variables */
    for (j = 0; j < NumOfInputs; j++)
    {
			NumOfMbfs = DataBase[dIndex];
			for (k = 0; k < NumOfMbfs; k++)
			{
					if ((CrispInput[j] < DataBase[dIndex+1])||(CrispInput[j]>DataBase[dIndex+4])){
							MbfDegree[mIndex++] = 0;
					}
					else if ((CrispInput[j]<=DataBase[dIndex+3])&&(CrispInput[j]>= DataBase[dIndex+2])){
							MbfDegree[mIndex++] = MaxTruth;
					}
					else if (CrispInput[j]<DataBase[dIndex+2] ){
							MbfDegree[mIndex++]=((CrispInput[j]-DataBase[dIndex+1])*DataBase[dIndex+5]);
					}
					else{
							MbfDegree[mIndex++]=((DataBase[dIndex+4]-CrispInput[j])*DataBase[dIndex+6]);
					}
					dIndex += 6; /* Point to next Mbfs */
			} /* End for k */
			dIndex++;
		} /* End for j */
		tmp1 = dIndex; /* Save current index for use later. */
		tmp2 = mIndex; /* Save current index for use later. */
		/* Next clear output mbf degree. */
		for (j = 0; j < NumOfOutputs; j++)
		{
			NumOfMbfs = DataBase[dIndex];
			for (k = 0; k < NumOfMbfs; k++)
			{
					MbfDegree[mIndex++] = 0; /* Clear degree here. */
					dIndex++;
			} /* End for k */
		} /* End for j */
		dIndex = tmp1; /* Restore index. */
		mIndex = tmp2; /* Restore index. */
		return;
}
//****************************************/
/* Function : Evaluate()		 						 */
/* Use fuzzified inputs to calculate the */
/* strength of each rule in rule base.   */
//****************************************/
void Evaluate()
{
    int Strength;
    int index;
    int rIndex;
    rIndex = 1;
    for (j = 0; j < NumOfRules; j++)
    {
			Strength = MaxTruth; /* Each rule starts at MaxTruth. */
			/* Process IF-side of rule. */
			while ((index = RuleBase[rIndex++]) != Delimiter) /* IF-side.*/
			{
					if ( FLC[2]==1) /* Min Intersection: */
							Strength=((Strength > MbfDegree[index]) ? (MbfDegree[index]) : Strength);
					else /* Product Intersection: */
							Strength = Strength * MbfDegree[index] / MaxTruth;
			} /* End while */
			/* Update THEN-side mbfs with Strength. */
			while((index = RuleBase[rIndex++]) != Delimiter)
			/* THEN-side */
			{
					if ( FLC[3] == 1 ){ /* MAX Aggregation: */
							if (MbfDegree[index] < Strength)
								MbfDegree[index] = Strength;
					}
					else { /* Additive Aggregation: */
							MbfDegree[index] += Strength;
					}
			} /* End while */
		} /* End for j */
		return;
}

//****************************************/
/* Function : Defuzzify()		 */
/* Calculate 16 bit crisp outputs.	 */
//****************************************/
void Defuzzify()
{
    int k,NumOfMbfs;
    int temp1,temp2,degree;
    for (j = 0; j < NumOfOutputs; j++)
    {
			temp1 = 0; temp2 = 0;
			NumOfMbfs = DataBase[dIndex++];
			if ( FLC[4] == 1 ) {
			/* Weighted average defuzzification. */
					for (k = 0; k < NumOfMbfs; k++)
					{
						degree = MbfDegree[mIndex++];
						temp1 += degree;
						temp2 += (DataBase[dIndex++])*degree;
					}
					/* End for k */
					CrispOutput[j] = (temp2/temp1); 
			} 
			else
			/* MAX defuzzification. */
					for (k = 0; k < NumOfMbfs; k++)
					{
						//printf("MbfDegree[%d] = %d\t",mIndex,MbfDegree[mIndex]);
						temp1 = MbfDegree[mIndex++];
						if (temp1 > temp2)
						{
								temp2 = temp1;
								CrispOutput[j] = DataBase[dIndex++];
						}
						else
							dIndex++;
					}
		} /* End for j */
		return;
}

void GetCrispInputs(){
		if(compAngleY >= 80)
				CrispInput[0] = 80;
		else
				CrispInput[0] = (int)compAngleY;
		CrispInput[1] = (int)Y_Angular_velocity;
		//printf("CrispInput[0] = %d\tCrispInput[1] = %d\n",CrispInput[0],CrispInput[1]);
}

#define ABS(X) (((X)>(0))?(X):(-1*X))
void ApplyCrispOutputs(){
		//static int ignore = TIME_PERIOD,pre_dir;
		int temp = CrispOutput[0];
		
		if(CrispOutput[0] > 0){
			//Please Forward
				Direction = 1;
				//printf("Forward %d duty\n",CrispOutput[0]);
		}else if(CrispOutput[0] < 0)
		{
				//Please Backward
				Direction = 0;
				//printf("Backward %d duty\n",CrispOutput[0]);
		}else{
				//Balance
					//printf("Balance\n");
		}
		/*if(pre_dir == Direction){
			ignore--;
			if(ignore == 0){
					ignore = TIME_PERIOD;
					count = 0;
			}
		}else{
			ignore = TIME_PERIOD;
			count++;
		}
		pre_dir = Direction;*/
		Duty = ABS(CrispOutput[0]);
		//printf("Direction = %d , Duty = %d \n",(int)Direction,(int)Duty);
}

void updatePitchRoll() {
    #ifdef RESTRICT_PITCH // Eq. 25 and 26
    roll = atan2(acc_y,acc_z) * RAD_TO_DEG;
    pitch = atan(-acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * RAD_TO_DEG;
    #else // Eq. 28 and 29
    roll = atan(acc_y / sqrt(acc_x * acc_x + acc_z * acc_z)) * RAD_TO_DEG;
    pitch = atan2(-acc_x, acc_z) * RAD_TO_DEG;
    #endif
}

void USART_Configuration(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	
		/* Init USART3 Interrupt */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_ClearPendingIRQ(USART3_IRQn);
		NVIC_Init(&NVIC_InitStructure);	
		NVIC_EnableIRQ(USART3_IRQn);
	
		//GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
	
		/* Configure USART3 Tx (PB10) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		/* Configure USART3 Rx (PB11) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
    USART_Init(USART3,&USART_InitStructure);
		/* Enable USART3 Receive and Transmit interrupts */
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		//USART_ITConfig(USART3, USART_IT_TC, ENABLE);
		
		USART_Cmd(USART3,ENABLE);
		//---------------USART 1----------------//
		/* Init USART1 Interrupt */
		/*NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_ClearPendingIRQ(USART1_IRQn);
		NVIC_Init(&NVIC_InitStructure);	
		NVIC_EnableIRQ(USART1_IRQn);
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);    
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);  
			
		
		USART_InitStructure.USART_BaudRate = 9600;	
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;	
		USART_InitStructure.USART_StopBits = USART_StopBits_1; 	
		USART_InitStructure.USART_Parity = USART_Parity_No ; 
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART1, &USART_InitStructure);  
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		USART_Cmd(USART1, ENABLE);*/
}
void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS)
    {
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK2Config(RCC_HCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div2);
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        RCC_PLLCmd(ENABLE);
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while(RCC_GetSYSCLKSource() != 0x08);
    }
		/* TIM2 clock enable */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		/* TIM2 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
		/* TIM3 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		/* TIM4 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
		
		/* UART3 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
}
#define GPIO_Remap_SWJ_JTAGDisable  ((uint32_t)0x00300200) 
void TIM_GPIO_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
		/* GPIOB Configuration:Motor Input */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		/* GPIOB Configuration:Motor Input & OLED RST,RS*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		/* GPIOA Configuration:Motor Input & OLED SD,SC*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		/* GPIOA Configuration:TIM2 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		/* GPIOB Configuration:TIM4*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		/*GPIO_ResetBits(GPIOB, GPIO_Pin_4);
		GPIO_ResetBits(GPIOB,GPIO_Pin_5);
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
		GPIO_ResetBits(GPIOA,GPIO_Pin_15);*/
}

void NVIC_Timer2_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    #ifdef  VECT_TAB_RAM
        NVIC_SetVectorTable(NVIC_VectTab_RAM,0x0);
    #else
        NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x0);
    #endif
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void NVIC_Timer4_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    #ifdef  VECT_TAB_RAM
        NVIC_SetVectorTable(NVIC_VectTab_RAM,0x0);
    #else
        NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x0);
    #endif
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM2_Configuration(void)
{
	//EX: T=(TIM_Period+1)*(TIM_Prescaler+1)/TIMxCLK=(99+1)*(7199+1)/72MHz=0.01s//
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_TimeBaseStructure.TIM_Period = 99;
		TIM_TimeBaseStructure.TIM_Prescaler = 7199;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
		TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);
		TIM_Cmd(TIM2,ENABLE);
}

void TIM4_Configuration(void)
{
	//EX: T=(TIM_Period+1)*(TIM_Prescaler+1)/TIMxCLK=(1000)*(7200)/72MHz=0.01s//
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_TimeBaseStructure.TIM_Period = 9;
		TIM_TimeBaseStructure.TIM_Prescaler = (int)(719/2);
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
		TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);
		TIM_Cmd(TIM4,ENABLE);
}

void updateMPU6050(void)
{
	static unsigned int err_cnt=0;
	err_cnt = err_cnt*115>>7;
	if(err_cnt>6570)
		iic_status = 0;
	else
		iic_status = 1;
	acc_x= getAccX();
	if(acc_x == 0)
		goto err;
	acc_x -= Cal_acc_x; 
	acc_y= getAccY();
	if(acc_y == 0)
		goto err;
	acc_y -= Cal_acc_y; 
	acc_z= getAccZ();
	if(acc_z == 0)
		goto err;	
	gy_x= getGyroX();
	if(gy_x == 0)
		goto err;
	gy_y= getGyroY();
	if(gy_y == 0)
		goto err;
	gy_z= getGyroZ();
	if(gy_z == 0)
		goto err;
	//printf("acc_x = %d,acc_y = %d,acc_z = %d,gy_x = %d,gy_y = %d,gy_z = %d\n",(int)acc_x,(int)acc_y,(int)acc_z,(int)gy_x,(int)gy_y,(int)gy_z);
	return;
err:
	err_cnt +=100;	
	return;
}

void sensor_cal(void)
{
	int i;
	float acc_x_cal,acc_y_cal;
	for(i = 0;i<CAL_COUNT;i++)
	{
			acc_x_cal += getAccX();
			acc_y_cal += getAccY();
			//acc_z_cal += getAccZ();
	}
	Cal_acc_x = acc_x_cal/CAL_COUNT;
	Cal_acc_y = acc_y_cal/CAL_COUNT;
	//Cal_acc_z = acc_z_cal/CAL_COUNT;
}

void InitAll()
{
		/* Calibration G+g */
		sensor_cal();
    /* Set Kalman and gyro starting angle */
    updateMPU6050();
    updatePitchRoll(); 
    
    setAngle(&kalmanX,roll); // First set roll starting angle
    gyroXangle = roll;
    compAngleX = roll;
    setAngle(&kalmanY,pitch); // Then pitch
    gyroYangle = pitch;
    compAngleY = pitch;
	
		/* Fuzzy array init */
		SFIE();// initialize FLC
		CrispInput[0] = (int)compAngleX;
		CrispInput[1] = (int)X_Angular_velocity;
}
#define SIGN(a) (((a)>=0)?(1):(-1))
void Sensor_fusion()
{
    float gyroXrate,gyroYrate,dt=0.01;
    /* Update all the IMU values */
    updateMPU6050();
    
    /* Roll and pitch estimation */
    updatePitchRoll();             //calculation Pitch and Roll
    gyroXrate = gy_x / GYRO_SENSITIVITY;     // Convert to deg/s
    gyroYrate = gy_y / GYRO_SENSITIVITY;     // Convert to deg/s
    
		//For fuzzy input[1]
		X_Angular_velocity = gyroXrate*0.1;
		Y_Angular_velocity = gyroYrate*0.1;
    #ifdef RESTRICT_PITCH        //#define RESTRICT_PITCH prevent -180 ~ 180 jump
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        setAngle(&kalmanX,roll);
				compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    } else
    kalAngleX = getAngle(&kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    
    if (fabs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
    kalAngleY = getAngle(&kalmanY,pitch, gyroYrate, dt);
    #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
    } else
    kalAngleY = getAngle(&kalmanY, pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
    
    if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
    kalAngleX = getAngle(&kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    #endif
    
    /* Estimate angles using gyro only */
    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate from the Kalman filter
    //gyroYangle += kalmanY.getRate() * dt;
    //gyroZangle += kalmanZ.getRate() * dt;
		/* Estimate angles using complimentary filter */
    compAngleX = 0.98 * (compAngleX + gyroXrate * dt) + 0.02 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.98 * (compAngleY + gyroYrate * dt) + 0.02 * pitch;
    // Reset the gyro angles when they has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;
		//printf("kalAngleX = %d , kalAngleY = %d\n",(int)kalAngleX,(int)kalAngleY);
		if(ABS((int)compAngleY)>80)
			compAngleY = SIGN(compAngleY)*80;
		printf("compAngleX = %d , compAngleY = %d , Y_Angular_velocity = %d\n",(int)compAngleX,(int)compAngleY,(int)Y_Angular_velocity);
} 

void TIM4_GPIO_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		/* GPIOB Configuration: TIM4 channel1,2 */
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
}
#define SYSCLK_FREQ_72MHz  72000000
static void TIM4_Mode_Config(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_OCInitTypeDef TIM_OCInitStructure;
		uint16_t CCR_Val = 0;
		uint16_t PrescalerValue = 0;
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) (SYSCLK_FREQ_72MHz / 24000000) - 1;
		
		/* The 4 is running at 36 KHz: TIM4 Frequency = TIM4 counter clock/(ARR + 1)
																										 = 24 MHz / 666 ~= 36 KHz ,
		TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50% */
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = 666;
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		
		TIM_OCInitStructure.TIM_Pulse = CCR_Val;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC1Init(TIM4, &TIM_OCInitStructure);
		/* PWM1 Mode configuration: Channel2 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = CCR_Val;
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		
		
		TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
		TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		
		TIM_ARRPreloadConfig(TIM4, ENABLE);
		/* TIM4 enable counter */
		TIM_Cmd(TIM4, ENABLE);
}

void TIM4_PWM_Init(void)
{
		TIM4_GPIO_Config();
		TIM4_Mode_Config();
}

void TIM1_Light_Mask_Configuration(void)
{
		//Just need timer to calculate
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_TimeBaseStructure.TIM_Period = 0xffff;
		TIM_TimeBaseStructure.TIM_Prescaler = 719;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
		TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE);
		TIM_Cmd(TIM1,ENABLE);
}

void Light_mask_init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
		/* EXTI line gpio config(PA.1) */ 
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		//
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);                                                                                                          
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;       
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
		EXTI_InitStructure.EXTI_Line = EXTI_Line1;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		//TIM 1 init PA.0 & 6
		TIM1_Light_Mask_Configuration();
}
extern int counter_number_ex;
int main(void)
{ 
		char str[10];
		SystemInit();
		Systick_Init(); //Sys_tick for delay function
		RCC_Configuration();
		TIM_GPIO_Config();	
		USART_Configuration();
		//MPU6050 init
		Sys_Configuration();
		MPU6050_Inital();
		InitAll();
		pid_init(&sPID, 7.5,0,-15);
		sPID.target = 0;
		//Timer 2 - 100HZ for sensor fusion (PA.3)
		NVIC_Timer2_Configuration();
		TIM2_Configuration();
		//Timer 4 - 10KHZ for Motor (PB.8)
		NVIC_Timer4_Configuration();
		TIM4_Configuration();
		//Make Timer 4 - PWM PB.6,7
		TIM4_PWM_Init();
		//Make IQR for mask caculate
		//Light_mask_init();
		OLED_Init();
		while (1){
			//OLED Debug
		OLED_Display_On();
		OLED_Clear();
		memset(str,'\0',10*sizeof(char));
		sprintf(str, "%s%d","Angle:" , (int)compAngleY);
		OLED_ShowString(0,0,(uint8_t *)str);
		memset(str,'\0',10*sizeof(char));
		sprintf(str, "%s%d","Dir:" , (int)Direction);
		OLED_ShowString(0,12,(uint8_t *)str);
		memset(str,'\0',10*sizeof(char));
		sprintf(str, "%s%d","PWM:" , (int)counter_number_ex);
		OLED_ShowString(0,24,(uint8_t *)str);
		OLED_Refresh_Gram();	
		//End Debug print
				//USART_SendData(USART1,'1');
		}
}
