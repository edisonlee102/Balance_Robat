#include "stm32f10x.h"
#include "led.h"
#include <stdio.h>
#include "stm32f10x_usart.h"
#include "mpu6050.h"
#include "stm32f10x_i2c.h"

void Delay(__IO u32 nCount); 
void USART_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStructure;
    USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
    USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
    USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;                                                                                                                                                     
    USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
    USART_ClockInit(USART1 , &USART_ClockInitStructure);
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
    USART_Init(USART1,&USART_InitStructure);
    USART_Cmd(USART1,ENABLE);
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
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1, ENABLE);
}
/*void I2C_init(void){
 
    I2C_InitTypeDef I2C_InitStructure; 
    GPIO_InitTypeDef GPIO_InitStructure;
 
    I2C_Cmd(I2C1,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);    
	
    I2C_InitStructure.I2C_Mode = I2C_Mode_SMBusHost;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00; // STM32 ??? I2C ??
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000 ; // ?? I2C ????? 100K 
    I2C_Init(I2C1, &I2C_InitStructure);
 
}*/

int main(void)
{
 uint8_t *data;
 SystemInit();	
 //I2C_init();
 MPU6050_I2C_Init();
 RCC_Configuration();
 LED_GPIO_Config();
 USART_Configuration();
 MPU6050_Initialize();
	printf("123\n");
 printf("%d\n",(int)MPU6050_TestConnection());
  while (1)
  {
		printf("123\n");
		MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS,0x48,MPU6050_ACONFIG_AFS_SEL_BIT,8,data);
		printf("\nMPU0x48 = %d\n",(uint8_t)data);
		Delay(0x200000);
  }
}

void Delay(__IO u32 nCount)
{
  for(; nCount != 0; nCount--);
} 



