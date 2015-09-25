#ifndef _iic_analog_h_
#define _iic_analog_h_

#include "stm32f10x.h"

#define IIC_GPIO (GPIOA)
#define IIC_GOIO_SDA (GPIOA)
#define IIC_GPIO_SCL (GPIOA)
#define IIC_SDA (GPIO_Pin_11)
#define IIC_SCL (GPIO_Pin_12)

extern void IIC_GPIO_Configuration( GPIO_TypeDef * GPIOx_SDA , uint16_t SDA_Pin , GPIO_TypeDef * GPIOx_SCL , uint16_t SCL_Pin );

#define SET_SDA		{ GPIO_SetBits( IIC_GPIO , IIC_SDA ); }
#define RESET_SDA	{ GPIO_ResetBits( IIC_GPIO , IIC_SDA );}
#define SET_SCL		{ GPIO_SetBits( IIC_GPIO , IIC_SCL ); }
#define RESET_SCL 	{ GPIO_ResetBits( IIC_GPIO , IIC_SCL); }
#define IIC_SDA_STATE (IIC_GPIO->IDR&IIC_SDA)
#define IIC_SCL_STATE (IIC_GPIO->IDR&IIC_SDA)

#define IIC_DELAY { IIC_Delay(); }

enum IIC_REPLAY_ENUM
{
	IIC_NACK = 0,
	IIC_ACK = 1
};

enum IIC_BUS_STATE_ENUM
{
	IIC_BUS_READY = 0,
	IIC_BUS_BUSY=1,
	IIC_BUS_ERROR=2
};


extern void IIC_Delay(void);
extern u8 IIC_Start(void);
extern void IIC_Stop(void);
extern void IIC_SendACK(void);
extern void IIC_SendNACK(void);
extern u8 IIC_SendByte(u8 Data);
extern u8 IIC_RecvByte(void);
extern void Single_Write_IIC(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
extern u8 Single_Read_IIC(u8 SlaveAddress, u8 REG_Address);

extern uint16_t GPIO_Filter( GPIO_TypeDef * GPIOx );

#endif

