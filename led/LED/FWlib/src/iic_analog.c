#include "stm32f10x.h"
#include "iic_analog.h"

void IIC_GPIO_Configuration( GPIO_TypeDef * GPIOx_SDA , uint16_t SDA_Pin , GPIO_TypeDef * GPIOx_SCL , uint16_t SCL_Pin )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint32_t RCC_GPIOx_SDA = 0;
	uint32_t RCC_GPIOx_SCL = 0;


	RCC_GPIOx_SDA = GPIO_Filter( GPIOx_SDA );
	RCC_GPIOx_SCL = GPIO_Filter( GPIOx_SCL );
	

	/* Enable I2C and GPIO clocks */
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_GPIOx_SDA,ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_GPIOx_SCL,ENABLE);


	GPIO_InitStructure.GPIO_Pin = SDA_Pin;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	GPIO_Init(GPIOx_SDA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SCL_Pin;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	GPIO_Init(GPIOx_SCL, &GPIO_InitStructure);


	SET_SDA;
	SET_SCL;  
}


/************************************************************/
/************************************************************/
void IIC_Delay(void)
{
	u32 i = 5;
	while( i-- );
}


u8 IIC_Start(void)
{
	SET_SDA;
	IIC_DELAY;

	SET_SCL;
	IIC_DELAY;

	if( IIC_SDA_STATE == RESET )
	{
		return IIC_BUS_BUSY;
	}

	RESET_SDA;
	IIC_DELAY;

	RESET_SCL;
	IIC_DELAY;

	if( IIC_SDA_STATE == SET )
	{
		return IIC_BUS_ERROR;
	}

	return IIC_BUS_READY;
}


void IIC_Stop(void)
{
	RESET_SDA;
	IIC_DELAY;

	SET_SCL;
	IIC_DELAY;

	SET_SDA;
	IIC_DELAY;
}


void IIC_SendNACK(void)
{
	RESET_SDA;
	IIC_DELAY;
	SET_SCL;
	IIC_DELAY;
	RESET_SCL; 
	IIC_DELAY; 
}


void IIC_SendACK(void)
{
	SET_SDA;
	IIC_DELAY;
	SET_SCL;
	IIC_DELAY;
	RESET_SCL; 
	IIC_DELAY;
}


u8 IIC_SendByte(u8 Data)
{
	 u8 i;
	 RESET_SCL;
	 for(i=0;i<8;i++)
	 {  

		if(Data&0x80)
		{
			SET_SDA;
		}
		else
		{
			RESET_SDA;
		} 
		Data<<=1;
		IIC_DELAY;

		SET_SCL;
		IIC_DELAY;
		RESET_SCL;
		IIC_DELAY;
		//---------------------------   
	 }
	
	 SET_SDA; 
	 IIC_DELAY;
	 SET_SCL;
	 IIC_DELAY;   
	 if(IIC_SDA_STATE)
	 {
		RESET_SCL;
		return IIC_NACK;
	 }
	 else
	 {
		RESET_SCL;
		return IIC_ACK;  
	 }    
}

u8 IIC_RecvByte(void)
{
	 u8 i,Dat = 0;
	 SET_SDA;
	 RESET_SCL; 
	 Dat=0;
	 for(i=0;i<8;i++)
	 {
		SET_SCL;
		IIC_DELAY; 
		Dat<<=1;
		if(IIC_SDA_STATE) 
		{
			Dat|=0x01; 
		}   
		RESET_SCL;
		IIC_DELAY;  
	 }
	 return Dat;
}

void Single_Write_IIC(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
    IIC_Start();                 
    IIC_SendByte(SlaveAddress);  
    IIC_SendByte(REG_Address);    
    IIC_SendByte(REG_data);       
    IIC_Stop();                   
}

u8 Single_Read_IIC(u8 SlaveAddress, u8 REG_Address)
{  
	u8 REG_data;
    IIC_Start();                         
    IIC_SendByte(SlaveAddress);          
    IIC_SendByte(REG_Address);            
    IIC_Start();                         
    IIC_SendByte(SlaveAddress+1);        
    REG_data = IIC_RecvByte();              
	IIC_SendACK();   
	IIC_Stop();                         
    return REG_data; 
}

uint16_t GPIO_Filter( GPIO_TypeDef * GPIOx )
{	 
	uint32_t RCC_GPIOx = 0; 

	if( GPIOx == GPIOA )
	{
		RCC_GPIOx = RCC_APB2Periph_GPIOA;
	}
	else if( GPIOx == GPIOA )
	{
		RCC_GPIOx = RCC_APB2Periph_GPIOA;
	}
	else if( GPIOx == GPIOB )
	{
		RCC_GPIOx = RCC_APB2Periph_GPIOB;
	}
	else if( GPIOx == GPIOC )
	{
		RCC_GPIOx = RCC_APB2Periph_GPIOC;
	}
	else if( GPIOx == GPIOD )
	{
		RCC_GPIOx = RCC_APB2Periph_GPIOD;
	}
	else if( GPIOx == GPIOE )
	{
		RCC_GPIOx = RCC_APB2Periph_GPIOE;
	}
	else if( GPIOx == GPIOF )
	{
		RCC_GPIOx = RCC_APB2Periph_GPIOF;
	}
	else if( GPIOx == GPIOG )
	{
		RCC_GPIOx = RCC_APB2Periph_GPIOG;
	}

	return RCC_GPIOx;
}

