/******************************************************************************/
/* SERIAL.C: Low Level Serial Routines                                        */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2007 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))


#include "stm32f10x_usart.h"                    /* STM32F10x Library Definitions      */
#include "stm32f10x.h"

/* Implementation of putchar (also used by printf function to output data)    */
int sendchar (int ch)                             /* Write character to Serial Port     */
{
    USART_SendData(USART1, (unsigned char) ch);
     /* Loop until the end of transmission */
   while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
   {
   }
    return (ch);
}


int getkey (void)                                 /* Read character from Serial Port    */
{

    while (!(USART1->SR & USART_FLAG_RXNE));
    return (USART_ReceiveData(USART1));
}