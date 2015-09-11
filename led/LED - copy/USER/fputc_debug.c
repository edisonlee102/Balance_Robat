/**

   @file
 fputc_debug.c

   @brief
 Trying to redirect printf() to debug port

   @date
 2012/06/25

*/

#include <stdio.h>

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
int fputc(int ch,FILE *f)
{
    USART_SendData(USART1,(u8) ch);
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
    return ch;
}
 
