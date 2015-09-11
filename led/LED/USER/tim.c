#include "tim.h"
#include "stm32f10x.h"
void Tim_Init(uint8_t TIM_x,uint16_t arr,uint16_t psc)
{
switch(TIM_x)
{
    case 1 :{
        RCC->APB2ENR |=1<<11;
        break;
    }
    case 2 :{
        RCC->APB1ENR |=1<<0;
        TIM2->ARR = arr;     
        TIM2->PSC = psc;      
        TIM2->DIER |= 1<<0;
        TIM2->DIER |= 1<<6;       
        TIM2->CR1 |= 0x81;                        
        break;
    }
    case 3 :{
        RCC->APB1ENR |=1<<1;
        break;
    }
    case 4 :{
        RCC->APB1ENR |=1<<2;              
        break;
    }
    case 5 :{
        RCC->APB1ENR |=1<<3;      
        break;
    }
    case 6 :{
        RCC->APB1ENR |=1<<4;
        break;
    }  
    case 7 :{
        RCC->APB1ENR |=1<<5;  
        break;
    }
    case 8 :{
        RCC->APB2ENR |=1<<13;
        break;
    }
}
}
void Tim_CCR_Set(uint8_t TIM_x,uint8_t CCR_x,uint32_t val)
{
 switch(TIM_x)
 {
     case 1 :{
         break;
     }
     case 2 :{
         TIM2->DIER |= 1 << CCR_x;      
         switch(CCR_x){
             case 1: {
                 TIM2 ->CCR1 = val;      
                 break;
             }
             case 2: {
                 TIM2 ->CCR2 = val;    
                 break;
             }
             case 3: {
                 TIM2 ->CCR3 = val;      
                 break;
             }
             case 4: {
                 TIM2 ->CCR4 = val;     
                 break;
             }
         }
         break;
     }
     case 3 :{
         break;
     }
     case 4 :{
         break;
     }
     case 5 :{
         break;
     }
     case 6 :{
         break;
     }  
     case 7 :{
         break;
     }
     case 8 :{
         break;
     }
 }
}
