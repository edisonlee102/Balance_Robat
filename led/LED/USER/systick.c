#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x.h"
#define SYSCLK_FREQ_72MHz  72000000
#define SysTick_CTRL_ENABLE_Pos 0
#define SysTick_CTRL_ENABLE_Msk (1ul << SysTick_CTRL_ENABLE_Pos)
int TimingDelay;
void Systick_Init(void)
{
	//SystemFrequency / 1000 1ms once time
	if (SysTick_Config(SYSCLK_FREQ_72MHz / 1000))
	{
			/* Capture error */
			while (1);
	}
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}
void Delay_ms(__IO u32 nTime)
{
	TimingDelay = nTime;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	while(TimingDelay != 0);
	
}
