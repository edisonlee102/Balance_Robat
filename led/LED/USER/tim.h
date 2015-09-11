#include <stdint.h>
#define  TIM_1  0x01
#define  TIM_2  0x02
#define  TIM_3  0x03
#define  TIM_4  0x04
#define  TIM_5  0x05
#define  TIM_6  0x06
#define  TIM_7  0x07
#define  TIM_8  0x08
#define  CCR_1  0x01
#define  CCR_2  0x02
#define  CCR_3  0x03
#define  CCR_4  0x04
void Tim_Init(uint8_t TIM_x,uint16_t arr,uint16_t psc);
void Tim_CCR_Set(uint8_t TIM_x,uint8_t CCR_x,uint32_t val);
