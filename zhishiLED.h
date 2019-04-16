#ifndef TIME_TEST_H
#define TIME_TEST_H


#include "stm32f10x.h"


/********************基本定时器TIM参数定义，只限TIM6、7************/
#define macTIM6 // 如果使用TIM7，注释掉这个宏即可

#ifdef  macTIM6 // 使用基本定时器TIM6
#define             macTIMx                 TIM6
#define             macTIM_APBxClock_FUN    RCC_APB1PeriphClockCmd
#define             macTIM_CLK              RCC_APB1Periph_TIM6
#define             macTIM_IRQ              TIM6_IRQn
#define             macTIM_INT_FUN          TIM6_IRQHandler

#else  // 使用基本定时器TIM7
#define             macTIMx                 TIM7
#define             macTIM_APBxClock_FUN    RCC_APB1PeriphClockCmd
#define             macTIM_CLK              RCC_APB1Periph_TIM7
#define             macTIM_IRQ              TIM7_IRQn
#define             macTIM_INT_FUN          TIM7_IRQHandler

#endif
/**************************函数声明********************************/
void               TIMx_NVIC_Configuration                   (void);
void               TIMx_Configuration                        (void);


#endif	/* TIME_TEST_H */


