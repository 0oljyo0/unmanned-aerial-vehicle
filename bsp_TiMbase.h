#ifndef TIME_TEST_H
#define TIME_TEST_H


#include "stm32f10x.h"


/********************高级定时器TIM参数定义，只限TIM1、8************/

#define             macTIMx2                                TIM1
#define             macTIM_APBxClock_FUN2                   RCC_APB2PeriphClockCmd
#define             macTIM_CLK2                             RCC_APB2Periph_TIM1
#define             macTIM_IRQ2                             TIM1_UP_IRQn
#define             macTIM_INT_FUN2                         TIM1_UP_IRQHandler


/**************************函数声明********************************/
void               TIMx_NVIC_Configuration                   (void);
void               TIMx_Configuration                        (void);


#endif	/* TIME_TEST_H */


