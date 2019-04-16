#ifndef TIME_TEST_H
#define TIME_TEST_H


#include "stm32f10x.h"


/********************������ʱ��TIM�������壬ֻ��TIM6��7************/
#define macTIM6 // ���ʹ��TIM7��ע�͵�����꼴��

#ifdef  macTIM6 // ʹ�û�����ʱ��TIM6
#define             macTIMx                 TIM6
#define             macTIM_APBxClock_FUN    RCC_APB1PeriphClockCmd
#define             macTIM_CLK              RCC_APB1Periph_TIM6
#define             macTIM_IRQ              TIM6_IRQn
#define             macTIM_INT_FUN          TIM6_IRQHandler

#else  // ʹ�û�����ʱ��TIM7
#define             macTIMx                 TIM7
#define             macTIM_APBxClock_FUN    RCC_APB1PeriphClockCmd
#define             macTIM_CLK              RCC_APB1Periph_TIM7
#define             macTIM_IRQ              TIM7_IRQn
#define             macTIM_INT_FUN          TIM7_IRQHandler

#endif
/**************************��������********************************/
void               TIMx_NVIC_Configuration                   (void);
void               TIMx_Configuration                        (void);


#endif	/* TIME_TEST_H */


