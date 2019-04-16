#ifndef __USART1_H
#define	__USART1_H


#include "stm32f10x.h"
#include <stdio.h>



/**************************USART参数定义********************************/
#define             macUSART1_BAUD_RATE                       115200

#define             macUSART1                                 USART1
#define             macUSART1_APBxClock_FUN                   RCC_APB2PeriphClockCmd
#define             macUSART1_CLK                             RCC_APB2Periph_USART1
#define             macUSART1_GPIO_APBxClock_FUN              RCC_APB2PeriphClockCmd
#define             macUSART1_GPIO_CLK                        RCC_APB2Periph_GPIOA     
#define             macUSART1_TX_PORT                         GPIOA   
#define             macUSART1_TX_PIN                          GPIO_Pin_9
#define             macUSART1_RX_PORT                         GPIOA 
#define             macUSART1_RX_PIN                          GPIO_Pin_10
#define             macUSART1_IRQ                             USART1_IRQn
#define             macUSART1_INT_FUN                         USART1_IRQHandler



void                USART1_Config                           ( void );



#endif /* __USART1_H */
