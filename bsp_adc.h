#ifndef __ADC_H
#define	__ADC_H


#include "stm32f10x.h"

// 注意：用作ADC采集的IO必须没有复用，否则采集电压会有影响
/********************ADC1输入通道（引脚）配置**************************/
#define    macADC_APBxClock_FUN             RCC_APB2PeriphClockCmd
#define    macADC_CLK                       RCC_APB2Periph_ADC1

#define    macADC_GPIO_APBxClock_FUN        RCC_APB2PeriphClockCmd
#define    macADC_GPIO_CLK                  RCC_APB2Periph_GPIOA  
#define    macADC_PORT                      GPIOA

// PC1-通道11 独立IO
#define    macADC_PIN                       GPIO_Pin_4
#define    macADC_CHANNEL                   ADC_Channel_4

#define    macADCx                          ADC1
#define    macADC_DMA_CHANNEL               DMA1_Channel1


/**************************函数声明********************************/
void               ADCx_Init                               (void);


#endif /* __ADC_H */

