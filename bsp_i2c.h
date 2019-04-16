
#ifndef __BSP_I2C_H
#define	__BSP_I2C_H

#include "stm32f10x.h"
#include "bsp_usart1.h"


/**************************I2C参数定义，I2C1或I2C2********************************/
#define             macI2Cx                                I2C1
#define             macI2C_APBxClock_FUN                   RCC_APB1PeriphClockCmd
#define             macI2C_CLK                             RCC_APB1Periph_I2C1
#define             macI2C_GPIO_APBxClock_FUN              RCC_APB2PeriphClockCmd
#define             macI2C_GPIO_CLK                        RCC_APB2Periph_GPIOB     
#define             macI2C_SCL_PORT                        GPIOB   
#define             macI2C_SCL_PIN                         GPIO_Pin_6
#define             macI2C_SDA_PORT                        GPIOB 
#define             macI2C_SDA_PIN                         GPIO_Pin_7

#define             macI2C2                                I2C2
//#define             macI2C_APBxClock_FUN                   RCC_APB1PeriphClockCmd
#define             macI2C2_CLK                            RCC_APB1Periph_I2C2
//#define             macI2C_GPIO_APBxClock_FUN              RCC_APB2PeriphClockCmd
//#define             macI2C_GPIO_CLK                        RCC_APB2Periph_GPIOB     
#define             macI2C_SCL_PORT2                       GPIOB   
#define             macI2C_SCL_PIN2                        GPIO_Pin_10
#define             macI2C_SDA_PORT2                       GPIOB 
#define             macI2C_SDA_PIN2                        GPIO_Pin_11

/*等待超时时间*/
#define I2CT_FLAG_TIMEOUT         ((uint32_t)0x10000)
#define I2CT_LONG_TIMEOUT         ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))

/*信息输出*/
#define MPU_DEBUG_ON         1

#define MPU_INFO(fmt,arg...)           printf("<<-MPU-INFO->> "fmt"\n",##arg)
#define MPU_ERROR(fmt,arg...)          printf("<<-MPU-ERROR->> "fmt"\n",##arg)
#define MPU_DEBUG(fmt,arg...)          do{\
                                          if(MPU_DEBUG_ON)\
                                          printf("<<-MPU-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)


void I2C_Bus_Init(void);
uint8_t I2C_ByteWrite(u8 DeviceAddr,u8 pBuffer, u8 WriteAddr);
uint8_t I2C_BufferRead(u8 DeviceAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
void I2C_WaitStandbyState(void);

#endif /* __BSP_I2C_H */
