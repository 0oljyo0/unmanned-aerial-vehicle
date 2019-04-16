#ifndef __MS5611_H__
#define __MS5611_H__


//#include "sys.h"
//#include "myiic.h"
#include "delay.h"
//#include "usart.h"

#include <math.h> //Keil library 
#include <stdlib.h> //Keil library 
#include <stdio.h> //Keil library  


#define  MS561101BA_SlaveAddress 0xee  //定义器件在IIC总线中的从地址

#define  MS561101BA_D1 0x40 
#define  MS561101BA_D2 0x50 
#define  MS561101BA_RST 0x1E 

//#define  MS561101BA_D1_OSR_256 0x40 
//#define  MS561101BA_D1_OSR_512 0x42 
//#define  MS561101BA_D1_OSR_1024 0x44 
//#define  MS561101BA_D1_OSR_2048 0x46 
#define  MS561101BA_D1_OSR_4096 0x48 

//#define  MS561101BA_D2_OSR_256 0x50 
//#define  MS561101BA_D2_OSR_512 0x52 
//#define  MS561101BA_D2_OSR_1024 0x54 
//#define  MS561101BA_D2_OSR_2048 0x56 
#define  MS561101BA_D2_OSR_4096 0x58 

#define  MS561101BA_ADC_RD 0x00 
#define  MS561101BA_PROM_RD 0xA0 
#define  MS561101BA_PROM_CRC 0xAE 

//////////////////////////////////////////////////////////////////////////////////// 
//extern float Pressure;				//温度补偿大气压
//extern float Temperature;//实际和参考温度之间的差异,实际温度,中间值
typedef struct
{
	s16 temperature;
	u32 pressure;
} defMS5611data;

extern defMS5611data ms5611data;


uint8_t  MS5611_write_com(uint8_t com);
uint8_t  MS5611_get_buff(u32 * buff,uint8_t nbytetoread);
uint8_t MS561101BA_RESET(void); 
uint8_t MS561101BA_PROM_READ(void); 
u32 MS561101BA_DO_CONVERSION(u8 command); 
void MS561101BA_GetTemperature();
void MS561101BA_GetPressure(uint32_t * pressure);
void MS561101BA_Init(void); 
void SeriPushSend(u8 send_data); 

//void Exchange_Number(void); 
void MS561101BA_readdata(defMS5611data *ms5611data);


#endif




