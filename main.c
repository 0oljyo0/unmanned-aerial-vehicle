#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "bsp_SysTick.h"
#include "bsp_led.h"
#include "bsp_usart1.h"
#include "mpu6050.h"
#include "bsp_i2c.h"
#include "bsp_breathing.h"
#include "SPI_NRF.h"
#include "bsp_TiMbase.h"
#include "ms5611.h"
#include "delay.h"
#include "MPU6050_1.h"
#include "ioi2c.h"
#include "bsp_iwdg.h"
#include "stm32f10x_iwdg.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "bsp_adc.h"

uint16_t LEDs=200;
uint8_t status;		 //ÓÃÓÚÅÐ¶Ï½ÓÊÕ/·¢ËÍ×´Ì¬
uint8_t txbuf[32]={0};	 //·¢ËÍ»º³å
uint8_t rxbuf[32]={1,0};	 //½ÓÊÕ»º³å
int16_t value1,value2,value3,value4,value5,value6;

float fvalue1,fvalue2,fvalue3,fvalue4,fvalue5,fvalue6;

extern __IO uint16_t ADC_ConvertedValue;

extern short accqianhou_jiaoyan ,acczuoyou_jiaoyan ,accz_jiaoyan ,gyroqianhou_jiaoyan ,gyrozuoyou_jiaoyan ,gyroz_jiaoyan ;
extern uint32_t pressure_init;
extern int32_t angle_qianhou,angle_qianhou_want;

uint16_t uart_jishu=0,recv_jishu=0; 
uint32_t pressure=0;
uint8_t link_flag=0;
uint8_t lock_flag=0;
#define uart_v 3
#define recv_v 1

int main(void)
{	
	USART1_Config();  //´®¿ÚÍ¨ÐÅ³õÊ¼»¯
	printf("usart1 init success\n");
	ADCx_Init();

	LED_GPIO_Config();//LED ¶Ë¿Ú³õÊ¼»¯ 
	
	delay_init(16);   //ÏµÍ³ÑÓÊ±³õÊ¼»¯
	
	SPI_NRF_Init();   //NRF³õÊ¼»¯
	NRF_RX_Mode();
	
	I2C_Bus_Init();	  //ÆøÑ¹¼Æ³õÊ¼»¯
	MS561101BA_Init();
	
	IIC_Init();
	DMP_Init();
	
	TIM3_PWM_Init();  //³õÊ¼»¯PWM²¨Êä³
	
	TIMx_Configuration();// ¶¨Ê±Æ÷ TIMx,x[1,8] ¶¨Ê±ÅäÖÃ 10msÖÐ¶Ï
	TIMx_NVIC_Configuration();	//ÅäÖÃ¶¨Ê±Æ÷ TIMx,x[1,8]µÄÖÐ¶ÏÓÅÏÈ¼¶ 
	macTIM_APBxClock_FUN2 (macTIM_CLK2, ENABLE);
	
  IWDG_Config(IWDG_Prescaler_4 ,0xfff);
	while(1)
	{
		status=NRF_Rx_Dat(rxbuf);
		if(rxbuf[31]==1)   //¼ì²âÊÇ·ñÔÊÐí·¢ËÍÊý¾Ý
		{
			txbuf[0]=ADC_ConvertedValue>>8;       //×°ÔØµç³ØµçÑ¹
			txbuf[1]=ADC_ConvertedValue&0x00ff;
			
			txbuf[2]=pressure&0x000000ff;         //×°ÔØÆøÑ¹Öµ
			txbuf[3]=pressure>>8&0x000000ff;
			txbuf[4]=pressure>>16&0x000000ff;
			txbuf[5]=pressure>>24&0x000000ff;
			
			TX_Mode();
			NRF_Tx_Dat(txbuf);
			RX_Mode();
			rxbuf[31]=0;
		}

		uart_jishu++;
		if(uart_jishu>=uart_v)
		{		
			uart_jishu=0;	
//			printf("%8d,%8d,%8d,%8d,%8d,%8d\n",value1,value2,value3,value4,value5,value6);
//			printf("%1.3f,%1.3f,%8d,%8d,%8d\n",fvalue1,fvalue2,value3,value4,pressure);
			printf("%d\n",pressure);
		}
	}
}
