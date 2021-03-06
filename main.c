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
uint8_t status;		 //用于判断接收/发送状态
uint8_t txbuf[32]={0};	 //发送缓冲
uint8_t rxbuf[32]={1,0};	 //接收缓冲
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
	USART1_Config();  //串口通信初始化
	printf("usart1 init success\n");
	ADCx_Init();

	LED_GPIO_Config();//LED 端口初始化 
	
	delay_init(16);   //系统延时初始化
	
	SPI_NRF_Init();   //NRF初始化
	NRF_RX_Mode();
	
	I2C_Bus_Init();	  //气压计初始化
	MS561101BA_Init();
	
	IIC_Init();
	DMP_Init();
	
	TIM3_PWM_Init();  //初始化PWM波输�
	
	TIMx_Configuration();// 定时器 TIMx,x[1,8] 定时配置 10ms中断
	TIMx_NVIC_Configuration();	//配置定时器 TIMx,x[1,8]的中断优先级 
	macTIM_APBxClock_FUN2 (macTIM_CLK2, ENABLE);
	
  IWDG_Config(IWDG_Prescaler_4 ,0xfff);
	while(1)
	{
		status=NRF_Rx_Dat(rxbuf);
		if(rxbuf[31]==1)   //检测是否允许发送数据
		{
			txbuf[0]=ADC_ConvertedValue>>8;       //装载电池电压
			txbuf[1]=ADC_ConvertedValue&0x00ff;
			
			txbuf[2]=pressure&0x000000ff;         //装载气压值
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
