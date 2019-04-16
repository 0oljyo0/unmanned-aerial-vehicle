/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "bsp_usart1.h"
#include "stm32f10x.h"
#include "bsp_led.h"
#include "mpu6050.h"
#include "bsp_breathing.h"
#include "SPI_NRF.h"
#include "bsp_TiMbase.h"
#include "math.h"
#include "ms5611.h"
#include "MPU6050_1.h"
#include "bsp_iwdg.h"   
#include "stm32f10x_iwdg.h"
#include "bsp_adc.h"

#define LED_T 300
uint16_t LEDjishuqi=0;
extern uint8_t txbuf[32];	 //发送缓冲
extern uint8_t rxbuf[32];	 //接收缓冲
extern uint8_t status;		 //用于判断接收/发送状态
extern uint16_t LEDs;
extern int16_t value1,value2,value3,value4,value5,value6;
extern float fvalue1,fvalue2,fvalue3,fvalue4,fvalue5,fvalue6;
extern uint8_t link_flag;
extern uint8_t lock_flag;
//uint8_t counter=0;
//uint8_t Temp[33];
//uint8_t sign=0;
float M1_i=0,M2_i=0,M3_i=0,M4_i=0;

float xuanzhuan_piancha_i=0,xuanzhuan_piancha_d=0;


//angle_qianhou=(accel_qianhou-angle_qianhou_want)*P+(angle_qianhou+gyro_qianhou*D);

short accel_qianhou=0,accel_zuoyou=0,accel_z=0;
int16_t youmen=0,zuoyou=500,qianhou=500,adc_value1=0,adc_value2=0;
uint16_t anjian_zuo=0,anjian_you=0;
short gyro_qianhou=0,gyro_zuoyou=0,gyro_z=0;

int32_t angle_qianhou=0,angle_qianhou_want=0,angle_zuoyou=0,angle_zuoyou_want=0,angle_xuanzhuan=0,oldangle_xuanzhuan=0,angle_xuanzhuan_want=0,xuanzhuan=0;
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
//	unsigned char i;
//	for(i=0;i<NumOfTask;i++)
//	{
//		if(Task_Delay[i])
//		{
//			Task_Delay[i]--;
//		}
//	}
}


void USART3_IRQHandler(void)
{
 
}
//void USART1_IRQHandler(void)
//{
//	uint8_t ch;
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//	{ 	
//			ch = USART_ReceiveData(USART1);
//	  	printf( "%c", ch );    //将接受到的数据直接返回打印
//	} 	 
//}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 







/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/


void int_xianfu(int16_t *a,int16_t max,int16_t min)   //整数限幅函数 
{
	if(*a>max)
		*a=max;
	if(*a<min)
		*a=min;
}

void float_xianfu(float *a,int16_t max,int16_t min)   //浮点数限幅函数 
{
	if(*a>max)
		*a=max;
	if(*a<min)
		*a=min;
}

void LEDshanshuo()
{ 
	LEDjishuqi++;
	if(LEDjishuqi<=LEDs)
		LED1_ON;
	if(LEDjishuqi>LEDs)
		LED1_OFF;
	if(LEDjishuqi>=LED_T)
		LEDjishuqi=0;
}

void huoqu_jiaosudu(int16_t *qianhou,int16_t *zuoyou,int16_t *xuanzhuan)    //获取三轴角速度值
{
//	*qianhou = ((short)(Temp[14]<<8| Temp[13]));  //向后旋转为正
//	*zuoyou = ((short)(Temp[16]<<8| Temp[15]));   //向左旋转为正
//	*xuanzhuan = ((short)(Temp[18]<<8| Temp[17])); //旋转角度		
}
void huoqu_jiaodu(int16_t *qianhou,int16_t *zuoyou,int16_t *xuanzhuan)      //获取三轴角度值
{
	int16_t qianhou_jiaozheng,zuoyou_jiaozheng;
	
	qianhou_jiaozheng=((rxbuf[8]*100+rxbuf[9])-2047)/12;
	zuoyou_jiaozheng=-((rxbuf[10]*100+rxbuf[11])-2047)/12;
//	*qianhou = ((short)(Temp[25]<<8| Temp[24]));                  //读取计算X轴滚转角（x 轴） 前后倾角
	if(*qianhou<0)
	 *qianhou+=32767;
	else
	 *qianhou-=32767;
	*qianhou/=30.0;
//	*zuoyou = (((short)(Temp[27]<<8| Temp[26]))-5)/30.0;          //读取计算Y轴俯仰角（y 轴） 左右倾角
//	*xuanzhuan = ((short)(Temp[29]<<8| Temp[28]))*180/32650*20;   //读取计算Z轴偏航角（z 轴）	旋转角度
	//printf("%d,%d\n",qianhou_jiaozheng*10,zuoyou_jiaozheng*10);
	*qianhou+=10;
	*zuoyou+=1;
}

void dianji_fuzhi(int16_t want_youmen,int16_t M1,int16_t M2,int16_t M3,int16_t M4)
{
	if(want_youmen>=20&&link_flag==1&&lock_flag==1)
	{
		motor1=M1;
		motor2=M2;
	  motor3=M3;
		motor4=M4;
	}
	else
	{
		motor1=0;
		motor2=0;
		motor3=0;
		motor4=0;
	}
}

void huoqu_yaokongqi(int16_t *youmen,int16_t *qianhou,int16_t *zuoyou,int16_t *xuanzhuan,int16_t *adc_value1,int16_t *adc_value2,uint16_t *anjian_zuo,uint16_t *anjian_you)
{
	*youmen=*qianhou=*zuoyou=*xuanzhuan=*adc_value1=*adc_value2=0;
	*zuoyou=rxbuf[1]<<8|rxbuf[0];
	*qianhou=1000-(rxbuf[3]<<8|rxbuf[2]);
	*xuanzhuan=rxbuf[5]<<8|rxbuf[4];
	*youmen=rxbuf[7]<<8|rxbuf[6];
	*adc_value1=rxbuf[9]<<8|rxbuf[8];
	*adc_value2=rxbuf[11]<<8|rxbuf[10];
	*anjian_zuo=rxbuf[13]<<8|rxbuf[12];
	*anjian_you=rxbuf[15]<<8|rxbuf[14];
}

float x,y,z,x_measure,y_measure,z_measure,easing_accel=0.999;
void angle_motor()
{
	int16_t M1=0,M2=0,M3=0,M4=0;   
	int16_t qianhou_piancha=0,zuoyou_piancha=0,xuanzhuan_piancha=0;
//	int16_t w_qianhou,w_zuoyou,w_xuanzhuan;
//	int16_t a_qianhou,a_zuoyou,a_xuanzhuan;
	int16_t want_youmen=0,want_qianhou=0,want_zuoyou=0,want_xuanzhuan=0;
	float M1_p=0,M2_p=0,M3_p=0,M4_p=0;
	float M1_d=0,M2_d=0,M3_d=0,M4_d=0;	
	float tuoluoyi_P=0.006,tuoluoyi_D=0.062,yaokongqi_P=0.2,yaokongqi_D=0.5,I=0.1;
	float xuanzhuan_P=4.0,xuanzhuan_D=94.0;
	float Pitch,Roll,Yaw;
	short gyro[3];
	uint16_t anjian_zuo_old,youmen_old;
  #define gX 0
	#define gY 1
	#define gZ 2
	
// 获取各轴角度
	if(!Read_DMP(gyro,&Pitch,&Roll,&Yaw))
	{
		angle_qianhou=1000*Pitch;
		angle_zuoyou=-1000*Roll;
//		angle_xuanzhuan=-1000*Yaw;
		xuanzhuan+=gyro[gZ]/10;
		oldangle_xuanzhuan=angle_xuanzhuan;
		angle_xuanzhuan=xuanzhuan/100;
		
		gyro_qianhou=gyro[gX];
		gyro_zuoyou=gyro[gY];
		gyro_z=gyro[gZ];
	}
	anjian_zuo_old=anjian_zuo;
	youmen_old=youmen;
	huoqu_yaokongqi(&youmen,&zuoyou,&qianhou,&want_xuanzhuan,&adc_value1,&adc_value2,&anjian_zuo,&anjian_you);  //获取遥控器值
	
	if(youmen_old!=0&&youmen==0)        
	{
		lock_flag=0;
	}
	if(anjian_zuo_old==65535&&anjian_zuo==32767)
	{
		lock_flag=1-lock_flag;
	}
	txbuf[2]=lock_flag;
	
//	tuoluoyi_P=277/100000.0;   //参数调试好了
//	tuoluoyi_D=536/10000.0;	   //参数调试好了
	
	tuoluoyi_P=adc_value1/100000.0;   //参数调试好了
	tuoluoyi_D=adc_value2/10000.0;	   //参数调试好了
	
//	value1=angle_qianhou;
//	value2=angle_zuoyou;
//	value3=angle_xuanzhuan;
	
	M1_p=angle_qianhou*tuoluoyi_P+(qianhou-500)*yaokongqi_P-angle_zuoyou*tuoluoyi_P+(zuoyou-500)*yaokongqi_P+angle_xuanzhuan*xuanzhuan_P;
	M2_p=-angle_qianhou*tuoluoyi_P-(qianhou-500)*yaokongqi_P-angle_zuoyou*tuoluoyi_P+(zuoyou-500)*yaokongqi_P-angle_xuanzhuan*xuanzhuan_P;
	M3_p=-angle_qianhou*tuoluoyi_P-(qianhou-500)*yaokongqi_P+angle_zuoyou*tuoluoyi_P-(zuoyou-500)*yaokongqi_P+angle_xuanzhuan*xuanzhuan_P;
	M4_p=angle_qianhou*tuoluoyi_P+(qianhou-500)*yaokongqi_P+angle_zuoyou*tuoluoyi_P-(zuoyou-500)*yaokongqi_P-angle_xuanzhuan*xuanzhuan_P;
	
	M1_d=(-gyro_qianhou-gyro_zuoyou)*tuoluoyi_D-(angle_xuanzhuan-oldangle_xuanzhuan)*xuanzhuan_D;
	M2_d=(gyro_qianhou-gyro_zuoyou)*tuoluoyi_D+(oldangle_xuanzhuan-angle_xuanzhuan)*xuanzhuan_D;
	M3_d=(gyro_qianhou+gyro_zuoyou)*tuoluoyi_D-(angle_xuanzhuan-oldangle_xuanzhuan)*xuanzhuan_D;
	M4_d=(-gyro_qianhou+gyro_zuoyou)*tuoluoyi_D+(oldangle_xuanzhuan-angle_xuanzhuan)*xuanzhuan_D;


  M1=youmen+M1_p-M1_d;
	M2=youmen+M2_p+M2_d;
	M3=youmen+M3_p-M3_d;
	M4=youmen+M4_p+M4_d;
	
	int_xianfu(&M1,1000,0);
	int_xianfu(&M2,1000,0);
	int_xianfu(&M3,1000,0);
	int_xianfu(&M4,1000,0);	

	dianji_fuzhi(youmen,M1,M2,M3,M4);   //直接把计算好的值赋给电机
}

extern uint32_t pressure;
void  macTIM_INT_FUN2 (void)
{
	if ( TIM_GetITStatus( macTIMx2, TIM_IT_Update) != RESET ) 
	{
		IWDG_Feed();
		LEDshanshuo();

		angle_motor();
		TIM_ClearITPendingBit(macTIMx2 , TIM_FLAG_Update);  		 
	}		 	
}


uint16_t count2=0;
void macTIMx_IRQHandler(void)
{	
	if (TIM_GetITStatus(macTIMx, TIM_IT_Update) != RESET)	//TIM_IT_Update
 	{			
		count2++;
		if(count2>=1000)
		{
			count2=0;
			MS561101BA_GetTemperature();
			MS561101BA_GetPressure(&pressure);
		}

		TIM_ClearITPendingBit (macTIMx, TIM_IT_Update);	//必须要清除中断标志位
	}
}