#include "ms5611.h"
#include "bsp_i2c.h"
#include "bsp_led.h"

//u8 exchange_Pres_num[8];//压力数据
//u8 exchange_Temp_num[8];//温度数据
uint32_t D1_Pres,D2_Temp;	// 数字压力值,数字温度值
uint64_t dT,TEMP;
uint64_t OFf,SENS;
float Aux;
uint32_t TEMP2,T2,OFF2,SENS2;
//uint32_t Pressure,Pressure_old,qqp;	
double OFF_;
u16 Cal_C[7];  //用于存放PROM中的8组数据
//u32 D1_Pres,D2_Temp; // 存放数字压力和温度
//float Pressure;				//温度补偿大气压
//float dT,Temperature,Temperature2;//实际和参考温度之间的差异,实际温度,中间值
//double OFF,SENS;  //实际温度抵消,实际温度灵敏度
defMS5611data ms5611data;
static __IO uint32_t  I2CTimeout = I2CT_LONG_TIMEOUT; 

#define OLD_NUM 40

uint32_t pressure_old[OLD_NUM]={0};
uint32_t pressure_init=0;

//=========================================================
//******MS561101BA程序********
//=========================================================

uint8_t  MS5611_write_com(uint8_t com)
{
	
	while(I2C_GetFlagStatus(macI2C2, I2C_FLAG_BUSY));
	I2C_AcknowledgeConfig(macI2C2,ENABLE);
//printf("9");
	I2C_GenerateSTART(macI2C2, ENABLE);
  while(!I2C_CheckEvent(macI2C2, I2C_EVENT_MASTER_MODE_SELECT));
//printf("8");
	I2C_Send7bitAddress(macI2C2, MS561101BA_SlaveAddress, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(macI2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//printf("7");
	I2C_SendData(macI2C2,com);
	while(!I2C_CheckEvent(macI2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//printf("6");
	I2C_GenerateSTOP(macI2C2, ENABLE);
//printf("5");
	return 1; //正常返回1
}

uint8_t  MS5611_get_buff(u32 * buff,uint8_t nbytetoread)
{
	uint8_t num=0;
//	printf("q");
	while(I2C_GetFlagStatus(macI2C2, I2C_FLAG_BUSY));
	I2C_AcknowledgeConfig(macI2C2,ENABLE);
//	printf("w");
	I2C_GenerateSTART(macI2C2, ENABLE);
	while(!I2C_CheckEvent(macI2C2, I2C_EVENT_MASTER_MODE_SELECT));
//	printf("e");
	I2C_Send7bitAddress(macI2C2, MS561101BA_SlaveAddress+0x01, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(macI2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
//  printf("r");
	while(nbytetoread)
	{
		if(nbytetoread==1)
		{
			I2C_AcknowledgeConfig(macI2C2, DISABLE);
			I2C_GenerateSTOP(macI2C2, ENABLE);
		}
		while(!(I2C_CheckEvent(macI2C2,I2C_EVENT_MASTER_BYTE_RECEIVED)));
		buff[num] = I2C_ReceiveData(macI2C2);
		num++;
		nbytetoread--;
	}
//	printf("t");
	//printf("d1=%d d2=%d\n",buff[0],buff[1]);
	I2C_AcknowledgeConfig(macI2C2, ENABLE);
}

void MS561101BA_Init(void)
{
	
	uint8_t i=0;
	uint32_t pressure=0,pressure_he=0;
	delay_ms(100);
	
	MS561101BA_RESET();
	delay_ms(100);
	
	MS561101BA_PROM_READ();
	delay_ms(100);
	
	for(i=0;i<=OLD_NUM;i++)
	{
		MS561101BA_GetTemperature();
		MS561101BA_GetPressure(&pressure);
	}
	pressure_init=pressure;
	printf(" %d\n\n",pressure);
} 

uint8_t MS561101BA_RESET(void)//如数据读不出来，或程序跑不下去，请注释掉此处while(IIC_Wait_Ack());
{	
	MS5611_write_com(MS561101BA_RST);
}

//从PROM读取出厂校准数据
uint8_t MS561101BA_PROM_READ(void)
{
	u32 d[2];
	u8 i;
	for(i=1;i<=6;i++)
	{
		MS5611_write_com((MS561101BA_PROM_RD+i*2));
		MS5611_get_buff(d,2);
		Cal_C[i]=(d[0]<<8)|d[1];
	}//打印PROM读取出厂校准数据，检测数据传输是否正常
	return 1;
}

u32 MS561101BA_DO_CONVERSION(u8 command)
{
	unsigned long conversion = 0;
	u32 temp[3];
//	printf("1");
	MS5611_write_com(command);
	delay_ms(50);	
//	printf("2");
	MS5611_write_com(0x00);
//	printf("3");
	MS5611_get_buff(temp,3);
//  printf("4");
	conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
	return conversion;
}


//读取数字温度
void MS561101BA_GetTemperature(void)
{	
//	printf("a");
	D2_Temp = MS561101BA_DO_CONVERSION(0x58);
	
	//delay_ms(10);
	dT=D2_Temp - (((uint32_t)Cal_C[5])<<8);
	TEMP=2000+dT*((uint32_t)Cal_C[6])/8388608;
}

//读取数字气压
void MS561101BA_GetPressure(uint32_t * pressure)
{
	
	uint8_t i=0;
	uint32_t pressure_he=0;
	
//	printf("b");
	D1_Pres= MS561101BA_DO_CONVERSION(0x48);
	//delay_ms(10);
	
	
	OFF_=(uint32_t)Cal_C[2]*65536+((uint32_t)Cal_C[4]*dT)/128;
	SENS=(uint32_t)Cal_C[1]*32768+((uint32_t)Cal_C[3]*dT)/256;

	if(TEMP<2000)
	{
		Aux = (2000-TEMP)*(2000-TEMP);
		T2 = (dT*dT) / 0x80000000; 
		OFF2 = 2.5f*Aux;
		SENS2 = 1.25f*Aux;
		
		TEMP = TEMP - T2;
		OFF_ = OFF_ - OFF2;
		SENS = SENS - SENS2;	
	}
	*pressure=(D1_Pres*SENS/2097152-OFF_)/32768;
	
	for(i=0;i<OLD_NUM;i++)
	{
		pressure_old[OLD_NUM-i]=pressure_old[OLD_NUM-1-i];
	}
	pressure_old[0]=*pressure;
	
	//printf("%d,%d,%d,%d\n",*pressure,pressure_old[0],pressure_old[1],pressure_old[2]);
	
	for(i=0;i<OLD_NUM;i++)
	{
		pressure_he+=pressure_old[i];
	}
	
	*pressure=pressure_he/OLD_NUM;
	//printf("%d\n",*pressure);
  //Pressure= (D1_Pres*SENS/2097152-OFF_)/32768;
	//printf("%d\n",Pressure);
}

void MS561101BA_readdata(defMS5611data *ms5611data)
{
	float dT,readdata,T1,T2,Aux,OFF2,SENS2;
	double OFF1,SENS;  //实际温度抵消,实际温度灵敏度
	readdata= MS561101BA_DO_CONVERSION(MS561101BA_D2_OSR_4096);	
//	delay_ms(10);
//	printf("%u /n",D2_Temp);
	dT=readdata - (((u32)Cal_C[5])<<8);
	T1=(s16)(2000+dT*((u32)Cal_C[6])/8388608);
//	ms5611data->temperature=T1;
	
	
	readdata= MS561101BA_DO_CONVERSION(MS561101BA_D1_OSR_4096);
	OFF1=(u32)(Cal_C[2]<<16)+((u32)Cal_C[4]*dT)/128;
	SENS=(u32)(Cal_C[1]<<15)+((u32)Cal_C[3]*dT)/256;
	//温度补偿
	if(T1 < 2000)// second order temperature compensation when under 20 degrees C
	{
		T2 = (dT*dT) / 0x80000000;
		Aux = (T1-2000)*(T1-2000);
		OFF2 = 2.5*Aux;
		SENS2 = 1.25*Aux;
		if(T1 < -1500)
		{
			Aux = (T1+1500)*(T1+1500);
			OFF2 = OFF2 + 7*Aux;
			SENS2 = SENS + 5.5*Aux;
		}
	}else  //(Temperature > 2000)
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}	
	T1 -= T2;
	OFF1 = OFF - OFF2;
	SENS = SENS - SENS2;	
	ms5611data->pressure=(u32)((readdata*SENS/2097152-OFF)/32768);
	ms5611data->temperature=T1;
}

/***************数据转换******************* 
void Exchange_Number(void) 
{ 
	u32 ex_Pressure;	//串口读数转换值
	u32 ex_Temperature; //串口读数转换值

	MS561101BA_GetTemperature(MS561101BA_D2_OSR_4096);
	MS561101BA_GetPressure(MS561101BA_D1_OSR_4096);
	
	ex_Pressure=(long)(Pressure); 
	
	exchange_Pres_num[0]=ex_Pressure/1000000+'0';
	exchange_Pres_num[1]=ex_Pressure/100000%10+'0';
	exchange_Pres_num[2]=ex_Pressure/10000%10+'0';
	exchange_Pres_num[3]=ex_Pressure/1000%10+'0';
	exchange_Pres_num[4]=ex_Pressure/100%10+'0';
	exchange_Pres_num[5]='.';
	exchange_Pres_num[6]=ex_Pressure/10%10+'0'; 
	exchange_Pres_num[7]=ex_Pressure%10+'0'; 
	printf("Atm: ");
	Usart_Send(exchange_Pres_num,8);
	printf("mbar\n");
	
	ex_Temperature=(long)(Temperature); 
	
	exchange_Temp_num[0]=ex_Temperature/1000000+'0';
	exchange_Temp_num[1]=ex_Temperature/100000%10+'0';
	exchange_Temp_num[2]=ex_Temperature/10000%10+'0';
	exchange_Temp_num[3]=ex_Temperature/1000%10+'0';
	exchange_Temp_num[4]=ex_Temperature/100%10+'0';
	exchange_Temp_num[5]='.';
	exchange_Temp_num[6]=ex_Temperature/10%10+'0'; 
	exchange_Temp_num[7]=ex_Temperature%10+'0'; 
	printf("Tem: ");
	Usart_Send(exchange_Temp_num,8);
	printf("℃\n");
} 
*/
		 


