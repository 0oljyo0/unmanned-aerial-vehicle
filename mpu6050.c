#include "mpu6050.h"
#include "bsp_usart1.h"
#include "bsp_i2c.h"
#include "delay.h"
/**
  * @brief   写数据到MPU6050寄存器
  * @param   
  * @retval  
  */

short accqianhou_jiaoyan=0,acczuoyou_jiaoyan=0,accz_jiaoyan=0,gyroqianhou_jiaoyan=0,gyrozuoyou_jiaoyan=0,gyroz_jiaoyan=0;
	
void MPU6050_WriteReg(u8 reg_add,u8 reg_dat)
{
	I2C_ByteWrite(MPU6050_SLAVE_ADDRESS,reg_dat,reg_add); 
}

/**
  * @brief   从MPU6050寄存器读取数据
  * @param   
  * @retval  
  */
void MPU6050_ReadData(u8 reg_add,unsigned char* Read,u8 num)
{
	I2C_BufferRead(MPU6050_SLAVE_ADDRESS,Read,reg_add,num);
}


/**
  * @brief   初始化MPU6050芯片
  * @param   
  * @retval  
  */
void MPU6050_Init(void)
{
  int i=0,j=0;
	int32_t accqianhou_he=0,acczuoyou_he=0,accz_he=0,gyroqianhou_he=0,gyrozuoyou_he=0,gyroz_he=0;
	short accqianhou=0,acczuoyou=0,accz=0,gyroqianhou=0,gyrozuoyou=0,gyroz=0;
  //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++)
    {
      ;
    }
  }
	MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	     //解除休眠状态
	MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);	    //陀螺仪采样率
	MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	
	MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x01);	  //配置加速度传感器工作在16G模式
	MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
	
	delay_ms(100);

	for(i=1;i<=50;i++)  //测量当前状态，确定校验值
	{
		MPU6050ReadAcc(&accqianhou,&acczuoyou,&accz);
		accqianhou_he+=accqianhou;
		acczuoyou_he+=acczuoyou;
		accz_he+=accz;
		delay_ms(50);
		//printf("%d\n",accz);
	}
	accqianhou_jiaoyan=accqianhou_he/50;
	acczuoyou_jiaoyan=acczuoyou_he/50;
	accz_jiaoyan=accz_he/50;
	
	for(i=1;i<=10;i++)   //测量当前状态，确定校验值
	{	
		MPU6050ReadGyro(&gyroqianhou,&gyrozuoyou,&gyroz);
		gyroqianhou_he+=gyroqianhou;
		gyrozuoyou_he+=gyrozuoyou;
		gyroz_he+=gyroz;
		delay_ms(20);
	}
	gyroqianhou_jiaoyan=gyroqianhou_he/10;
	gyrozuoyou_jiaoyan=gyrozuoyou_he/10;
	gyroz_jiaoyan=gyroz_he/10;
}
/**
  * @brief   读取MPU6050的ID
  * @param   
  * @retval  正常返回1，异常返回0
  */
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
    MPU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //读器件地址
	if(Re != 0x68)
	{
		MPU_ERROR("MPU6050 dectected error!\r\n检测不到MPU6050模块，请检查模块与开发板的接线");
		return 0;
	}
	else
	{
		MPU_INFO("MPU6050 ID = %d\r\n",Re);
		return 1;
	}
		
}

/**
  * @brief   读取MPU6050的加速度数据
  * @param   
  * @retval  
  */
void MPU6050ReadAcc(short *accqianhou,short *acczuoyou,short *accz)
{
    u8 buf[6];
    MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
    *accqianhou = (short)((buf[0] << 8) | buf[1])-accqianhou_jiaoyan;
    *acczuoyou = (short)((buf[2] << 8) | buf[3])-acczuoyou_jiaoyan;
    *accz = (short)((buf[4] << 8) | buf[5])-accz_jiaoyan;
}

/**
  * @brief   读取MPU6050的角加速度数据
  * @param   
  * @retval  
  */
void MPU6050ReadGyro(short *gyroqianhou,short *gyrozuoyou,short *gyroz)
{
    u8 buf[6];
	
    MPU6050_ReadData(MPU6050_GYRO_OUT,buf,6);
    *gyrozuoyou = (short)((buf[0] << 8) | buf[1])-gyrozuoyou_jiaoyan;
    *gyroqianhou = (short)((buf[2] << 8) | buf[3])-gyroqianhou_jiaoyan;
    *gyroz = (short)((buf[4] << 8) | buf[5])-gyroz_jiaoyan;
}

/**
  * @brief   读取MPU6050的原始温度数据
  * @param   
  * @retval  
  */
void MPU6050ReadTemp(short *tempData)
{
	u8 buf[2];
    MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
    *tempData = (buf[0] << 8) | buf[1];
}

/**
  * @brief   读取MPU6050的温度数据，转化成摄氏度
  * @param   
  * @retval  
  */
void MPU6050_ReturnTemp(short*Temperature)
{
	short temp3;
	u8 buf[2];
	
	MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
  temp3= (buf[0] << 8) | buf[1];
	*Temperature=(((double) (temp3 + 13200)) / 280)-13;
}
