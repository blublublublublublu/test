#define MPU_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//自检寄存器Z
#define MPU_SELF_TESTA_REG		0X10	//自检寄存器A
#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_CFG_REG				0X1A	//配置寄存器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG	0X67	//IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG	0X69	//运动检测控制寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器
 
//如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
//如果接V3.3,则IIC地址为0X69(不包含最低位).
#define MPU_ADDR				0X68




//void MPU6050_WriteReg(u8 reg_add,u8 reg_dat)
//{
//    i2c_Start();
//    i2c_SendByte(MPU6050_SLAVE_ADDRESS);    //写从机地址，并配置成写模式
//    i2c_WaitAck();
//    i2c_SendByte(reg_add);     //写寄存器地址
//    i2c_WaitAck();
//    i2c_SendByte(reg_dat);     //写寄存器数据
//    i2c_WaitAck();
//    i2c_Stop();
//}

void MPU6050_WriteReg_kea128(I2C_Type *pI2Cx,uint8 reg_add,uint8 reg_dat)
{
	I2C_Start(pI2Cx);
	I2C_WriteOneByte(pI2Cx, MPU_ADDR);
	delayus(4);
	I2C_WriteOneByte(pI2Cx, reg_add);
	delayus(4);
	I2C_WriteOneByte(pI2Cx, reg_dat);
	delayus(4);
	I2C_Stop(pI2Cx);
	delayus(4);	
} 





//void MPU6050_ReadData(u8 reg_add,unsigned char*Read,u8 num)
//{
//    unsigned char i;
//    
//    i2c_Start();
//    i2c_SendByte(MPU6050_SLAVE_ADDRESS);
//    i2c_WaitAck();
//    i2c_SendByte(reg_add);
//    i2c_WaitAck();
//    
//    i2c_Start();
//    i2c_SendByte(MPU6050_SLAVE_ADDRESS+1);   //写从机地址，并配置成读模式
//    i2c_WaitAck();
//    
//    for(i=0;i<(num-1);i++)
//    {
//        *Read=i2c_ReadByte(1);
//        Read++;
//    }
//    *Read=i2c_ReadByte(0);
//    i2c_Stop();
//}

void MPU6050_ReadData_kea128(I2C_Type *pI2Cx,uint8 reg_add,uint8*Read,uint8 num)
{
	uint8 i;
	I2C_Start(pI2Cx);
	I2C_WriteOneByte(pI2Cx, MPU_ADDR);
	delayus(4);	
	I2C_WriteOneByte(pI2Cx, reg_add);
	delayus(4);	
	
	I2C_RepeatStart(pI2Cx)
	I2C_WriteOneByte(pI2Cx, MPU_ADDR+1);
	delayus(4);	
	
	uint8_t RdBuff;
	for(i=0;i<(num-1);i++)
    {
        I2C_ReadOneByte(pI2Cx, &RdBuff, 1);
		*Read=RdBuff;
        Read++;
    }
    I2C_ReadOneByte(pI2Cx, &RdBuff, 0);
	*Read=RdBuff;
	I2C_Stop(pI2Cx);
	delayus(4);		
}


void MPU6050_Init_kea128(I2C_Type *pI2Cx)
{
  int i=0,j=0;
  //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++)
    {
      ;
    }
  }
    MPU6050_WriteReg_kea128(pI2Cx,MPU_PWR_MGMT1_REG, 0x00);        //解除休眠状态
    MPU6050_WriteReg_kea128(pI2Cx,MPU_SAMPLE_RATE_REG , 0x07);        //陀螺仪采样率，1KHz
    MPU6050_WriteReg_kea128(pI2Cx,MPU_CFG_REG , 0x06);            //低通滤波器的设置，截止频率是1K，带宽是5K
    MPU6050_WriteReg_kea128(pI2Cx,MPU_ACCEL_CFG_REG , 0x00);      //配置加速度传感器工作在2G模式，不自检
    MPU6050_WriteReg_kea128(pI2Cx,MPU_GYRO_CFG_REG, 0x18);     //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
}

/**
  * @brief   读取MPU6050的ID
  * @param   
  * @retval  
  */
//uint8_t MPU6050ReadID(void)
//{
//    unsigned char Re = 0;
//    MPU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //读器件地址
//    if(Re != 0x68)
//    {
//        printf("MPU6050 dectected error!\r\n检测不到MPU6050模块，请检查模块与开发板的接线");
//        return 0;
//    }
//    else
//    {
//        printf("MPU6050 ID = %d\r\n",Re);
//        return 1;
//    }
//        
//}
/**
  * @brief   读取MPU6050的加速度数据
  * @param   
  * @retval  
  */
//#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
//#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
//#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
//#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
//#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
//#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器
void MPU6050ReadAcc_kea128(I2C_Type *pI2Cx,uint16 *accData)
{
    uint8 buf[6];
    MPU6050_ReadData_kea128(pI2Cx,MPU_ACCEL_XOUTH_REG,buf,6);
//    MPU6050_ReadData_kea128(pI2Cx,MPU_ACCEL_XOUTL_REG,&buf[0],1);
//    MPU6050_ReadData_kea128(pI2Cx,MPU_ACCEL_YOUTH_REG,&buf[0],1);
//    MPU6050_ReadData_kea128(pI2Cx,MPU_ACCEL_YOUTL_REG,&buf[0],1);
//    MPU6050_ReadData_kea128(pI2Cx,MPU_ACCEL_ZOUTH_REG,&buf[0],1);
//    MPU6050_ReadData_kea128(pI2Cx,MPU_ACCEL_ZOUTL_REG,&buf[0],1);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

/**
  * @brief   读取MPU6050的角加速度数据
  * @param   
  * @retval  
  */
//#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
//#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
//#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
//#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
//#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
//#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器
void MPU6050ReadGyro_kea128(I2C_Type *pI2Cx,uint16 *gyroData)
{
    uint8 buf[6];
    MPU6050_ReadData_kea128(pI2Cx,MPU_GYRO_XOUTH_REG,buf,6);
//    MPU6050_ReadData_kea128(pI2Cx,MPU_GYRO_XOUTL_REG,&buf[0],1);
//    MPU6050_ReadData_kea128(pI2Cx,MPU_GYRO_YOUTH_REG,&buf[0],1);
//    MPU6050_ReadData_kea128(pI2Cx,MPU_GYRO_YOUTL_REG,&buf[0],1);
//    MPU6050_ReadData_kea128(pI2Cx,MPU_GYRO_ZOUTH_REG,&buf[0],1);
//    MPU6050_ReadData_kea128(pI2Cx,MPU_GYRO_ZOUTL_REG,&buf[0],1);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}


/**
  * @brief   读取MPU6050的原始温度数据
  * @param   
  * @retval  
  */
void MPU6050ReadTemp_kea128(I2C_Type *pI2Cx,uint16 *tempData)
{
    uint8 buf[2];
    MPU6050_ReadData_kea128(pI2Cx,MPU_TEMP_OUTH_REG,buf,2);     //读取温度值
    *tempData = (buf[0] << 8) | buf[1];
}


/**
  * @brief   读取MPU6050的温度数据，转化成摄氏度
  * @param   
  * @retval  
  */
void MPU6050_ReturnTemp_kea128(I2C_Type *pI2Cx,uint16*Temperature)
{
    uint8 temp3;
    uint8 buf[2];
    
    MPU6050_ReadData_kea128(pI2Cx,MPU_TEMP_OUTH_REG,buf,2);    //读取温度值
  	temp3= (buf[0] << 8) | buf[1];
    *Temperature=(((double) (temp3 + 13200)) / 280)-13;
}
typedef struct
{
    I2C_SettingType sSetting;
    uint16_t u16F;              /*!< setting the band rate for I2C */
    uint16_t u16OwnA1;          /*!< slave address */
    uint16_t u16OwnA2;          /*!< the second slave address */
    uint16_t u16RangeA;         /*!< range address */
    uint16_t u16Filt;           /*!< Filter for I2C   */
    uint16_t u16Slt;            /*!< SCL Low timeout register low */
    
}I2C_ConfigType, *I2C_ConfigPtr;

I2C_ConfigPtr pI2CConfig;
pI2CConfig->u16F=9600;
pI2CConfig->u16OwnA1=0X68;
I2C_Init(pI2C1,pI2CConfig);
