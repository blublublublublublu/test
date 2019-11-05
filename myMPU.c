#define MPU_SELF_TESTX_REG		0X0D	//�Լ�Ĵ���X
#define MPU_SELF_TESTY_REG		0X0E	//�Լ�Ĵ���Y
#define MPU_SELF_TESTZ_REG		0X0F	//�Լ�Ĵ���Z
#define MPU_SELF_TESTA_REG		0X10	//�Լ�Ĵ���A
#define MPU_SAMPLE_RATE_REG		0X19	//����Ƶ�ʷ�Ƶ��
#define MPU_CFG_REG				0X1A	//���üĴ���
#define MPU_GYRO_CFG_REG		0X1B	//���������üĴ���
#define MPU_ACCEL_CFG_REG		0X1C	//���ٶȼ����üĴ���
#define MPU_MOTION_DET_REG		0X1F	//�˶���ֵⷧ���üĴ���
#define MPU_FIFO_EN_REG			0X23	//FIFOʹ�ܼĴ���
#define MPU_I2CMST_CTRL_REG		0X24	//IIC�������ƼĴ���
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC�ӻ�0������ַ�Ĵ���
#define MPU_I2CSLV0_REG			0X26	//IIC�ӻ�0���ݵ�ַ�Ĵ���
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC�ӻ�0���ƼĴ���
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC�ӻ�1������ַ�Ĵ���
#define MPU_I2CSLV1_REG			0X29	//IIC�ӻ�1���ݵ�ַ�Ĵ���
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC�ӻ�1���ƼĴ���
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC�ӻ�2������ַ�Ĵ���
#define MPU_I2CSLV2_REG			0X2C	//IIC�ӻ�2���ݵ�ַ�Ĵ���
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC�ӻ�2���ƼĴ���
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC�ӻ�3������ַ�Ĵ���
#define MPU_I2CSLV3_REG			0X2F	//IIC�ӻ�3���ݵ�ַ�Ĵ���
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC�ӻ�3���ƼĴ���
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC�ӻ�4������ַ�Ĵ���
#define MPU_I2CSLV4_REG			0X32	//IIC�ӻ�4���ݵ�ַ�Ĵ���
#define MPU_I2CSLV4_DO_REG		0X33	//IIC�ӻ�4д���ݼĴ���
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC�ӻ�4���ƼĴ���
#define MPU_I2CSLV4_DI_REG		0X35	//IIC�ӻ�4�����ݼĴ���

#define MPU_I2CMST_STA_REG		0X36	//IIC����״̬�Ĵ���
#define MPU_INTBP_CFG_REG		0X37	//�ж�/��·���üĴ���
#define MPU_INT_EN_REG			0X38	//�ж�ʹ�ܼĴ���
#define MPU_INT_STA_REG			0X3A	//�ж�״̬�Ĵ���

#define MPU_ACCEL_XOUTH_REG		0X3B	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_XOUTL_REG		0X3C	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_YOUTH_REG		0X3D	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_YOUTL_REG		0X3E	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_ZOUTH_REG		0X3F	//���ٶ�ֵ,Z���8λ�Ĵ���
#define MPU_ACCEL_ZOUTL_REG		0X40	//���ٶ�ֵ,Z���8λ�Ĵ���

#define MPU_TEMP_OUTH_REG		0X41	//�¶�ֵ�߰�λ�Ĵ���
#define MPU_TEMP_OUTL_REG		0X42	//�¶�ֵ��8λ�Ĵ���

#define MPU_GYRO_XOUTH_REG		0X43	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_XOUTL_REG		0X44	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_YOUTH_REG		0X45	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_YOUTL_REG		0X46	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_ZOUTH_REG		0X47	//������ֵ,Z���8λ�Ĵ���
#define MPU_GYRO_ZOUTL_REG		0X48	//������ֵ,Z���8λ�Ĵ���

#define MPU_I2CSLV0_DO_REG		0X63	//IIC�ӻ�0���ݼĴ���
#define MPU_I2CSLV1_DO_REG		0X64	//IIC�ӻ�1���ݼĴ���
#define MPU_I2CSLV2_DO_REG		0X65	//IIC�ӻ�2���ݼĴ���
#define MPU_I2CSLV3_DO_REG		0X66	//IIC�ӻ�3���ݼĴ���

#define MPU_I2CMST_DELAY_REG	0X67	//IIC������ʱ����Ĵ���
#define MPU_SIGPATH_RST_REG		0X68	//�ź�ͨ����λ�Ĵ���
#define MPU_MDETECT_CTRL_REG	0X69	//�˶������ƼĴ���
#define MPU_USER_CTRL_REG		0X6A	//�û����ƼĴ���
#define MPU_PWR_MGMT1_REG		0X6B	//��Դ����Ĵ���1
#define MPU_PWR_MGMT2_REG		0X6C	//��Դ����Ĵ���2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO�����Ĵ����߰�λ
#define MPU_FIFO_CNTL_REG		0X73	//FIFO�����Ĵ����Ͱ�λ
#define MPU_FIFO_RW_REG			0X74	//FIFO��д�Ĵ���
#define MPU_DEVICE_ID_REG		0X75	//����ID�Ĵ���
 
//���AD0��(9��)�ӵ�,IIC��ַΪ0X68(���������λ).
//�����V3.3,��IIC��ַΪ0X69(���������λ).
#define MPU_ADDR				0X68




//void MPU6050_WriteReg(u8 reg_add,u8 reg_dat)
//{
//    i2c_Start();
//    i2c_SendByte(MPU6050_SLAVE_ADDRESS);    //д�ӻ���ַ�������ó�дģʽ
//    i2c_WaitAck();
//    i2c_SendByte(reg_add);     //д�Ĵ�����ַ
//    i2c_WaitAck();
//    i2c_SendByte(reg_dat);     //д�Ĵ�������
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
//    i2c_SendByte(MPU6050_SLAVE_ADDRESS+1);   //д�ӻ���ַ�������óɶ�ģʽ
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
  //�ڳ�ʼ��֮ǰҪ��ʱһ��ʱ�䣬��û����ʱ����ϵ�����ϵ����ݿ��ܻ����
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++)
    {
      ;
    }
  }
    MPU6050_WriteReg_kea128(pI2Cx,MPU_PWR_MGMT1_REG, 0x00);        //�������״̬
    MPU6050_WriteReg_kea128(pI2Cx,MPU_SAMPLE_RATE_REG , 0x07);        //�����ǲ����ʣ�1KHz
    MPU6050_WriteReg_kea128(pI2Cx,MPU_CFG_REG , 0x06);            //��ͨ�˲��������ã���ֹƵ����1K��������5K
    MPU6050_WriteReg_kea128(pI2Cx,MPU_ACCEL_CFG_REG , 0x00);      //���ü��ٶȴ�����������2Gģʽ�����Լ�
    MPU6050_WriteReg_kea128(pI2Cx,MPU_GYRO_CFG_REG, 0x18);     //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
}

/**
  * @brief   ��ȡMPU6050��ID
  * @param   
  * @retval  
  */
//uint8_t MPU6050ReadID(void)
//{
//    unsigned char Re = 0;
//    MPU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //��������ַ
//    if(Re != 0x68)
//    {
//        printf("MPU6050 dectected error!\r\n��ⲻ��MPU6050ģ�飬����ģ���뿪����Ľ���");
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
  * @brief   ��ȡMPU6050�ļ��ٶ�����
  * @param   
  * @retval  
  */
//#define MPU_ACCEL_XOUTH_REG		0X3B	//���ٶ�ֵ,X���8λ�Ĵ���
//#define MPU_ACCEL_XOUTL_REG		0X3C	//���ٶ�ֵ,X���8λ�Ĵ���
//#define MPU_ACCEL_YOUTH_REG		0X3D	//���ٶ�ֵ,Y���8λ�Ĵ���
//#define MPU_ACCEL_YOUTL_REG		0X3E	//���ٶ�ֵ,Y���8λ�Ĵ���
//#define MPU_ACCEL_ZOUTH_REG		0X3F	//���ٶ�ֵ,Z���8λ�Ĵ���
//#define MPU_ACCEL_ZOUTL_REG		0X40	//���ٶ�ֵ,Z���8λ�Ĵ���
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
  * @brief   ��ȡMPU6050�ĽǼ��ٶ�����
  * @param   
  * @retval  
  */
//#define MPU_GYRO_XOUTH_REG		0X43	//������ֵ,X���8λ�Ĵ���
//#define MPU_GYRO_XOUTL_REG		0X44	//������ֵ,X���8λ�Ĵ���
//#define MPU_GYRO_YOUTH_REG		0X45	//������ֵ,Y���8λ�Ĵ���
//#define MPU_GYRO_YOUTL_REG		0X46	//������ֵ,Y���8λ�Ĵ���
//#define MPU_GYRO_ZOUTH_REG		0X47	//������ֵ,Z���8λ�Ĵ���
//#define MPU_GYRO_ZOUTL_REG		0X48	//������ֵ,Z���8λ�Ĵ���
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
  * @brief   ��ȡMPU6050��ԭʼ�¶�����
  * @param   
  * @retval  
  */
void MPU6050ReadTemp_kea128(I2C_Type *pI2Cx,uint16 *tempData)
{
    uint8 buf[2];
    MPU6050_ReadData_kea128(pI2Cx,MPU_TEMP_OUTH_REG,buf,2);     //��ȡ�¶�ֵ
    *tempData = (buf[0] << 8) | buf[1];
}


/**
  * @brief   ��ȡMPU6050���¶����ݣ�ת�������϶�
  * @param   
  * @retval  
  */
void MPU6050_ReturnTemp_kea128(I2C_Type *pI2Cx,uint16*Temperature)
{
    uint8 temp3;
    uint8 buf[2];
    
    MPU6050_ReadData_kea128(pI2Cx,MPU_TEMP_OUTH_REG,buf,2);    //��ȡ�¶�ֵ
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
