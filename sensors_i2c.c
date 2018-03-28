#include "config.h"
#include "iic_com_driver.h"
#include "sensors_i2c.h"
#include "hal.h"
#include "uart.h"

/*=== Structure where to put the read data from the gyro ===*/
gyro_input_struct_t GYRO_RAW_INPUT_STRUCT;
acc_input_struct_t ACC_RAW_INPUT_STRUCT;
mag_input_struct_t MAG_RAW_INPUT_STRUCT;
unsigned char GYRO_INIT_INPUT_STRUCT[5];

unsigned char GyroQueueIndex;
unsigned char GyroRawQueueIndex;
unsigned char AccQueueIndex;

#if(DEBUG_GYRO_DATA)
/*********TEST ONLY******/
float saved_gyro_rate_x[200];
float saved_absolute_val[200];
unsigned char status_reg[200];
unsigned long saved_clock[200];
#endif

/*===============================================================*/
/*================== GYRO =======================================*/
/*===============================================================*/
/*=== Test the I2C communication ===*/
unsigned char GYRO_COM_TEST_VAL = 0x0;
const iic_comm_t GYRO_COM_TEST_STRUCT = {MasterReceive, 0x69, 0x0F, &GYRO_COM_TEST_VAL ,1, 0};

/*=== Initialization struct of the Gyroscope ===*/
unsigned char GYRO_INIT_VAL[] = {0x4F, 0x21, 0x0, 0x0, 0x0};//0x53
const iic_comm_t GYRO_INIT_STRUCT = {MasterTransmit, 0x69, 0xA0, GYRO_INIT_VAL ,5, 0};

unsigned char GYRO_FIFO_INIT_VAL = 0x0;	 //0x40
const iic_comm_t GYRO_INIT_STRUCT_FIFO = {MasterTransmit, 0x69, 0xAE, &GYRO_FIFO_INIT_VAL ,1, 0};

/*=== Struct to read the data from the Gyro ===*/
const iic_comm_t GYRO_READ_STRUCT = {MasterReceive, 0x69, 0xA6, (unsigned char*)&GYRO_RAW_INPUT_STRUCT, 8, 0};
const iic_comm_t GYRO_INIT_READ_STRUCT = {MasterReceive, 0x69, 0xA0, (unsigned char*)&GYRO_INIT_INPUT_STRUCT, 5, 0};

/*=== Struct where to put the transformed data from Gyro ===*/
gyro_angular_data_t GYRO_DATA_STRUCT;

/*===============================================================*/
/*================== ACCELERATION================================*/
/*===============================================================*/
/*=== Test the I2C communication ===*/
unsigned char ACC_COM_TEST_VAL = 0x0;
const iic_comm_t ACC_COM_TEST_STRUCT = {MasterReceive, 0x53, 0x0, &ACC_COM_TEST_VAL ,1, 0};

//unsigned char ACC_INIT_VAL[] = {0xB, 0x8, 0x0};
unsigned char ACC_INIT_VAL = 0x8;
const iic_comm_t ACC_INIT_STRUCT = {MasterTransmit, 0x53, 0x2D, &ACC_INIT_VAL ,1, 0};

unsigned char ACC_FIFO_INIT_VAL = 0xC0;
const iic_comm_t ACC_FIFO_INIT_STRUCT = {MasterTransmit, 0x53, 0x38, &ACC_FIFO_INIT_VAL ,1, 0};

const iic_comm_t ACC_READ_STRUCT = {MasterReceive, 0x53, 0x32, (unsigned char*)&ACC_RAW_INPUT_STRUCT, 6, 0};

acc_angular_data_t ACC_DATA_STRUCT;

/*===============================================================*/
/*================== MAGNETOMETER================================*/
/*===============================================================*/
unsigned char MAG_INIT_VAL[] = {0x0};
const iic_comm_t MAG_INIT_STRUCT = {MasterTransmit, 0x1E, 0x02, MAG_INIT_VAL ,1, 0};
const iic_comm_t MAG_READ_STRUCT = {MasterReceive, 0x1E, 0x3, (unsigned char*)&MAG_RAW_INPUT_STRUCT, 6, 0};

mag_data_struct_t MAG_DATA_STRUCT;

/*===============================================================*/
/*================== BAROMETER===================================*/
/*===============================================================*/
unsigned char BARO_INIT_VAL[] = {0x0, 0x0, 0x0, 0x0, 0x0};
const iic_comm_t BARO_INIT_STRUCT = {MasterTransmit, 0x69, 0xA0, BARO_INIT_VAL ,5, 0};

/*=== Function to calculate the angular velocity and the angle of deviation from gyro raw data ===*/
void gyroCalculateData()
{
	signed short tempVar;
#if(DEBUG_GYRO_DATA)
	static unsigned char INdex;
#endif

	if(CLOCK_VAR == 1)
	{
#if(DEBUG_GYRO_DATA)
		unsigned char i;
#endif
		GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL = 0;
		GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH = 0;
		GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL = 0;
		GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH = 0;
		GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL = 0;
		GYRO_DATA_STRUCT.GYRO_Z_ANGLE_TO_EARTH = 0;
#if(DEBUG_GYRO_DATA)
		for(i=0;i<200;i++)
		{
			saved_gyro_rate_x[i]=0;
			status_reg[i]=0;
			saved_absolute_val[i]=0;
			saved_clock[i] = 0;
		}
		INdex = 0;
#endif
	}
#if(DEBUG_GYRO_DATA)
	if(INdex == 200) 
	{
		INdex=0;
	}
#endif

	/*=== If new X axis data is available, calculate the raw angular velocity ===*/
	if(GYRO_RAW_INPUT_STRUCT.STATUS_REG & GyroMaskXdataAvail)
	{
		tempVar = (signed short)((GYRO_RAW_INPUT_STRUCT.OUT_X_H<<8) | (GYRO_RAW_INPUT_STRUCT.OUT_X_L));
		GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL = (float)(((tempVar)*9)); //mili degrees per second
		GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH += (GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL/200000);
#if(DEBUG_GYRO_DATA)
		saved_gyro_rate_x[INdex++] = GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL;
		saved_absolute_val[INdex] = GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH;
		saved_clock[INdex] = CLOCK_VAR;
		status_reg[INdex] = GYRO_RAW_INPUT_STRUCT.STATUS_REG;
#endif
	}
	if(GYRO_RAW_INPUT_STRUCT.STATUS_REG & GyroMaskYdataAvail)
	{
		tempVar = (signed short)((GYRO_RAW_INPUT_STRUCT.OUT_Y_H<<8) | (GYRO_RAW_INPUT_STRUCT.OUT_Y_L));
		GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL = (float)(((tempVar)*9));
		GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH += GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL/200000;
	}
	if(GYRO_RAW_INPUT_STRUCT.STATUS_REG & GyroMaskZdataAvail)
	{
		tempVar = (signed short)((GYRO_RAW_INPUT_STRUCT.OUT_Z_H<<8) | (GYRO_RAW_INPUT_STRUCT.OUT_Z_L));
		GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL = (float)(((tempVar)*9));
		GYRO_DATA_STRUCT.GYRO_Z_ANGLE_TO_EARTH += GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL/200000;
	}
}

void accCalculateData()
{
	signed short tempVar;
	
	tempVar = (signed short)((ACC_RAW_INPUT_STRUCT.OUT_X_H<<8) | (ACC_RAW_INPUT_STRUCT.OUT_X_L));
	ACC_DATA_STRUCT.ACC_RAW_X_ANGLE_VEL = (double)((((double)tempVar)/256)); /*g*/
	
	tempVar = (signed short)((ACC_RAW_INPUT_STRUCT.OUT_Y_H<<8) | (ACC_RAW_INPUT_STRUCT.OUT_Y_L));
	ACC_DATA_STRUCT.ACC_RAW_Y_ANGLE_VEL = (float)((((float)tempVar)/256)); /*g*/
	
	tempVar = (signed short)((ACC_RAW_INPUT_STRUCT.OUT_Z_H<<8) | (ACC_RAW_INPUT_STRUCT.OUT_Z_L));
	ACC_DATA_STRUCT.ACC_RAW_Z_ANGLE_VEL = (float)((((float)tempVar)/256)); /*g*/	
}

void magCalculateData()
{
	signed short tempVar;
	tempVar = (signed short)((MAG_RAW_INPUT_STRUCT.OUT_X_H<<8)|(MAG_RAW_INPUT_STRUCT.OUT_X_L));
	MAG_DATA_STRUCT.MILIGAUSS_X_AXIS = (float)(((float)tempVar)*0.92);

	tempVar = (signed short)((MAG_RAW_INPUT_STRUCT.OUT_Y_H<<8)|(MAG_RAW_INPUT_STRUCT.OUT_Y_L));
	MAG_DATA_STRUCT.MILIGAUSS_Y_AXIS = (float)(((float)tempVar)*0.92);

	tempVar = (signed short)((MAG_RAW_INPUT_STRUCT.OUT_Z_H<<8)|(MAG_RAW_INPUT_STRUCT.OUT_Z_L));
	MAG_DATA_STRUCT.MILIGAUSS_Z_AXIS = (float)(((float)tempVar)*0.92);
}

/*===== Interface to store the Gyro init values =====*/
void STORE_GYRO_INIT_VALUES(float accXAngle_loc, float accYAngle_loc)
{
	GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH = accXAngle_loc;
	GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH = accYAngle_loc;
}

/*For this function, I am not sure I need to store the last 5 samples; but could be useful in the future!*/
ret_val_t HAL_GET_GYRO_RAW_DATA_IF(gyro_raw_data_t* Buffer)
{
	ret_val_t retValue;
#if (USE_SMOOTHING_FILTER == 1)
	unsigned char x;
	if(GyroRawQueueIndex == 5)
	{
		for(x = 0; x < 4; x++)
		{
			Buffer->GET_GYRO_RAW_X_ANGLE_VEL[x] = Buffer->GET_GYRO_RAW_X_ANGLE_VEL[x+1];
			Buffer->GET_GYRO_RAW_Y_ANGLE_VEL[x] = Buffer->GET_GYRO_RAW_Y_ANGLE_VEL[x+1];
			Buffer->GET_GYRO_RAW_Z_ANGLE_VEL[x] = Buffer->GET_GYRO_RAW_Z_ANGLE_VEL[x+1];
		}
		Buffer->GET_GYRO_RAW_X_ANGLE_VEL[4] = GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL;
		Buffer->GET_GYRO_RAW_Y_ANGLE_VEL[4] = GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL;
		Buffer->GET_GYRO_RAW_Z_ANGLE_VEL[4] = GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL;
	}
	else
	{
		Buffer->GET_GYRO_RAW_X_ANGLE_VEL[GyroRawQueueIndex] = GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL;
		Buffer->GET_GYRO_RAW_Y_ANGLE_VEL[GyroRawQueueIndex] = GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL;
		Buffer->GET_GYRO_RAW_Z_ANGLE_VEL[GyroRawQueueIndex] = GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL;
		GyroRawQueueIndex++;
	}
#else
	Buffer->GET_GYRO_RAW_X_ANGLE_VEL[0] = GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL;
	Buffer->GET_GYRO_RAW_Y_ANGLE_VEL[0] = GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL;
	Buffer->GET_GYRO_RAW_Z_ANGLE_VEL[0] = GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL;
#endif
	retValue = E_OK;
	return retValue;
}

/*===== API te read the angles provided by the GYRO=====*/
ret_val_t HAL_GET_GYRO_DATA_IF(gyro_data_t* Buffer)
{
	ret_val_t retValue;
#if (USE_SMOOTHING_FILTER == 1)
	unsigned char x;
#endif
	if((GYRO_RAW_INPUT_STRUCT.STATUS_REG & GyroMaskXdataAvail) && (GYRO_RAW_INPUT_STRUCT.STATUS_REG & GyroMaskYdataAvail) && (GYRO_RAW_INPUT_STRUCT.STATUS_REG & GyroMaskZdataAvail) )
	{
#if (USE_SMOOTHING_FILTER == 1)
		if(GyroQueueIndex == 5)
		{
			for(x = 0; x < 4; x++)
			{
				Buffer->ANGLE_RELATIVE_TO_EARTH_X[x] = Buffer->ANGLE_RELATIVE_TO_EARTH_X[x+1];
				Buffer->ANGLE_RELATIVE_TO_EARTH_Y[x] = Buffer->ANGLE_RELATIVE_TO_EARTH_Y[x+1];
				Buffer->ANGLE_RELATIVE_TO_EARTH_Z[x] = Buffer->ANGLE_RELATIVE_TO_EARTH_Z[x+1];
			}
			Buffer->ANGLE_RELATIVE_TO_EARTH_X[4] = GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH;
			Buffer->ANGLE_RELATIVE_TO_EARTH_Y[4] = GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH;
			Buffer->ANGLE_RELATIVE_TO_EARTH_Z[4] = GYRO_DATA_STRUCT.GYRO_Z_ANGLE_TO_EARTH;
		}
		else
		{
			Buffer->ANGLE_RELATIVE_TO_EARTH_X[GyroQueueIndex] = GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH;
			Buffer->ANGLE_RELATIVE_TO_EARTH_Y[GyroQueueIndex] = GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH;
			Buffer->ANGLE_RELATIVE_TO_EARTH_Z[GyroQueueIndex] = GYRO_DATA_STRUCT.GYRO_Z_ANGLE_TO_EARTH;
			GyroQueueIndex++;
		}
		retValue = E_OK;
#else
		Buffer->ANGLE_RELATIVE_TO_EARTH_X[0] = GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH;
		Buffer->ANGLE_RELATIVE_TO_EARTH_Y[0] = GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH;
		Buffer->ANGLE_RELATIVE_TO_EARTH_Z[0] = GYRO_DATA_STRUCT.GYRO_Z_ANGLE_TO_EARTH;
#endif
	}
	else retValue = E_NOK;
	return retValue;
}

ret_val_t HAL_GET_ACC_DATA_IF(acc_data_t* Buffer)
{
#if (USE_SMOOTHING_FILTER == 1)
	unsigned char x;
	if(AccQueueIndex == 5)
		{
			for(x = 0; x < 4; x++)
			{
				Buffer->ACCELERATIO_AXIS_X[x] = Buffer->ACCELERATIO_AXIS_X[x+1];
				Buffer->ACCELERATIO_AXIS_Y[x] = Buffer->ACCELERATIO_AXIS_Y[x+1];
				Buffer->ACCELERATIO_AXIS_Z[x] = Buffer->ACCELERATIO_AXIS_Z[x+1];
			}
			Buffer->ACCELERATIO_AXIS_X[4] = ACC_DATA_STRUCT.ACC_RAW_X_ANGLE_VEL;
			Buffer->ACCELERATIO_AXIS_Y[4] = ACC_DATA_STRUCT.ACC_RAW_Y_ANGLE_VEL;
			Buffer->ACCELERATIO_AXIS_Z[4] = ACC_DATA_STRUCT.ACC_RAW_Z_ANGLE_VEL;
		}
		else
		{
			Buffer->ACCELERATIO_AXIS_X[AccQueueIndex] = ACC_DATA_STRUCT.ACC_RAW_X_ANGLE_VEL;
			Buffer->ACCELERATIO_AXIS_Y[AccQueueIndex] = ACC_DATA_STRUCT.ACC_RAW_Y_ANGLE_VEL;
			Buffer->ACCELERATIO_AXIS_Z[AccQueueIndex] = ACC_DATA_STRUCT.ACC_RAW_Z_ANGLE_VEL;
			AccQueueIndex++;
		}
#else
		Buffer->ACCELERATIO_AXIS_X[0] = ACC_DATA_STRUCT.ACC_RAW_X_ANGLE_VEL;
		Buffer->ACCELERATIO_AXIS_Y[0] = ACC_DATA_STRUCT.ACC_RAW_Y_ANGLE_VEL;
		Buffer->ACCELERATIO_AXIS_Z[0] = ACC_DATA_STRUCT.ACC_RAW_Z_ANGLE_VEL;
#endif
	return E_OK;
}

