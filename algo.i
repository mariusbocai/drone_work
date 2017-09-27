#line 1 "algo.c"

#line 1 "hal.h"



#line 1 "config.h"




 
 
 







#line 5 "hal.h"
#line 1 "LPC2103.h"
 
 
 
 
 
 
 
 
 
 




 
#line 59 "LPC2103.h"

 



 





 






 




 





 


 



 





 
#line 120 "LPC2103.h"

 
#line 140 "LPC2103.h"

 
#line 156 "LPC2103.h"


 
#line 174 "LPC2103.h"

 
#line 183 "LPC2103.h"

 






 
#line 218 "LPC2103.h"

 
#line 232 "LPC2103.h"

 
#line 241 "LPC2103.h"

 
#line 252 "LPC2103.h"

 
#line 271 "LPC2103.h"

 
#line 290 "LPC2103.h"

 


 


 


 





#line 6 "hal.h"
#line 1 "sensors_i2c.h"




#line 1 "iic_com_driver.h"
 
 
 












#line 24 "iic_com_driver.h"

typedef struct iic_comm_tag
{
   unsigned char IIC_OP;  
   unsigned char IIC_SLAVE_ADDR;  
   unsigned char IIC_SLAVE_REG;  
   unsigned char* IIC_DATA_PNT;  
   unsigned char IIC_MULTI_MODE;  
   unsigned char IIC_RECEIVE_ORDERED; 
} iic_comm_t;

typedef struct iic_com_sm_tag
{
	unsigned char IIC_STATE;
} iic_com_sm_t;

extern iic_comm_t IIC_STRUCT;
extern void IIC_Init(unsigned int desiredBaud);
extern void IIC_COMM_REQ(iic_comm_t* inputStruct);
void ISR_IIC_COMM(void) __irq;

#line 6 "sensors_i2c.h"





 
extern unsigned char GYRO_INIT_VAL[];
extern const iic_comm_t GYRO_INIT_STRUCT;
extern const iic_comm_t GYRO_READ_STRUCT;
extern const iic_comm_t GYRO_INIT_READ_STRUCT;
extern unsigned char GYRO_FIFO_INIT_VAL;
extern const iic_comm_t GYRO_INIT_STRUCT_FIFO;
extern const iic_comm_t ACC_INIT_STRUCT;
extern const iic_comm_t ACC_FIFO_INIT_STRUCT;
extern const iic_comm_t ACC_READ_STRUCT;
extern const iic_comm_t MAG_INIT_STRUCT;
extern const iic_comm_t MAG_READ_STRUCT;
extern unsigned long CLOCK_VAR;

 
typedef struct gyro_input_struct_tag
{
	unsigned char OUT_TEMP;
	unsigned char STATUS_REG;
	unsigned char OUT_X_L;
	unsigned char OUT_X_H;
	unsigned char OUT_Y_L;
	unsigned char OUT_Y_H;
	unsigned char OUT_Z_L;
	unsigned char OUT_Z_H;
} gyro_input_struct_t;

 
typedef struct gyro_angular_data_tag
{
	float GYRO_RAW_X_ANGLE_VEL;
	float GYRO_RAW_Y_ANGLE_VEL;
	float GYRO_RAW_Z_ANGLE_VEL;
	float GYRO_X_ANGLE_TO_EARTH;
	float GYRO_Y_ANGLE_TO_EARTH;
	float GYRO_Z_ANGLE_TO_EARTH;
	unsigned char GYRO_NEW_DATA_AVAILABLE;
} gyro_angular_data_t;

typedef struct acc_input_struct_tag
{
	unsigned char OUT_X_L;
	unsigned char OUT_X_H;
	unsigned char OUT_Y_L;
	unsigned char OUT_Y_H;
	unsigned char OUT_Z_L;
	unsigned char OUT_Z_H;
} acc_input_struct_t;

typedef struct acc_angular_data_tag
{
	float ACC_RAW_X_ANGLE_VEL;
	float ACC_RAW_Y_ANGLE_VEL;
	float ACC_RAW_Z_ANGLE_VEL;
	float ACC_X_ANGLE_TO_EARTH;
	float ACC_Y_ANGLE_TO_EARTH;
	float ACC_Z_ANGLE_TO_EARTH;
	unsigned char ACC_NEW_DATA_AVAILABLE;
} acc_angular_data_t;

typedef struct mag_input_struct_tag
{
	unsigned char OUT_X_H;
	unsigned char OUT_X_L;
	unsigned char OUT_Z_H;
	unsigned char OUT_Z_L;
	unsigned char OUT_Y_H;
	unsigned char OUT_Y_L;
} mag_input_struct_t;

typedef struct	mag_data_struct_tag
{
	float MILIGAUSS_X_AXIS;
	float MILIGAUSS_Y_AXIS;
	float MILIGAUSS_Z_AXIS;
} mag_data_struct_t;

extern void gyroCalculateData(void);
extern void accCalculateData(void);
extern void magCalculateData(void);

#line 7 "hal.h"
#line 1 "dio.h"






extern void reqDioFunction(unsigned char pin_no, unsigned char pin_function);
extern void reqDioDirection(unsigned char pin_no, unsigned char pin_direction);
extern void reqDioOutput(unsigned char pin_no, unsigned char pin_output);
extern unsigned char reqDioInput(unsigned char pin_no, unsigned char pin_input);

#line 8 "hal.h"
#line 1 "spi.h"






typedef struct {
	unsigned char chipSelectPin;
	unsigned char numberOfBytes;
	unsigned char *pTxBuffer;
	unsigned char *pRxBuffer;
} spi_comm_t;

extern void SPI_Init(void);
extern unsigned short SPI_Send(spi_comm_t *dataStruct);

#line 9 "hal.h"
#line 1 "pid.h"



typedef struct
{
	double dState; 
	double iState; 
	double iMax, iMin; 
	double iGain, pGain, dGain; 
} SPid;

extern double actualRollAngle, actualPitchAngle, actualRawYawRate;
extern double rollAngleError, pitchAngleError, yawRateError;
extern double MCU_LOAD;

extern void PID_Init(void);
extern double UpdatePID(SPid * pid, double error, double position);
extern unsigned int calcAndLimitCommand(unsigned int inputThrottle, signed int commDelta);
extern unsigned int limitCommand(signed int commDelta);

#line 10 "hal.h"
#line 1 "output.h"



 




 
extern double rollPIDResult;
extern double pitchPIDResult;
extern double yawPIDResult;

extern SPid rollAxis;
extern SPid pitchAxis;
extern SPid yawAxis;

extern unsigned char selectPidCurrentLoop;

 
extern void PID_Main(void);
extern void resetStateMachine(void);

 
extern char indexChart;



#line 11 "hal.h"
#line 1 "uart.h"



extern void serialInit(void);
extern void serialSendChar(double* value, unsigned char newLine);













#line 12 "hal.h"

 

 
enum {
	E_NOK = 0,	 
	E_OK 		 
};

 






typedef unsigned char ret_val_t;

 
typedef struct {
	float ANGLE_RELATIVE_TO_EARTH_X[5]; 	 
	float ANGLE_RELATIVE_TO_EARTH_Y[5];	 
	float ANGLE_RELATIVE_TO_EARTH_Z[5];	 
} gyro_data_t;

typedef struct {
	float GET_GYRO_RAW_X_ANGLE_VEL[5];
	float GET_GYRO_RAW_Y_ANGLE_VEL[5];
	float GET_GYRO_RAW_Z_ANGLE_VEL[5];
}gyro_raw_data_t;

typedef struct {
	float ACCELERATIO_AXIS_X[5];	 
	float ACCELERATIO_AXIS_Y[5];	 
	float ACCELERATIO_AXIS_Z[5];	 
}acc_data_t;

 
typedef struct {
	unsigned int THROTTLE_STICK;	 
	unsigned char THROTTLE_OK;		 
	unsigned int RUDDER_STICK;
	unsigned char RUDDER_OK;
	unsigned int ROLL_STICK;
	unsigned char ROLL_OK;
	unsigned int PITCH_STICK;
	unsigned char PITCH_OK;
} stick_data_t;

extern stick_data_t INPUT_COMM;
extern unsigned char STATUS_ARMED;
extern unsigned char GyroQueueIndex;
extern unsigned char AccQueueIndex;
extern unsigned char GyroRawQueueIndex;
extern unsigned char STATE;
extern unsigned char LEAVE_STATE;
extern unsigned long CLOCK_VAR;

enum {
	stateInit = 0,
	stateCalibration,
	stateRunning
};



 
enum {
	StartConditionCheck = 0,
	calcStateMachineEnum,
	DesiredAngleModel,
	ActualAngleModel,
	WirelessTelemetry,
	PIDMain,
	LedMain,
	NumberOfFunctioCalls
};

 
extern void checkSafetyTriggerCondition(void);
extern void calcDesiredAngle(void);
extern void calcActualAngle(void);
extern void telemetryMain(void);
extern void calcStateMachine(void);
extern void PID_Main(void);
extern void LED_Main(void);

 
 
 
 
 
 

#line 116 "hal.h"

 
extern ret_val_t HAL_GET_GYRO_DATA_IF(gyro_data_t* Buffer);
extern ret_val_t HAL_GET_ACC_DATA_IF(acc_data_t* Buffer);
extern ret_val_t HAL_GET_GYRO_RAW_DATA_IF(gyro_raw_data_t* Buffer);
																
 
extern void setCommandtoAct(unsigned char channelNumber, unsigned int command);

 
extern void STORE_GYRO_INIT_VALUES(float accXAngle_loc, float accYAngle_loc);

 
extern mag_data_struct_t MAG_DATA_STRUCT;
																
 
extern void LCDClear(void);
extern void LCDBitmap(char my_array[]);
extern void LCDString(char *characters);
extern void LCDInit(void);

 
#line 3 "algo.c"
#line 4 "algo.c"

unsigned char STATUS_ARMED = 0;
unsigned char autoHover;

  float desiredRollAngle; 
  float desiredPitchAngle; 
  float desiredYawRate;
double rollAngleError, pitchAngleError, yawRateError;

const signed short A[5] = {-3, 12, 17, 12, 3};

float smoothAccRollAngle[5];

double actualRollAngle;
double actualPitchAngle;
double actualRawYawRate;

static float accXAngle;
static float accYAngle;

gyro_data_t GYRO_DATA;
acc_data_t ACCELERATION_DATA;
gyro_raw_data_t GET_GYRO_RAW_DATA;

unsigned char STATE;
unsigned char LEAVE_STATE;

void resetStateMachine(void)
{
	accXAngle = 0;
	accYAngle = 0;
	AccQueueIndex = 0;
	GyroRawQueueIndex = 0;
	STATE = stateInit;
}

 
void calcStateMachine(void)
{
	switch (STATE)
	{
		case stateInit:
			if ((LEAVE_STATE)) STATE = stateCalibration;
			break;
		case stateCalibration:
			if ((LEAVE_STATE)) STATE = stateRunning;
			break;
		case stateRunning:
			break;
		default:
			break;
	}
}

 
void checkSafetyTriggerCondition(void)
{
	static unsigned short CNT_ARM = 0;

	if((INPUT_COMM.THROTTLE_OK)&&(INPUT_COMM.RUDDER_OK))
	{
		if((INPUT_COMM.THROTTLE_STICK<100)&&(INPUT_COMM.RUDDER_STICK>700))
				if (STATUS_ARMED == 0)
				{
					CNT_ARM++;
				}
		if(CNT_ARM == 250) 
		{
			 
			STATUS_ARMED = 1;
		}
		if((INPUT_COMM.THROTTLE_STICK<200)&&(INPUT_COMM.RUDDER_STICK<200))
		{
				if (STATUS_ARMED == 1)
				{
					CNT_ARM--;
				}
		}
		if(CNT_ARM == 0)
		{
			STATUS_ARMED = 0;
		}
	}
}

unsigned char isHoveringDesired()
{
	return 1;
}

 
void calcDesiredAngle(void)
{
	 
	if(INPUT_COMM.ROLL_OK)
	{
		desiredRollAngle = 50 - (float)INPUT_COMM.ROLL_STICK/10;		 
	}
	else
	{
		desiredRollAngle = 0;		 
	}
		if(INPUT_COMM.PITCH_OK)
	{
		desiredPitchAngle = 50 - (float)INPUT_COMM.PITCH_STICK/10;		 
	}
	else
	{
		desiredPitchAngle = 0;		 
	}
	if(INPUT_COMM.RUDDER_OK)
	{
		desiredYawRate = 500 - (float)INPUT_COMM.RUDDER_STICK;		 
	}
	else
	{
		desiredYawRate = 0;		 
	}
}

 
void calcActualAngle(void)
{



	static double accXAngle_deg;
	static double accYAngle_deg;
	
	 
	if(STATE == stateInit)
	{
		
		(void)HAL_GET_GYRO_RAW_DATA_IF(&GET_GYRO_RAW_DATA);  
		(void)HAL_GET_ACC_DATA_IF(&ACCELERATION_DATA);




			(LEAVE_STATE) = 1;



	}
	else if(STATE == stateCalibration)
	{
		 
		if(HAL_GET_ACC_DATA_IF(&ACCELERATION_DATA) == E_OK)
		{
#line 168 "algo.c"
			accXAngle = ACCELERATION_DATA.ACCELERATIO_AXIS_X[0];
			accYAngle = ACCELERATION_DATA.ACCELERATIO_AXIS_Y[0];
			actualPitchAngle = (accXAngle * 57.3);  
			actualRollAngle = (accYAngle * 57.3);   
			(LEAVE_STATE) = 1;

		}
	}
	else if(STATE == stateRunning)
	{
		 
		if(HAL_GET_ACC_DATA_IF(&ACCELERATION_DATA) == E_OK)
		{
#line 195 "algo.c"
			accXAngle = ACCELERATION_DATA.ACCELERATIO_AXIS_X[0];
			accYAngle = ACCELERATION_DATA.ACCELERATIO_AXIS_Y[0];
			accXAngle_deg = (accXAngle * 57.3);  
			accYAngle_deg = (accYAngle * 57.3);  

		}
		if(HAL_GET_GYRO_RAW_DATA_IF(&GET_GYRO_RAW_DATA) == E_OK)
		{
#line 209 "algo.c"
			actualPitchAngle = (float)((0.98)*(actualPitchAngle + (float)(GET_GYRO_RAW_DATA.GET_GYRO_RAW_X_ANGLE_VEL[0]/200000)) + (0.02)*accXAngle_deg);
			actualRollAngle = (0.98)*(actualRollAngle + (float)(GET_GYRO_RAW_DATA.GET_GYRO_RAW_Y_ANGLE_VEL[0]/200000)) + (0.02)*accYAngle_deg;
			actualRawYawRate = GET_GYRO_RAW_DATA.GET_GYRO_RAW_Z_ANGLE_VEL[0];

		}
		rollAngleError = desiredRollAngle - actualRollAngle;
		pitchAngleError = desiredPitchAngle - actualPitchAngle;
	  yawRateError = desiredYawRate - actualRawYawRate;
	}
}

 






 
