#line 1 "output.c"
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

 
#line 2 "output.c"









 
SPid rollAxis;
SPid pitchAxis;
SPid yawAxis;

static double * CHART_OUT_ARRAY[2][3];
char indexChart;



double rollPIDResult, pitchPIDResult, yawPIDResult;
unsigned char selectPidCurrentLoop;

void PID_Main(void)
{
    static signed int commandMot1, commandMot2, commandMot3, commandMot4;
   
    if(CLOCK_VAR == 1)
    {
        commandMot1 = 0;
        commandMot2 = 0;
        commandMot3 = 0;
        commandMot4 = 0;
    }
   
    if((STATUS_ARMED)&& (INPUT_COMM.THROTTLE_STICK >= 100))
    {
			rollPIDResult = UpdatePID(&rollAxis, rollAngleError, actualRollAngle);
			if(rollPIDResult < 0) rollPIDResult = 0;
			pitchPIDResult = UpdatePID(&pitchAxis, pitchAngleError, actualPitchAngle);
			if(pitchPIDResult < 0) pitchPIDResult = 0;
			yawPIDResult = UpdatePID(&yawAxis, yawRateError, actualRawYawRate);
			if(yawPIDResult < 0) yawPIDResult = 0;
			commandMot1 = INPUT_COMM . THROTTLE_STICK + rollPIDResult* +1 + pitchPIDResult* +1 + yawPIDResult* +1; 
			commandMot2 = INPUT_COMM . THROTTLE_STICK + rollPIDResult* -1 + pitchPIDResult* +1 + yawPIDResult* -1; 
			commandMot3 = INPUT_COMM . THROTTLE_STICK + rollPIDResult* -1 + pitchPIDResult* -1 + yawPIDResult* +1;
			commandMot4 = INPUT_COMM . THROTTLE_STICK + rollPIDResult* +1 + pitchPIDResult* -1 + yawPIDResult* -1; 		
			setCommandtoAct(1, limitCommand(commandMot1));
			setCommandtoAct(4, limitCommand(commandMot4));
			setCommandtoAct(2, limitCommand(commandMot2));
			setCommandtoAct(3, limitCommand(commandMot3));
    }
		else
		{  
			PID_Init();
			rollPIDResult = 0;
			pitchPIDResult = 0;
			setCommandtoAct(99, 4000); 
			resetStateMachine();
		}

    {
        if(((CLOCK_VAR%10)==0) && (MCU_LOAD<25))
        {
					if(indexChart < 2)
					{
						CHART_OUT_ARRAY[0][0] = &actualPitchAngle;
						CHART_OUT_ARRAY[0][1] = &actualRollAngle;
						CHART_OUT_ARRAY[0][2] = &actualRawYawRate;
					
						CHART_OUT_ARRAY[1][0] = &pitchPIDResult;
						CHART_OUT_ARRAY[1][1] = &pitchPIDResult;
						CHART_OUT_ARRAY[1][2] = &rollPIDResult;
					
						serialSendChar(CHART_OUT_ARRAY[indexChart][0], 0);
						serialSendChar(CHART_OUT_ARRAY[indexChart][1], 0);
						serialSendChar(CHART_OUT_ARRAY[indexChart][2], 1);
					}
					else
					{
						double cm1 = (double)commandMot1;
						double cm2 = (double)commandMot2;
						double cm3 = (double)commandMot3;
						double cm4 = (double)commandMot4;
					
            serialSendChar(&cm1, 0);
						serialSendChar(&cm2, 0);
						serialSendChar(&cm3, 0);
            serialSendChar(&cm4, 1);
						
            
            
						
						
						
						
						
					}
        }
				else if((MCU_LOAD >= 25))
				{
					 
				}
    }

} 
