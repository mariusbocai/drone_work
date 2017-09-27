#ifndef HAL_H_
#define HAL_H_

#include "config.h"
#include "LPC2103.h"
#include "sensors_i2c.h"
#include "dio.h"
#include "spi.h"
#include "pid.h"
#include "output.h"
#include "uart.h"

/*===== Hardware Abstraction Layer =======*/

/*===== Acceptable return types for API's =====*/
enum {
	E_NOK = 0,	/* The function returned all the values correct and updated */
	E_OK 		/* Either the values could not be returned because of errors (communication, etc) or the values are not all up to date*/
};

/*===== Motor numbers =====*/
#define Motor1 1
#define Motor2 2
#define Motor3 3
#define Motor4 4
#define AllMotors 99

typedef unsigned char ret_val_t;

/*===== Structure to hold the data =====*/
typedef struct {
	float ANGLE_RELATIVE_TO_EARTH_X[5]; 	/* This angle is the inclination positive or negative with respect to the calibration axis */
	float ANGLE_RELATIVE_TO_EARTH_Y[5];	/* This angle is the inclination positive or negative with respect to the calibration axis */
	float ANGLE_RELATIVE_TO_EARTH_Z[5];	/* This angle is the inclination positive or negative with respect to the calibration axis */
} gyro_data_t;

typedef struct {
	float GET_GYRO_RAW_X_ANGLE_VEL[5];
	float GET_GYRO_RAW_Y_ANGLE_VEL[5];
	float GET_GYRO_RAW_Z_ANGLE_VEL[5];
}gyro_raw_data_t;

typedef struct {
	float ACCELERATIO_AXIS_X[5];	/* This acceleration is measured in G's */
	float ACCELERATIO_AXIS_Y[5];	/* This acceleration is measured in G's */
	float ACCELERATIO_AXIS_Z[5];	/* This acceleration is measured in G's */
}acc_data_t;

/*===== Stick position information =====*/
typedef struct {
	unsigned int THROTTLE_STICK;	/* Stick position information, from 0 to 1000 (=100%) */
	unsigned char THROTTLE_OK;		/* Throttle is active, meaning the a throttle pulse has been received since 40ms */
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

#define okToLeaveState (LEAVE_STATE)

/*===== Add here a symbol for your function =====*/
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

/*===== Functions to be called once each loop =====*/
extern void checkSafetyTriggerCondition(void);
extern void calcDesiredAngle(void);
extern void calcActualAngle(void);
extern void telemetryMain(void);
extern void calcStateMachine(void);
extern void PID_Main(void);
extern void LED_Main(void);

/*===== Array of pointers to functions =====*/
/*===== To call a function each loop, follow the steps below =====*/
/* 1st step: Add a symbol to the enum above, this is to have a correct array size*/
/* 2nd step: Add a "extern" declaration of your function above */
/* 3rd step: Add your function to the array below */
/* IMPORTANT: be very careful where you add your function, because the functions are called in the order from the array */

#define INIT_FUNC_ARRAY \
 void (*functioPointerArray[NumberOfFunctioCalls])() = {\
																checkSafetyTriggerCondition,\
																calcStateMachine,\
																calcDesiredAngle,\
																calcActualAngle,\
																telemetryMain,\
	                              PID_Main,\
																LED_Main,\
																}

/*===== Interface to get the IMU Data =====*/
extern ret_val_t HAL_GET_GYRO_DATA_IF(gyro_data_t* Buffer);
extern ret_val_t HAL_GET_ACC_DATA_IF(acc_data_t* Buffer);
extern ret_val_t HAL_GET_GYRO_RAW_DATA_IF(gyro_raw_data_t* Buffer);
																
/*===== Interfaces to send commands to actuators =====*/
extern void setCommandtoAct(unsigned char channelNumber, unsigned int command);

/*===== Interface to store the Gyro init values =====*/
extern void STORE_GYRO_INIT_VALUES(float accXAngle_loc, float accYAngle_loc);

/*===== Interface to get Magnetomenter data =====*/
extern mag_data_struct_t MAG_DATA_STRUCT;
																
/*===== Interface for LCD panel =====*/
extern void LCDClear(void);
extern void LCDBitmap(char my_array[]);
extern void LCDString(char *characters);
extern void LCDInit(void);

/*===== End of file=====*/
#endif
