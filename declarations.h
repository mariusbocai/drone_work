#ifndef DECLARATIONS_H_INCLUDED
#define DECLARATIONS_H_INCLUDED

#define Rising 1
#define Falling 2
#define True 1
#define False 0

#define SIM_DEBUG 0

#define No_error 0
#define Error_unnasigned_TCAP_interrupt 1
#define Error_negative_command_time 2
#define Error_too_long_input_pulse 3
#define Error_too_short_input_pulse 4
#define Error_loop_time 5
#define Error_i2c_com_too_long 6

typedef struct
{
	unsigned int currentEdge;
	unsigned int nextEdge;
	unsigned int risingEdgeTime;
	unsigned int fallingEdgeTime;
	unsigned int newDataAvail;
	unsigned int commandUs;
	unsigned int commandPercent;
	unsigned short calibrationDone;
	unsigned short lowRange;					//used for calibration, lowest stick command 
	unsigned short hiRange;					//used for calibration, highes stick command
	unsigned int noDataCounter;
} input_capture_channel_t;

extern input_capture_channel_t channel[4];
extern unsigned int errorCause;
extern unsigned short startLoop;
extern unsigned int startCondCounter;
extern unsigned short startCond;

#endif

