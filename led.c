#include "hal.h"

unsigned char cycleInProgress = 0;
unsigned char ledOnLoopCounter;
unsigned char ledOffLoopCounter;

void ledCycleCommand(unsigned char on, unsigned char off)
{
	if(cycleInProgress == 0)
	{
		cycleInProgress = 1;
		ledOnLoopCounter = on;
		ledOffLoopCounter = off;
	}
	if(ledOnLoopCounter--)
	{
		reqDioOutput(23,1);
	}
	else
	{
		if(ledOffLoopCounter--) 
		{
			reqDioOutput(23,0);
			cycleInProgress	= 0;
		}
	}
}

void LED_Main()
{
	if(STATUS_ARMED)
	{
		reqDioOutput(23,1);
	}
	else
	{
		/*if(cycleInProgress == 0)*/ ledCycleCommand(100,100);
	}
		
}
