#include "LPC2103.H"

void reqDioFunction(unsigned char pin_no, unsigned char pin_function)
{
	if(pin_no <= 15)
	{
		/*Erase former configuration of the pin*/
		PINSEL0 = PINSEL0 & (~ (unsigned int)(0x3<<(pin_no*2)));
		/*Now write the new configuration*/
		PINSEL0 = PINSEL0 | ((unsigned int)pin_function<<(pin_no*2));
	}
	else
	{
		/*Erase former configuration of the pin*/
		PINSEL1 = PINSEL1 & (~ (unsigned int)(0x3<<((pin_no-16)*2)));
		/*Now write the new configuration*/
		PINSEL1 = PINSEL1 | ((unsigned int)pin_function<<((pin_no-16)*2));
	}
}

void reqDioDirection(unsigned char pin_no, unsigned char pin_direction)
{
	/*Mask the pin desired*/
	FIOMASK = ~(unsigned int)(0x1<<(pin_no));
	/*Write the new configuration*/
	if(pin_direction == 1)
	{
		FIODIR = 0;
	}
	else
	{
		FIODIR = 0xFFFFFFFF;
	}
	/*Mask all pins again*/
	FIOMASK = 0xFFFFFFFF;
}

void reqDioOutput(unsigned char pin_no, unsigned char pin_output)
{
	/*Mask the pin desired*/
	FIOMASK = ~(unsigned int)(0x1<<(pin_no));
	/*Write the new configuration*/
	if (pin_output == 1)
	{
		FIOSET = 0xFFFFFFFF;
	}
	else
	{
		FIOCLR = 0xFFFFFFFF;
	}
	/*Mask all pins again*/
	FIOMASK = 0xFFFFFFFF;
}

unsigned char reqDioInput(unsigned char pin_no, unsigned char pin_input)
{
	return 0;
	/*Mask the pin desired*/
	/*Read the pin value*/
	/*Mask all pins again*/
}
