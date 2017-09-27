#line 1 "dio.c"
#line 1 "LPC2103.H"
 
 
 
 
 
 
 
 
 
 




 
#line 59 "LPC2103.H"

 



 





 






 




 





 


 



 





 
#line 120 "LPC2103.H"

 
#line 140 "LPC2103.H"

 
#line 156 "LPC2103.H"


 
#line 174 "LPC2103.H"

 
#line 183 "LPC2103.H"

 






 
#line 218 "LPC2103.H"

 
#line 232 "LPC2103.H"

 
#line 241 "LPC2103.H"

 
#line 252 "LPC2103.H"

 
#line 271 "LPC2103.H"

 
#line 290 "LPC2103.H"

 


 


 


 





#line 2 "dio.c"

void reqDioFunction(unsigned char pin_no, unsigned char pin_function)
{
	if(pin_no <= 15)
	{
		 
		(*((volatile unsigned long *) 0xE002C000)) = (*((volatile unsigned long *) 0xE002C000)) & (~ (unsigned int)(0x3<<(pin_no*2)));
		 
		(*((volatile unsigned long *) 0xE002C000)) = (*((volatile unsigned long *) 0xE002C000)) | ((unsigned int)pin_function<<(pin_no*2));
	}
	else
	{
		 
		(*((volatile unsigned long *) 0xE002C004)) = (*((volatile unsigned long *) 0xE002C004)) & (~ (unsigned int)(0x3<<((pin_no-16)*2)));
		 
		(*((volatile unsigned long *) 0xE002C004)) = (*((volatile unsigned long *) 0xE002C004)) | ((unsigned int)pin_function<<((pin_no-16)*2));
	}
}

void reqDioDirection(unsigned char pin_no, unsigned char pin_direction)
{
	 
	(*((volatile unsigned long *) 0x3FFFC010)) = ~(unsigned int)(0x1<<(pin_no));
	 
	if(pin_direction == 1)
	{
		(*((volatile unsigned long *) 0x3FFFC000)) = 0;
	}
	else
	{
		(*((volatile unsigned long *) 0x3FFFC000)) = 0xFFFFFFFF;
	}
	 
	(*((volatile unsigned long *) 0x3FFFC010)) = 0xFFFFFFFF;
}

void reqDioOutput(unsigned char pin_no, unsigned char pin_output)
{
	 
	(*((volatile unsigned long *) 0x3FFFC010)) = ~(unsigned int)(0x1<<(pin_no));
	 
	if (pin_output == 1)
	{
		(*((volatile unsigned long *) 0x3FFFC018)) = 0xFFFFFFFF;
	}
	else
	{
		(*((volatile unsigned long *) 0x3FFFC01C)) = 0xFFFFFFFF;
	}
	 
	(*((volatile unsigned long *) 0x3FFFC010)) = 0xFFFFFFFF;
}

unsigned char reqDioInput(unsigned char pin_no, unsigned char pin_input)
{
	return 0;
	 
	 
	 
}
