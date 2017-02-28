/*
 * Firefighting.c
 *
 * Created: 07-01-2015 12:49:40
 *  Author: simran, sudeep, aditya, vikas
 */ 


# define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.c"
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp1=0,sharp2=0,sharp3=0, distance=0, adc_reading=0;
unsigned int value1=0,value2=0,value3=0;
unsigned char flag = 0;
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

void adc_pin_config (void)
{
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
}

void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 			// removing upper nibbel as it is not needed
	PortARestore = PORTA; 			// reading the PORTA's original status
	PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
	PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
	PORTA = PortARestore; 			// setting the command to the port
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	 timer5_init();
	sei(); //Enables the global interrupts
}


int main(void)
{
   unsigned int value;
   init_devices();
   lcd_set_4bit();
   lcd_init();
   while(1)
      {
	  
	   sharp2 = ADC_Conversion(10);
	   value2 = Sharp_GP2D12_estimation(sharp2);
	   lcd_print(1,2,value2,3);
	   sharp1 = ADC_Conversion(9);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	   value1 = Sharp_GP2D12_estimation(sharp1);				//Stores Distance calsulated in a variable "value".
	   lcd_print(1,6,value1,3);
	   sharp3 = ADC_Conversion(11);
	    value3 = Sharp_GP2D12_estimation(sharp3);
	    lcd_print(1,10,value3,3);
	    flag=0;
		
			
		if((value1< 250) && (value2<250))	
	 {	 						
	   if(sharp1==sharp2) 
	   {   
	   forward();
	   velocity(250,250);
	   }	
	   if(sharp1 > sharp2)	
	   {
	    forward (); 
	    velocity(200,250);
	   }	
	   if(sharp1 < sharp2)
	   {
	   forward (); 
	   velocity(250,200); 
		}	   
	   }	
	   if((value1> 250) || (value2>250))   
	   { 
		  
		  if(sharp1 > sharp2)
		  {
			  left ();
			  velocity(100,250);
		  }
		  if(sharp1 < sharp2)
		  {
			 right();
			  velocity(180,80);
		  }
		  if(sharp1==sharp2)
		  {
			  forward();
			  velocity(250,250);
		  }
	   }  
	   }	      
	   }
	   	    
    
    
