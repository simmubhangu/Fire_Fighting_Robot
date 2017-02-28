
/*
 * Firefighting.c
 *
 * Created: 07-01-2015 12:49:40
 *  Author: Author: simran, sudeep, Aditya, Vikas
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
unsigned char valuenew = 800;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char Upper_Right_white_line = 0;
unsigned char Upper_Center_white_line = 0;
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
unsigned char servo = 0;//for making servo get to 90 at the center
unsigned char fire=0;
unsigned counter=0;
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}


void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
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

void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

//MOSFET switch port configuration
void MOSFET_switch_config (void)
{
	DDRH = DDRH | 0x0C; //make PORTH 3 and PORTH 1 pins as output
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0

	DDRG = DDRG | 0x04; //make PORTG 2 pin as output
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}

//Function to Initialize PORTS
void port_init()
{
	MOSFET_switch_config();
	lcd_port_config();
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	adc_pin_config();
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
	motion_pin_config();
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}
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

void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

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

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;	//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	velocity(250,250);
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	velocity(250,250);
	angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	velocity(0,250);
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
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

void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
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
void turn_on_sharp234_wl (void) //turn on Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
{
	PORTG = PORTG & 0xFB;
}

void turn_off_sharp234_wl (void) //turn off Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
{
	PORTG = PORTG | 0x04;
}

void turn_on_sharp15 (void) //turn on Sharp IR range sensors 1,5
{
	PORTH = PORTH & 0xFB;
}

void turn_off_sharp15 (void) //turn off Sharp IR range sensors 1,5
{
	PORTH = PORTH | 0x04;
}

void turn_on_ir_proxi_sensors (void) //turn on IR Proximity sensors
{
	PORTH = PORTH & 0xF7;
}

void turn_off_ir_proxi_sensors (void) //turn off IR Proximity sensors
{
	PORTH = PORTH | 0x08;
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
	velocity(0,200);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void) //hard stop
{
	motion_set(0x00);
}

void turn_on_all_proxy_sensors (void) // turn on Sharp 2, 3, 4, red LED of the white line sensors
// Sharp 1, 5 and IR proximity sensor
{
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}

void turn_off_all_proxy_sensors (void) // turn off Sharp 2, 3, 4, red LED of the white line sensors
// Sharp 1, 5 and IR proximity sensor
{
	PORTH = PORTH | 0x0C; //set PORTH 3 and PORTH 1 pins to 1
	PORTG = PORTG | 0x04; //set PORTG 2 pin to 1
}

void line_following(void)
{
 
if((Left_white_line<11)&& (Center_white_line<11)&&(Right_white_line>11)) // far right
{
	
	forward();
	velocity(220,100);
	
}

if((Left_white_line<11)&& (Center_white_line>11)&&(Right_white_line<11)) // center
{
	
	forward();
	velocity(220,220);
	
}

if((Left_white_line>=11)&& (Center_white_line<11)&&(Right_white_line<11)) // far left
{
	
	forward();
	velocity(100,220);
	
	
}

if((Left_white_line>=11)&& (Center_white_line>=11)&&(Right_white_line<11)) // slightly left
{
	
	forward();
	velocity(150,220);
	
}
if((Left_white_line<11)&& (Center_white_line>=11)&&(Right_white_line>=11)) // center
{
	
	forward();
	velocity(220,150);
	
}

 
}

void line_following_entry(void)
{
	
	
		if((Left_white_line<11)&& (Center_white_line<11)&&(Right_white_line>11)) // far right
		{
			
			forward();
			velocity(180,80);
			
		}

		if((Left_white_line<11)&& (Center_white_line>11)&&(Right_white_line<11)) // center
		{
			
			forward();
			velocity(180,180);
			
		}

		if((Left_white_line>=11)&& (Center_white_line<11)&&(Right_white_line<11)) // far left
		{
			
			forward();
			velocity(80,180);
		}
		
	}
	
	
	
	


void wall_following_less_than_250(void)
{
	
  if(value3>200)
  {
	if(sharp1==sharp2)
	{
		forward();
		velocity(250,250);
	}
	if(sharp1 > sharp2)
	{
		forward ();
		velocity(220,250);
	}
	if(sharp1 < sharp2)
	{
		forward ();
		velocity(250,220);
	}  
  }
  if (value3<=200&&value3!=51&&value3!=52&&value3!=50)
  {
	  stop();
	  _delay_ms(300);
	  right_degrees(90);
	  stop();
	  _delay_ms(300);
	  servo_1(0);
	  servo=0;
  }
  
	   } 

void stay1(void)  //for less degree rotation
{
	unsigned char t=0;
	unsigned char s=0;
	stop();
	_delay_ms(300);
	for(t=0;t<10;t++)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		print_sensor(2,1,3);	//Prints value of White Line Sensor1
		print_sensor(2,5,2);	//Prints Value of White Line Sensor2
		print_sensor(2,9,1);	//Prints Value of White Line Sensor3
		//lcd_print(2,13,t,3);
		forward();
		velocity(0,180);//first rotating left for sometime
		if((Center_white_line>=11))
		{
			stop();
		}
		if(t==9)
		{
			break;
		}
	}
	
	
	
	for(s=0;s<10;s++) //this portion to rotate right
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		print_sensor(2,1,3);	//Prints value of White Line Sensor1
		print_sensor(2,5,2);	//Prints Value of White Line Sensor2
		print_sensor(2,9,1);	//Prints Value of White Line Sensor3
		//lcd_print(2,13,s,3);
		forward();
		velocity(180,0);//
		if((Center_white_line>=11))
		{
			stop();
		}
		if(s==9)
		{
			break;
		}
	}


}
int stay2(void)//again stay function with small degree of rotation
{
	unsigned char t=0;
	unsigned char s=0;
	stop();
	_delay_ms(300);
	for(t=0;t<15;t++)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		print_sensor(2,1,3);	//Prints value of White Line Sensor1
		print_sensor(2,5,2);	//Prints Value of White Line Sensor2
		print_sensor(2,9,1);	//Prints Value of White Line Sensor3
		//lcd_print(2,13,t,3);
		forward();
		velocity(0,180);//first rotating left for sometime
		if((Left_white_line>=11)||(Center_white_line>=11)||(Right_white_line>=11))
		{
			stop();
			_delay_ms(300);
			return(1);
		}
		if(t==14)
		{
			break;
		}
	}
	
	
	
	for(s=0;s<30;s++) //this portion to rotate right
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		print_sensor(2,1,3);	//Prints value of White Line Sensor1
		print_sensor(2,5,2);	//Prints Value of White Line Sensor2
		print_sensor(2,9,1);	//Prints Value of White Line Sensor3
		//lcd_print(2,13,s,3);
		forward();
		velocity(180,0);// rotate right
		if((Left_white_line>=11)||(Center_white_line>=11)||(Right_white_line>=11))
		{
			stop();
			_delay_ms(300);
			return(1);
		}
		if(s==29)
		{
			break;
		}
	}


}
 int stay(void)            //this function will be used to detect black line with long degree of rotation
 {   unsigned char t=0;
	 unsigned char s=0;
	 stop();
	 _delay_ms(300);
	 
		for(t=0;t<20;t++)
		{
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
			print_sensor(2,1,3);	//Prints value of White Line Sensor1
			print_sensor(2,5,2);	//Prints Value of White Line Sensor2
			print_sensor(2,9,1);	//Prints Value of White Line Sensor3
			//lcd_print(2,13,t,3);
			forward();
			velocity(0,200);//first rotating left for sometime
			if((Left_white_line>=11)||(Center_white_line>=11)||(Right_white_line>=11))
			{
				stop();
				_delay_ms(300);
				return(1);
			}
			if(t==19)
			{
				break;
			}
		}
		
		
	
		for(s=0;s<20;s++) //this portion to rotate right
		{
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
			print_sensor(2,1,3);	//Prints value of White Line Sensor1
			print_sensor(2,5,2);	//Prints Value of White Line Sensor2
			print_sensor(2,9,1);	//Prints Value of White Line Sensor3
			//lcd_print(2,13,s,3);
			forward();
			velocity(200,0);//
			if((Left_white_line>=11)||(Center_white_line>=11)||(Right_white_line>=11))
			{
				stop();
				_delay_ms(300);
				return(1);
			}
			if(s==19)
			{
				break;
			}
		}
	} 
 
int door(int w) //assigning values according door and it will return value 1  
{
	if( w==2)
	{	stop();
		servo_1(180);
		_delay_ms(300);
		forward_mm(90);
		stop();
		for (i=0;i<2;i++)
		{
			servo_1(i);
			_delay_ms(2);
		}
		_delay_ms(300);
		soft_left_degrees(90);
		stop();
		_delay_ms(300);
		forward_mm(20);
		stop();
		_delay_ms(300);
	}
	if( w==1)
	{	stop();
		servo_1(180);
		_delay_ms(300);
		forward_mm(80);
		stop();
		_delay_ms(300);
		soft_left_degrees(90);
		stop();
		_delay_ms(300);
		forward_mm(100);
		stop();
	}
	return 1;
}

int fire_detection(void)
{ 
	int j=0;
	servo_1(0);
	flag=0;
	if(flag==0)
	{
		for(i=0;i<90;i++)
		{
			servo_1(i);
			_delay_ms(5);
		}
		for(j=0;j<30;j++)
		{
			print_sensor(1,1,14);
			Upper_Right_white_line=ADC_Conversion(14);
			if(j==29)
				{
					flag=1;
					break;
				}
		}
		if(Upper_Right_white_line<20&&j==29)
		{
		return i;
		}
	}	
    
	if(flag==1&&i==90)
	{
		for(i=90;i<180;i++)
		{
			servo_1(i);
			_delay_ms(5);
		}
		for(j=0;j<30;j++)
		{
			print_sensor(1,1,14);
			Upper_Right_white_line=ADC_Conversion(14);
			if(j==29)
			{
				break;
			}
		}		
		if(Upper_Right_white_line<20&&j==29)
		{
			return i;
		}		
	}
	return 0;
}

void far_wala_fire(void)
{
	line_following();
	lcd_print(1,5,counter,2);
	if(((Center_white_line>11&&Right_white_line>11)||(Center_white_line>11&&Left_white_line>11)||(Center_white_line>11&&Left_white_line>11&&Right_white_line>11))&&counter==0)
	{
		forward_mm(70);
		stop();
		_delay_ms(100);
		for(i=0;i<20;i++)
		{
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		}
		if((Center_white_line>11&&Right_white_line>11)||(Center_white_line>11&&Left_white_line>11)||(Center_white_line>11&&Left_white_line>11&&Right_white_line>11))
		{
			Center_white_line=0;
			Left_white_line=0;
			Right_white_line=0;
			stop();
			_delay_ms(400);
			back_mm(50);
			fire=fire_detection();
			if(fire==0)
			{
				forward_mm(20);
				counter=1;
			}
			
		}
		else
		{
			back_mm(120);
			stop();
			_delay_ms(300);
			fire=fire_detection();
			if(fire==0)
			{
				forward_mm(20);
				counter=1;
			}
		}
	}
	if(((Center_white_line>11&&Right_white_line>11)||(Center_white_line>11&&Left_white_line>11)||(Center_white_line>11&&Left_white_line>11&&Right_white_line>11))&&counter==1)
	{
		Center_white_line=0;
		Left_white_line=0;
		Right_white_line=0;
		forward_mm(20);
		stop();
		_delay_ms(300);
		counter=2;
		
	}
	if(((Center_white_line>11&&Right_white_line>11)||(Center_white_line>11&&Left_white_line>11)||(Center_white_line>11&&Left_white_line>11&&Right_white_line>11))&&counter==2)
	{
		Center_white_line=0;
		Left_white_line=0;
		Right_white_line=0;
		forward_mm(70);
		stop();
		_delay_ms(300);
		fire=fire_detection();
		if(fire==180)
		{
			forward_mm(60);
			right_degrees(90);
			counter=3;
		}
		else
		{
			back_mm(70);
			right_degrees(190);
			counter=7;
		}
		
	}
	if(((Center_white_line>11&&Right_white_line>11)||(Center_white_line>11&&Left_white_line>11)||(Center_white_line>11&&Left_white_line>11&&Right_white_line>11))&&counter==3)
	{
		Center_white_line=0;
		Left_white_line=0;
		Right_white_line=0;
		forward_mm(20);
		stop();
		_delay_ms(300);
		counter=4;
		
		
		
	}
	
	if(((Center_white_line>11&&Right_white_line>11)||(Center_white_line>11&&Left_white_line>11)||(Center_white_line>11&&Left_white_line>11&&Right_white_line>11))&&counter==4)
	{
		Center_white_line=0;
		Left_white_line=0;
		Right_white_line=0;
		right_degrees(200);
		stop();
		_delay_ms(300);
		back_mm(20);
		stop();
		_delay_ms(3000);
		counter=5;
	}
	if(((Center_white_line>11&&Right_white_line>11)||(Center_white_line>11&&Left_white_line>11)||(Center_white_line>11&&Left_white_line>11&&Right_white_line>11))&&counter==5)
	{
		
		forward_mm(130);
		stop();
		_delay_ms(300);
		left_degrees(80);
		counter=6;
	}
	if(((Center_white_line>11&&Right_white_line>11)||(Center_white_line>11&&Left_white_line>11)||(Center_white_line>11&&Left_white_line>11&&Right_white_line>11))&&counter==6)
	{
		forward_mm(20);
		stop();
		_delay_ms(300);
		counter=7;
	}
	if(((Center_white_line>11&&Right_white_line>11)||(Center_white_line>11&&Left_white_line>11)||(Center_white_line>11&&Left_white_line>11&&Right_white_line>11))&&counter==7)
	{
		forward_mm(70);
		stop();
		_delay_ms(300);
		if((Center_white_line>11&&Right_white_line>11)||(Center_white_line>11&&Left_white_line>11)||(Center_white_line>11&&Left_white_line>11&&Right_white_line>11))
		{
			Center_white_line=0;
			Left_white_line=0;
			Right_white_line=0;
			forward_mm(20);
			stop();
			_delay_ms(300);
			counter=8;
		}
		else
		{
			counter=8;
		}
	}
	
}
void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	timer1_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei(); //Enables the global interrupts
}

//Main Function
int main(void)
{
	
	init_devices();
	lcd_set_4bit();
	lcd_init();
	unsigned char c = 0;
	unsigned char w = 0;
	unsigned char s = 2;
	unsigned char a = 0;
	unsigned char t = 0;
	unsigned char r = 0;
	
	unsigned char counter = 0;// for counting black box
	unsigned char room = 0;//for
	unsigned char entry=0;//for following line at the entry of room
	//for(i=0;i<2;i++)
	//{
		//servo_1(i);
		//_delay_ms(5);
	//}
	while (1)
	{
	sharp2 = ADC_Conversion(10);
	value2 = Sharp_GP2D12_estimation(sharp2);
	//lcd_print(1,1,value2,3);
	sharp1 = ADC_Conversion(9);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	value1 = Sharp_GP2D12_estimation(sharp1);				//Stores Distance calsulated in a variable "value".
	//lcd_print(1,5,value1,3);
	//lcd_print(1,1,entry,2);
	//lcd_print(1,5,r,2);
	//sharp3 = ADC_Conversion(11);
	//value3 = Sharp_GP2D12_estimation(sharp3);
	//lcd_print(2,13,value3,3);
	//lcd_print(1,13,counter,2);
	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
	//Upper_Right_white_line=ADC_Conversion(14);
	//Upper_Center_white_line=ADC_Conversion(15);
	lcd_print(1,5,counter,2);
	print_sensor(2,1,3);	//Prints value of White Line Sensor1
	print_sensor(2,5,2);	//Prints Value of White Line Sensor2
	print_sensor(2,9,1);	//Prints Value of White Line Sensor3*/
	
	//print_sensor(1,9,14);
	far_wala_fire();
	
   
	
	
	/*if(Upper_Right_white_line > 70)
		{   
				
				turn_off_ir_proxi_sensors();
				if(servo==0)
				{
					for (i=0;i<90;i++)
					{
						servo_1(i);
						_delay_ms(5);
						servo=1;
					}
				}
				if (c==0)
				{	
					if(value1>120)
					{
						s=stay();
						c=1;	
					}
					if(value1<=120)
					{
						stay1();
						s=1;
						c=1;
					}
					
				}
				if(s==1)
				{
					line_following();
				}
				if(((Left_white_line>12 && Center_white_line>12) || (Right_white_line>12 && Center_white_line>12)||( Left_white_line>12 && Center_white_line>12 && Right_white_line>12))&&t==0)
				{
					stop();
					counter=counter+1;
					forward_mm(15);
					line_following();
				}
				if(counter==1)
				{
					forward_mm(50);
					stop();
					for(i=90;i<180;i++)
					{
						servo_1(i);
						_delay_ms(5);
					}
					_delay_ms(300);
					room=0;
					left_degrees(80);
					stop();
					line_following();
					counter=2;
					t=1;
				}
				if(counter==6)
				{
					stop();
					right_degrees(200);
					stop();
					_delay_ms(300);
					line_following();
					counter=7;
				}
				if(counter==9)
				{
					stop();
					_delay_ms(300);
					line_following();
					for(i=0;i<90;i++)
					{
						servo_1(i);
						_delay_ms(5);
					}
					stop();
					forward_mm(200);
					stop();
					_delay_ms(300);
					counter=0;
				}
				
				if(t==1)
					{
						r=stay();
						t=2;
					}
				if(a==1)
				{
					r=stay2();
					entry=1;
					a=2;
				}	
				if(r==1&&entry==1)
				{
					line_following_entry();
					if(((Left_white_line>12 && Center_white_line>12) || (Right_white_line>12 && Center_white_line>12)||( Left_white_line>12 && Center_white_line>12 && Right_white_line>12))&&t==2)
					{
						stop();
						counter=counter+1;
						forward_mm(6);
						entry=0;
						line_following_entry();
					}  
				}
				if(r==1&&entry==0)
				{
					line_following();
					if(((Left_white_line>12 && Center_white_line>12) || (Right_white_line>12 && Center_white_line>12)||( Left_white_line>12 && Center_white_line>12 && Right_white_line>12))&&t==2)
					{
						stop();
						counter=counter+1;
						forward_mm(15);
						line_following();
					}
		
				}
				
		}
		
	if(Upper_Right_white_line < 70 )		
		{
			turn_on_ir_proxi_sensors();
			wall_following_less_than_250();
			
			if(value2>200 && room==0)
			{
				a=door(2);
				w=1;
				room=1;
			}
			if(value1<200 && w==1)
			{
				wall_following_less_than_250();
			}
			
			/*if(value1 > 250 && w==1)
			{
				//wall_following_less_than_250();
				stop();
				forward_mm(50);
				stop();
				_delay_ms(300);
				soft_left_degrees(90);
				stop();
				w=2;
		  }
		}*/
		}}
			
		
		
			
			
	


