/*
=============================================================================
MCU: Arduino Mega 2560 (ATmega 2560)
=============================================================================
Functions:
1. Serial communication at 4800 baud rate
2. Timer 4 outputs two pins' pwm waves
3. Robot can move forward and backward, turn left while moving forward,
   turn right while moving forward, turn left while standing at the same
   position,turn right while standing at the same position
4. The robot's moving speed can be adjusted via pwm waves
5. Measure a motor's speed via INT 1,2 and Timer 1
   The speed value is stored in variable "speed"
6. Detect a motor's direction via INT 1,2
  if the motor rotates forward, then the serial port will output: +
  if the motor rotates backward,then the serial port will output: -
7. Real-time measurement of the battery voltage whose value is
   stored in variable "batteryvoltage"
8. Robot will be stopped if the position switch is triggered.
9. Motors are controlled wirelessly using radio controller with channel 1,2,4
   Channel 1: turn left or turn right while moving forward
   Channel 2: move forward or backward
   Channel 4: turn left or turn right while standing at the same position
=============================================================================
Pin Connection Description:
1. PA0,PA1,PA3,PA4 connect to left motor's IN1,IN2 and right motor's IN1 IN2
2. PH3(OC4A),PH4(OC4B) connect to left motor's EN and right motor' EN and
   these two pins can output pwm waves
3. PD0(INT0),PD1(INT1),PD2(INT2) connect to limit switch, encoder's A phase
   encoder's B phase
4. PD3(INT3),PE4(INT4),PE5(INT5) connect to the radio controller's receiver's
   channel 1,channel 2 and channel 3. These three channels control motion of the robot
5. The AVCC, AREF and ADC0 should connect to the supply power in order to
   measure its voltage
PA0 is digital pin 22 on arduino mega 2560
PA1 is digital pin 23 on arduino mega 2560
PA2 is digital pin 24 on arduino mega 2560
PA3 is digital pin 25 on arduino mega 2560
PD0 is digital pin 21 on arduino mega 2560
PD1 is digital pin 20 on arduino mega 2560
PD2 is digital pin 19 on arduino mega 2560
PD3 is digital pin 18 on arduino mega 2560
PE4 is digital pin 2 on arduino mega 2560
PE5 is digital pin 3 on arduino mega 2560
PH3 is digital pin 6 on arduino mega 2560
PH4 is digital pin 7 on arduino mega 2560
ADC0 is analog pin 0 on arduino mega 2560
=============================================================================
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#define sbi(x,y) x |= _BV(y) //set bit - using bitwise OR operator
#define cbi(x,y) x &= ~(_BV(y)) //clear bit - using bitwise AND operator
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define abs(x)    ((x)>0?(x):-(x))
#define FOSC 16000000  //clock speed
#define BAUD 4800
#define MYUBRR (FOSC/16/BAUD-1)


int ch1;
int ch2;
int ch4;
int absch1;
int absch2;
int absch4;
double batteryvoltage;
double speed;
unsigned long int freq = 0; //  to store value of frequency value
unsigned char indata;  // store the received data using USART
unsigned int pulsenumber = 0;
static unsigned int n;  // scale to transform the pulse frequency into motor's speed



void PORT_Init(void);
void USART_Init(unsigned int ubrr);
void INT_Init(void);
void TIMER1_Init(void);
void TIMER3_Init(void);
void ADC_Init(void);
void USART_Transmit(unsigned char data);
void left_pwm_output(int x);
void right_pwm_output(int x);
void controlmove(void);
void forward_turnleft(int x);
void forward_turnright(int x);
void standing_turnright(int x);
void standing_turnleft(int x);
void forward(int x);
void reverse(int x);
void stopcar(void);
int map(int ch, int x, int y, int a, int b);


int main(void)
{

	/* Initialization */
	PORT_Init();
	USART_Init(MYUBRR);
	INT_Init();
	TIMER1_Init();
	TIMER3_Init();
	ADC_Init();
	stopcar();
	wdt_disable();
	/* Global Interrupt Enable */
	sei();  //SREG=(1<<7);
	while (1)
	{
		controlmove();
	}
	return 0;
}


void PORT_Init(void)
{
	/* PA0,PA1,PA2,PA3 output high */
	DDRA = 0x0F;
	PORTA = 0x0F;
	/* PD0(INT0),PD1(INT1),PD2(INT2) input(pull-up)
	 PD3(INT3) input(no pull-up) */
	DDRD = 0x00;
	PORTD = 0x07;
	//PE4(INT4),PE5(INT5) input(no pull-up)
	DDRE = 0x00;
	PORTE = 0x00;
	// PH3(OC4A),PH4(OC4B) output high
	DDRH = 0x18;
	PORTH = 0x18;
}
void USART_Init(unsigned int ubrr)
{
	/* Set baud rate */
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable RX complete Interrupt, Enable receiver and transmitter */
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
	/* Sychronous USART; Set frame format:8 data,2 stop bits */
	UCSR0C = (1 << UMSEL00) | (1 << USBS0) | (3 << UCSZ00);
}
void INT_Init(void)
{
	/*// falling edge of INT0,
	   falling edge of INT1,
	   rising edge of INT2,
	   rising edge of INT3,
	   rising edge of INT5,
	   rising edge of INT6 */
	EICRA |= (1 << ISC01) | (1 << ISC11) | (1 << ISC21) | (1 << ISC20) | (1 << ISC31) | (1 << ISC30);
	EICRB |= (1 << ISC61) | (1 << ISC60) | (1 << ISC51) | (1 << ISC50);
	//external interrupt 0,1,2,3,5,6 request enable
	EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2) | (1 << INT3) | (1 << INT5) | (1 << INT6);
	// clear external interrupt 0,1,2,3,5,6 flag
	EIFR |= (1 << INTF0) | (1 << INTF1) | (1 << INTF2) | (1 << INTF3) | (1 << INTF5) | (1 << INTF6);
}
void TIMER1_Init(void)
{
	TCCR1A = 0x00;
	TCCR1B = (1 << CS12); // prescaler :clk/256  i.e. 62500Hz
	TCNT1 = 0x85EE; // timer: 0.5s
	TIMSK1 = (1 << TOIE1); //overflow interrupt enable
	TIFR1 = (1 << TOV1); //clear the timer 1 overflow flag
}
void TIMER3_Init(void)
{
	/*no prescaling */
	TCCR3A = 0x00;
	TCCR3B = (1 << CS30);
	TCNT3 = 0x00;
}
void ADC_Init(void)
{
	ADMUX = (1 << REFS1); // use internal 1.1V voltage reference
	// select ADC0 as the input battery voltage pin
	/* ADC Enable, ADC Start Conversion, ADC Auto Trigger Enable
	ADC Interrupt Enable, Divison factor=32 */
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE) | 5;
}
void USART_Transmit(unsigned char data)
{
	/* wait for empty transmit buffer */
	while (!(UCSR0A & (1 << UDRE0)));
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

/*
unsigned char USART_Receive(void)
{
  // wait for data to be received
  while (!(UCSR0A & (1<<RXC0)));
  //Get and return received data from buffer
  return UDR0;
}
*/

ISR(USART0_RX_vect)
{
	indata = UDR0; // The received data is store in variable indata
}

ISR(ADC_vect)
{
	unsigned char adc_data;
	adc_data = ADC >> 2; //read 8 bit value
	batteryvoltage = 1.1 * 255 / adc_data;
}
ISR(INT0_vect)
{
	stopcar();
}
ISR(INT1_vect)
{
	pulsenumber++;
	if (!bit_is_clear(PIND, PD1))
	{
		USART_Transmit('+');  // The motor is rotating forward
	}
	else
	{
		USART_Transmit('-');  // The motor is rotating backward
	}
}
ISR(INT2_vect)
{
	if (!bit_is_clear(PIND, PD2))
	{
		USART_Transmit('+');  // The motor is rotating forward
	}
	else
	{
		USART_Transmit('-');  // The motor is rotating backward
	}
}
ISR(INT3_vect)
{
	TCNT3 = 0x0000;
	while (bit_is_set(PIND, PD3));
	unsigned int now = TCNT3;
	ch1 = now / 16.0 / 1000; //Measuring the width of incoming high
	//voltage pulse in channel 1
	ch1 = map(ch1, 1000, 2000, -255, 255);
	ch1 = constrain(ch1, -255, 255);
	if (ch1 < 20 && ch1 > -20)
	{ch1 = 0;}
	absch1 = abs(ch1);
}
ISR(INT4_vect)
{
	TCNT3 = 0x0000;
	while (bit_is_set(PINE, PE4));
	unsigned int now = TCNT3;
	ch2 = now / 16.0 / 1000; //Measuring the width of incoming high
	//voltage pulse in channel 2
	ch2 = map(ch2, 1000, 2000, -255, 255);
	ch2 = constrain(ch2, -255, 255);
	if (ch2 < 20 && ch2 > -20)
	{ch2 = 0;}
	absch2 = abs(ch2);
}
ISR(INT5_vect)
{
	TCNT3 = 0x0000;
	while (bit_is_set(PINE, PE5));
	unsigned int now = TCNT3;
	ch4 = now / 16.0 / 1000; //Measuring the width of incoming high
	//voltage pulse in channel 2
	ch4 = map(ch4, 1000, 2000, -255, 255);
	ch4 = constrain(ch4, -255, 255);
	if (ch4 < 20 && ch4 > -20)
	{ch4 = 0;}
	absch4 = abs(ch4);
}
ISR(TIMER1_OVF_vect)
{
	// Timer 1 :0.5 s ----> overflow
	freq = 2 * pulsenumber; //calculate the pulse freqency
	TCNT1 = 0x85EE;
	pulsenumber = 0;
	speed = freq * n; // transform the pulse frequency into the motor's speed
	// and n is the scale which is to be determined by the encoder
}
void left_pwm_output(int x)
{
	// x should be between 0 and 255
	/*fast pwm,8-bit, top value is 0x00FF*/
	TCCR4A = (1 << COM4A1) | (1 << WGM40);
	TCCR4B = (1 << WGM42) | (1 << CS40);
	TCNT4 = 0x0000;
	OCR4AL = x;
}
void right_pwm_output(int x)
{
	// x should be between 0 and 255
	/*fast pwm,8-bit */
	TCCR4A = (1 << COM4A1) | (1 << WGM40);
	TCCR4B = (1 << WGM42) | (1 << CS40);
	TCNT4 = 0x0000;
	OCR4BL = x;
}
void controlmove(void)
{
	if (ch1 == 0 && ch2 == 0 && ch4 == 0)
	{
		stopcar();
	}
	else if (ch1 > 0)
	{
		forward_turnleft(absch1);
	}
	else if (ch1 < 0)
	{
		forward_turnright(absch1);
	}
	else if (ch2 > 0)
	{
		forward(absch2);
	}
	else if (ch2 < 0)
	{
		reverse(absch2);
	}
	else if (ch4 > 0)
	{
		standing_turnleft(absch4);
	}
	else
	{
		standing_turnright(absch4);
	}
}
void forward_turnleft(int x)
{
	int a, b;
	a = x;
	if (a <= 100)
	{
		a = 100;
	}
	b = x - 80;
	left_pwm_output(b);
	right_pwm_output(a);
	PORTA = 0x05; // digital pin 22,23,24,25 on arduino mega 2560
	// left and right motors both rotate forward, but
	// right motor has higher speed
}
void forward_turnright(int x)
{
	int a, b;
	a = x;
	if (a <= 100)
	{
		a = 100;
	}
	b = x - 80;
	left_pwm_output(a);
	right_pwm_output(b);
	PORTA = 0x05; // left and right motors both rotate forward, but
	// left motor has higher speed
}
void standing_turnright(int x)
{
	left_pwm_output(x);
	right_pwm_output(x);
	PORTA = 0x09; // left motor rotates forward, right motor rotates backward
}
void standing_turnleft(int x)
{
	left_pwm_output(x);
	right_pwm_output(x);
	PORTA = 0x06; // left motor rotates backward, right motor rotates forward
}
void forward(int x)
{
	left_pwm_output(x);
	right_pwm_output(x);
	PORTA = 0x05; // left and right motors rotate forward
}
void reverse(int x)
{
	left_pwm_output(x);
	right_pwm_output(x);
	PORTA = 0x0A;  // left and right motors rotate backward
}
void stopcar(void)
{
	PORTA |= 0x0F;
	sbi(PORTE, 3);
	sbi(PORTH, 3);
	TCCR3B &= 0x00; //stop timer 3
}
int map(int ch, int x, int y, int a, int b)
{
	return ((b - a) * (ch * 1.0 - x) / (y - x) + a);
}