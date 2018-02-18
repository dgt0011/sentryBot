
#define F_CPU 16000000UL

#define UART_BAUD_RATE	9600

#define TX_NEWLINE {uart_putc(0x0d); uart_putc(0x0a);}

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "MotorControl.h"
#include "hmc5883l/hmc5883l.h"
#include "magn_docalibration.h"
#include "utility.h"

#define MAGN_DOCALIBRATION 0 //enable or disable magnetometer calibration
#define magncal_getrawdata(mxraw, myraw, mzraw) hmc5883l_getrawdata(mxraw, myraw, mzraw); //set the function that return magnetometer raw values

#define LED_PORT	PORTC
#define LED_DDR		DDRC

#define MOTOR_PORT	PORTB
#define MOTOR_DDR	DDRB

#define PORT_LED_MASK		(1<<PC0)
#define STARBOARD_LED_MASK	(1<<PC1)
#define ACTIVE_LED_MASK		(1<<PC2)

#define DEBUG_PIN_MASK		(1<<PC3)

#define PORT_MOTOR			(1<<PB1)
#define STARBOARD_MOTOR		(1<<PB2)

#define DELAY		1000

#define FOSC 16000000                       // Clock Speed
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD -1)

int16_t getHeadingInt(void);

int16_t getPulseComparitorForDistance(int distanceInDecimeters);

void startUpIndicator(void);

// SPI communications
void send_string_SPI(char s[]);
void send_char_SPI(char c);
void init_spi_master(void);

// UART debugging
void uart_init(void);
void uart_puts(const char *s );
void uart_putc(unsigned char data);

const int BEARING_TOLERANCE = 5;

const int16_t SINGLE_REVOLUTION_FWD = 4850;
const int16_t SINGLE_REVOLUTION_REV = 4850;
const int16_t MEASURED_DISTANCE_PER_REVOLUTION = 35;	//centimeters

volatile int uartBufferIdx;
volatile char uartBuffer[20];
volatile char commandBuffer[20];

volatile int16_t pulseCount;
volatile int16_t pulseCountComparitor;

volatile uint8_t command;
volatile uint8_t commandDurationMs;
volatile uint8_t durationIdx;
uint8_t _commandTest;

int main(void)
{
	int16_t mxraw = 0;
	int16_t myraw = 0;
	int16_t mzraw = 0;
	double mx = 0;
	double my = 0;
	double mz = 0;
	char itmp[10];	
	
	pulseCountComparitor = 0;
	pulseCount = 0;
	
	LED_DDR |= PORT_LED_MASK | STARBOARD_LED_MASK | ACTIVE_LED_MASK | DEBUG_PIN_MASK;
	MOTOR_DDR |= PORT_MOTOR |STARBOARD_MOTOR;
	
	startUpIndicator();
	
	init_spi_master();
	uart_init();
	
	uart_puts("Comms online.");
	TX_NEWLINE;
	send_string_SPI("Comms online.\n");
	
	uart_puts("Initialising sensors ...");
	TX_NEWLINE;
	send_string_SPI("Initialising sensors ..\n");
	
	hmc5883l_init();
	
	uart_puts("Initialising drives ..");
	TX_NEWLINE;
	send_string_SPI("Initialising drives ..\n");
	
	drive_init();
	
	// h/w interrupts on INT0 and INT1
	DDRD &= ~(1 << DDD2);		// Clear the PD2 & PD3 pin & enable PD2 & PD3 as input
	DDRD &= ~(1 << DDD3);
	PORTD |= (1 << PORTD2) | (1 << PORTD3);

	//trigger INT0, INT1 on rising edge
	EICRA |= (1 << ISC11) | (1<< ISC01);
	EIMSK |= (1 << INT0) | (1 << INT1);
	
	//timer 0 for 1 ms
	// Set the Timer Mode to CTC
	TCCR0A |= (1 << WGM01);
	OCR0A = 0xF9;	//249
	TCCR0B |= (1 << CS02);
	TIMSK0 |= (1 << OCIE0A);    //Set the ISR COMPA vect
	TCCR0B |= (0 << CS01) | (0 << CS00);  // sets prescaler to 64 and stops the timer
	// NOTE: Timer0 NOT currently needed (was used for attempt #1 of event driven driving)
	
	#if MAGN_DOCALIBRATION  == 1
	// magncal_docalibrationclient(uart_putc, uart_getc);
	#endif

	sei();
	
	uart_puts("System ready.");
	TX_NEWLINE;
	send_string_SPI("System ready.\n");
	
	////test drive - simple approach #1
	//LED_PORT |= DEBUG_PIN_MASK;
	//pulseCount = 0;
	//forward();
	//_delay_ms(5300); //TODO: Tese all need to now be timers - now that we've got interrupts
	//
	//char pulseStr[5];
	//
	//dec_to_str(pulseStr,pulseCount, 4);
	//pulseStr[4] = '\n';
	//send_string_SPI(pulseStr);
	//uart_puts(pulseStr);
	//
	//allStop();
	//_delay_ms(1000);
	//
	//pulseCount = 0;
	//reverse();
	//_delay_ms(5300);	//TODO: Tese all need to now be timers - now that we've got interrupts
	//
	//dec_to_str(pulseStr,pulseCount, 4);
	//pulseStr[4] = '\n';
	//send_string_SPI(pulseStr);
	//uart_puts(pulseStr);
	
	//allStop();
	
	//turnRightTo(90);
	
	//command = FORWARD;
	//commandDurationMs = 5300;
	//durationIdx = 0;
	//_commandTest = FORWARD;
	//pulseCount = 0;
	
	//TCCR0B |= (1 << CS01) | (1 << CS00);  // set prescaler to 64 and start the timer
	//sei();
	
	while (1)
	{
		// read of compass testing ---
		hmc5883l_getrawdata(&mxraw, &myraw, &mzraw);
		hmc5883l_getdata(&mx, &my, &mz);
		
		//get magnetic heading (in degrees)
		float heading = 0;
		heading = atan2((double)myraw,(double)mxraw)*57.29578;
		//add magnetic declination (optional)
		//get magnetic declination (in degrees) http://magnetic-declination.com
		//if you have an EAST declination use +, if you have a WEST declination use -
		//es, my declination is 1.73 positive
		float declination = 10.59;
		heading += declination;
		//check 360degree heading
		if(heading < 0)
		heading = 360 + heading;
		
		itoa(mxraw, itmp, 10); uart_puts(itmp); uart_putc(' ');
		itoa(myraw, itmp, 10); uart_puts(itmp); uart_putc(' ');
		itoa(mzraw, itmp, 10); uart_puts(itmp); uart_putc(' ');
		dtostrf(mx, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(my, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(mz, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(heading, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");
		
		int headingInt = (int)heading;
		char distStr[5];
		dec_to_str(distStr,headingInt, 3);
		distStr[3] = '\n';
		send_string_SPI(distStr);
		
		_delay_ms(500);
		// end read of compass testing ---
		
		// -- first attempt at 'event' driven command testing (failure)
		//if(command > 0)
		//{
		//if(command == FORWARD) {
		//if(durationIdx == 0)
		//{
		//uart_puts("motors forward");
		//TX_NEWLINE;
		//send_string_SPI("motors forward\n");
		//forward();
		//TCCR0B |= (1 << CS01) | (1 << CS00);  // set prescaler to 64 and start the timer
		//sei();
		//}
		//
		//// have we driven forward enough?
		//if(durationIdx >= commandDurationMs) {
		//cli();
		////yep.
		//command = STOP;
		//// time limit the stahp
		//commandDurationMs = 500;
		//durationIdx = 0;
		//
		//// debuggery
		//char pulseStr[5];
		//dec_to_str(pulseStr,pulseCount, 4);
		//pulseStr[4] = '\n';
		//send_string_SPI(pulseStr);
		//uart_puts(pulseStr);
		//sei();
		//
		//}
		//}
		//
		//if(command == REVERSE) {
		//if(durationIdx == 0)
		//{
		//uart_puts("motors reverse");
		//TX_NEWLINE;
		//send_string_SPI("motors reverse\n");
		//reverse();
		//TCCR0B |= (1 << CS01) | (1 << CS00);  // set prescaler to 64 and start the timer
		//sei();
		//}
		//// have we driven in reverse enough?
		//if(durationIdx >= commandDurationMs) {
		//cli();
		////yep.
		//command = STOP;
		//// time limit the stahp
		//commandDurationMs = 500;
		//durationIdx = 0;
		//
		//// debuggery
		//char pulseStr[5];
		//dec_to_str(pulseStr,pulseCount, 4);
		//pulseStr[4] = '\n';
		//send_string_SPI(pulseStr);
		//uart_puts(pulseStr);
		//sei();
		//}
		//}
		//
		//if(command == STOP) {
		//
		//if(durationIdx == 0)
		//{
		//uart_puts("motors stopped");
		//TX_NEWLINE;
		//send_string_SPI("motors stopped\n");
		//}
		//
		//allStop();
		//
		//if(durationIdx >= commandDurationMs) {
		//
		////stahped long enough
		//
		////for now - choose what next
		//if(_commandTest == FORWARD){
		//cli();
		//_commandTest = REVERSE;
		//command = REVERSE;
		//commandDurationMs = 5300;
		//durationIdx = 0;
		//pulseCount = 0;
		//sei();
		//} else {
		////we're done
		//TCCR0B |= (0 << CS01) | (0 << CS00);  //STOP the timer
		//}
		//}
		//
		//}
		//
		//}
		// -- first attempt at 'event' driven command testing (failure)
		
		if(pulseCountComparitor > 0)
		{
			if(pulseCount >= pulseCountComparitor){
				uart_puts("pulseCount >= pulseCountComparitor");
				TX_NEWLINE;
				
				char pulseStr[16];
				dec_to_str(pulseStr,pulseCount, 12);
				uart_puts(pulseStr);
				TX_NEWLINE;
				
				
				TX_NEWLINE;
				allStop();
				pulseCountComparitor = 0;
				pulseCount = 0;
			}
		}
		
		//if the command buffer is not empty ...
		if(commandBuffer[0] != 0){
			
			uart_puts(commandBuffer);
			TX_NEWLINE;
			send_string_SPI(commandBuffer);
			
			//temporarily halt interrupts to try to avoid data loss
			cli();
			
			// parse the buffer into command : argument
			char delim = ':';
			int val = 0;
			int idx = 0;
			int handled = 0;
			char operation = 0;
			
			char** tokens;
			tokens = str_split(commandBuffer, delim);
			if(tokens) {
				operation = *(tokens+idx)[0];
				idx++;
				val = safeStrtoi(*(tokens+idx),10);
				
				uart_puts("Value : ");
				uart_puts(*(tokens+1));
				
				uart_puts(" [ ");
				char valStr[5];
				dec_to_str(valStr,val, 3);
				uart_puts(valStr);
				uart_puts(" ] ");
				TX_NEWLINE;
				
				//cleanup
				int i;
				for(i=0;*(tokens+i);i++){
					free(*(tokens+i));
				}
				free(tokens);
				
				if(operation == STOP)
				{
					handled = 1;
					pulseCountComparitor = 0;
					pulseCount = 0;
					allStop();
				}
				
				if(operation == FORWARD)
				{
					handled = 1;
					if(val == 0){
						pulseCountComparitor = SINGLE_REVOLUTION_FWD;
					} else {
						pulseCountComparitor = getPulseComparitorForDistance(val);
					}
					pulseCount = 0;
					forward();
				}
				
				if(operation == REVERSE)
				{
					handled = 1;
					if(val == 0){
						pulseCountComparitor = SINGLE_REVOLUTION_REV;
						} else {
						pulseCountComparitor = getPulseComparitorForDistance(val);
					}					
										
					pulseCountComparitor = SINGLE_REVOLUTION_REV;
					pulseCount = 0;
					reverse();
				}

				// TODO: The following are not 'event' enabled!
				if(operation == TURNRIGHT)
				{
					handled = 1;
					turnRightBy(val);
					pulseCount = 0;
				}
				if(operation == TURNLEFT)
				{
					handled = 1;
					turnLeftBy(val);
					pulseCount = 0;
				}
				
				
				} else {
				uart_puts("bad command :");
				uart_puts(commandBuffer);
				TX_NEWLINE;
			}
			
			if(handled == 0){
				uart_puts("Unknown command :");
				uart_putc(operation);
				TX_NEWLINE;
				send_string_SPI("Unknown command\n");
				allStop();
			}
			
			commandBuffer[0] = 0;
			// restart interrupts
			sei();
			
			
			} else {
			//uart_putc('^');
		}
	}
}

void startUpIndicator(void) {
	//PORTD &= ~(1 << n); // Pin n goes low
	//PORTD |= (1 << n); // Pin n goes high
	LED_PORT |= ACTIVE_LED_MASK;
	_delay_ms(1000);
	LED_PORT &= ~ACTIVE_LED_MASK;
	_delay_ms(1000);
	LED_PORT |= ACTIVE_LED_MASK;
	_delay_ms(1000);
	LED_PORT &= ~ACTIVE_LED_MASK;
	_delay_ms(1000);
	LED_PORT |= ACTIVE_LED_MASK;
	_delay_ms(1000);
	LED_PORT &= ~ACTIVE_LED_MASK;
	_delay_ms(1000);
	LED_PORT |= ACTIVE_LED_MASK;
	_delay_ms(1000);
	
	LED_PORT |= PORT_LED_MASK | STARBOARD_LED_MASK;
	_delay_ms(1000);
	LED_PORT &= ~PORT_LED_MASK;
	LED_PORT &= ~STARBOARD_LED_MASK;
}

void init_spi_master()
{
	DDRB |= (1<<2)|(1<<3)|(1<<5);    // SCK, MOSI and SS as outputs
	DDRB &= ~(1<<4);                 // MISO as input

	SPCR |= (1<<MSTR);               // Set as Master
	SPCR |= (1<<SPR0)|(1<<SPR1);     // divided clock by 128
	//SPCR |= (1<<SPIE);               // Enable SPI Interrupt - SPI is only for outgoing/logging - not incoming, so not needed in this case
	SPCR |= (1<<SPE);                // Enable SPI
}

void send_string_SPI(char s[])
{
	int i =0;
	while (s[i] != 0)
	{
		send_char_SPI(s[i]);
		i++;
	}
}

void send_char_SPI(char c)
{
	SPDR = c;                 // send the data
	while(!(SPSR & (1<<SPIF)));  // wait until transmission is complete
}

void uart_init(void){
	/*Set baud rate */
	UBRR0H = (MYUBRR >> 8);
	UBRR0L = MYUBRR;
	
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);      // Enable receiver and transmitter
	UCSR0B |= (1 << RXCIE0);                    // Enable reciever interrupt
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);    // Set frame: 8data, 1 stp

	uartBufferIdx = 0;
	memset(uartBuffer, 0, sizeof(uartBuffer));
	//sei();                                      // Lets not forget to enable interrupts =P
}

void uart_putc(unsigned char data)
{
	while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
	UDR0 = data;                                   // load data in the register
}

void uart_puts(const char *s) {
	while (*s)
	{
		uart_putc(*s++);
	}
}

void drive_init(void) {
	DDRB |= (1 << DDB1)|(1 << DDB2);
	// enable ORC1A and ORC1B (PB1 and PB2 PWM out)
	TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	
	TCNT1H=0x00;
	TCNT1L=0x00;
	
	ICR1H=0x9C;
	ICR1L=0x3F;
	
	allStop();
	_delay_ms(1000);
}

int16_t getHeadingInt() {

	int16_t mxraw = 0;
	int16_t myraw = 0;
	int16_t mzraw = 0;
	hmc5883l_getrawdata(&mxraw, &myraw, &mzraw);

	//get magnetic heading (in degrees)
	float heading = 0;
	heading = atan2((double)myraw,(double)mxraw)*57.29578;
	//add magnetic declination (optional)
	//get magnetic declination (in degrees) http://magnetic-declination.com
	//if you have an EAST declination use +, if you have a WEST declination use -
	//es, my declination is 1.73 positive
	float declination = 10.59;
	heading += declination;
	//check 360degree heading
	if(heading < 0)
	heading = 360 + heading;
	
	return (int16_t)heading;
}

void allStop(void) {
	LED_PORT &= ~PORT_LED_MASK;
	LED_PORT &= ~STARBOARD_LED_MASK;
	
	uart_puts("motors stop");
	TX_NEWLINE;
	send_string_SPI("motors stop\n");
	
	OCR1AH=0x0B;
	OCR1AL=0xB8;
	OCR1BH=0x0B;
	OCR1BL=0xB8;
}

void forward() {
	LED_PORT  |= PORT_LED_MASK | STARBOARD_LED_MASK;
	
	uart_puts("motors forward");
	TX_NEWLINE;
	send_string_SPI("motors forward\n");
	
	//drive forward (both wheels)
	// aim is OCR1 to be between 1.5 and 2.0ms for forward.
	// stopped OCR1 above is 3ms (3000) (half this for fast pwm versus corrected pwm??
	// if so - 1.8ms should be forward x 1000 to get 1800 then x2 for 3600 for the value of OCR1 - E10 in hex
	OCR1AH=0x09;
	OCR1AL=0x60;
	OCR1BH=0x09;
	OCR1BL=0x60;
}

void reverse(void){
	//drive backward (both wheels)
	// aim is OCR1 to be between 1.0 and 1.5ms for backward.
	// stopped OCR1 above is 3ms (3000) (half this for fast pwm versus corrected pwm??
	// if so - 1.2ms should be forward x 1000 to get 1200 then x2 for 2400 for the value of OCR1 - 960 in hex
	
	uart_puts("motors reverse");
	TX_NEWLINE;
	send_string_SPI("motors reverse\n");
	
	OCR1AH=0x0E;
	OCR1AL=0x10;
	OCR1BH=0x0E;
	OCR1BL=0x10;
}

void turnLeft(void){
	LED_PORT |= STARBOARD_LED_MASK;
	OCR1AH=0x0E;
	OCR1AL=0x10;
	OCR1BH=0x09;
	OCR1BL=0x60;
}

void turnRight(void){
	LED_PORT  |= PORT_LED_MASK;
	OCR1AH=0x09;
	OCR1AL=0x60;
	OCR1BH=0x0E;
	OCR1BL=0x10;
}

void turnRightBy(int16_t degrees){
	
	char buffer[14];
	char heading[4];
	char target[4];
	
	uart_puts("	-> turnRightBy");
	TX_NEWLINE;
		
	//rudimentary average sampling
	int16_t sample1 = getHeadingInt();
	int16_t sample2 = getHeadingInt();
	int16_t sample3 = getHeadingInt();	
	int16_t headingVal = (sample1 + sample2 + sample3)/3;
	
	dec_to_str(heading,headingVal, 3);
	strncpy(buffer, heading, 3);
	buffer[3] = '+';
	
	dec_to_str(target,degrees, 3);
	strncpy(buffer+4, target, 3);
	buffer[7] = '=';
	
	int16_t newHeadingVal = headingVal + degrees;
	
	dec_to_str(target,newHeadingVal, 3);
	strncpy(buffer+8, target, 3);
	uart_puts(buffer);
	TX_NEWLINE;
	
	if(newHeadingVal > 360){
		newHeadingVal = newHeadingVal - 360;
	}
	
	turnRightTo(newHeadingVal);
	
	uart_puts("	<- turnRightBy");
	TX_NEWLINE;
}

void turnLeftBy(int16_t degrees) {
	
	uart_puts("	-> turnLeftBy");
	TX_NEWLINE;
	
	//rudimentary average sampling
	int16_t sample1 = getHeadingInt();
	int16_t sample2 = getHeadingInt();
	int16_t sample3 = getHeadingInt();
	int16_t headingVal = (sample1 + sample2 + sample3)/3;
	
	int16_t newHeadingVal = headingVal - degrees;
	if(newHeadingVal < 0){
		newHeadingVal = abs(newHeadingVal);
	}
	turnLeftTo(newHeadingVal);
	
	uart_puts("	<- turnLeftBy");
	TX_NEWLINE;
}

void turnRightTo(int16_t targetBearing){
	
	char buffer[14];
	char heading[4];
	char target[4];
	
	uart_puts("	-> turnRightTo");
	TX_NEWLINE;
	
	int16_t sample1 = getHeadingInt();
	int16_t sample2 = getHeadingInt();
	int16_t sample3 = getHeadingInt();
	int16_t headingVal = (sample1 + sample2 + sample3)/3;
	
	dec_to_str(heading,headingVal, 3);
	strncpy(buffer, heading, 3);
	
	strncpy(buffer+3, " -> ", 4);
	
	dec_to_str(target,targetBearing, 3);
	strncpy(buffer+7, target, 3);
	
	uart_puts(buffer);
	TX_NEWLINE;
	
	buffer[10] = '\n';
	send_string_SPI(buffer);
	
	int16_t degreesToTurn = 0;
	if(headingVal <= targetBearing)
	{
		degreesToTurn = targetBearing - headingVal;
	} else {
		degreesToTurn = (360-headingVal)+targetBearing;
	}
	
	while(degreesToTurn > 0) {
		
		headingVal = getHeadingInt();
		
		dec_to_str(heading,headingVal, 3);
		strncpy(buffer, heading, 3);
		
		strncpy(buffer+3, " -> ", 4);
		
		dec_to_str(target,targetBearing, 3);
		strncpy(buffer+7, target, 3);
		
		uart_puts(buffer);
		TX_NEWLINE;
		
		buffer[10] = '\n';
		send_string_SPI(buffer);
		
		turnRight();		
	
		if(headingVal <= targetBearing)
		{
			degreesToTurn = targetBearing - headingVal;
		} else {
			degreesToTurn = (360-headingVal)+targetBearing;
		}
		
		if(abs(degreesToTurn) <=  BEARING_TOLERANCE)
		{
			degreesToTurn = 0;			
		}
		_delay_ms(250);
	}
	
	uart_puts("	<- turnRightTo");
	TX_NEWLINE;
	
	allStop();
}

void turnLeftTo(int16_t targetBearing){
	
	char buffer[14];
	char heading[4];
	char target[4];
	
	uart_puts("	-> turnLeftTo");
	TX_NEWLINE;

	dec_to_str(target,targetBearing, 3);
	
	strncpy(buffer, target, 3);
	strncpy(buffer+3, " <- ", 4);

	int16_t sample1 = getHeadingInt();
	int16_t sample2 = getHeadingInt();
	int16_t sample3 = getHeadingInt();
	int16_t headingVal = (sample1 + sample2 + sample3)/3;
	
	dec_to_str(heading,headingVal, 3);
	strncpy(buffer+7, heading, 3);
	
	buffer[10] = '\n';
	
	uart_puts(buffer);
	TX_NEWLINE;
	send_string_SPI(buffer);
	
	int degreesToTurn = 0;
	
	if(headingVal >= targetBearing)
	{
		degreesToTurn = headingVal - targetBearing;
		} else {
		degreesToTurn = 360 - (targetBearing - headingVal);
	}
	
	while(degreesToTurn > 0) {
		
		headingVal = getHeadingInt();
				
		dec_to_str(target,targetBearing, 3);
		strncpy(buffer, target, 3);
		strncpy(buffer+3, " <- ", 4);
		
		dec_to_str(heading,headingVal, 3);
		strncpy(buffer+7, heading, 3);
		
		uart_puts(buffer);
		TX_NEWLINE;
		
		buffer[10] = '\n';
		send_string_SPI(buffer);	
		
		turnLeft();
				
		if(headingVal >= targetBearing)
		{
			degreesToTurn = headingVal - targetBearing;
		} else {
			degreesToTurn = 360 - (targetBearing - headingVal);
		}
		
		if(abs(degreesToTurn) <=  BEARING_TOLERANCE)
		{
			degreesToTurn = 0;
		}
		
		_delay_ms(250);
	}
	
	uart_puts("	<- turnLeftTo");
	TX_NEWLINE;
	
	allStop();
}

void turnTo(int16_t targetBearing){
	
}

int16_t getPulseComparitorForDistance(int distanceInDecimeters){
	
	int16_t revsPerCentimetre = (int16_t)(SINGLE_REVOLUTION_FWD / MEASURED_DISTANCE_PER_REVOLUTION);
	return revsPerCentimetre * (distanceInDecimeters * 10);	
}

ISR (INT0_vect)
{
	if( (PIND & (1<<PIND2)) == 0)    //is the pin set
	{
		_delay_us(55);
		if( (PIND & (1<<PIND2)) == 0) // is the pin still set
		{
			if(pulseCountComparitor > 0){
				pulseCount = pulseCount+1;
			}
		}
		else
		{
			// do nothing because the input is invalid
		}
	}
}

ISR (INT1_vect)
{

}

ISR (TIMER0_COMPA_vect)  // timer0 overflow interrupt
{
	//event to be executed every 1ms here
	durationIdx += 1;
}

ISR (USART_RX_vect)
{
	char receivedChar;
	receivedChar = UDR0;                       // Read data from the RX buffer
	
	uartBuffer[uartBufferIdx] = receivedChar;
	
	if(receivedChar == '\n') {
		uartBuffer[uartBufferIdx] = '\0';
	}
	
	uartBufferIdx ++;
	
	if(receivedChar == '\n') {
		strncpy(commandBuffer, uartBuffer, sizeof(commandBuffer));
		uartBufferIdx = 0;
		memset(uartBuffer, 0, sizeof(uartBuffer));
	}
	
	//UDR0 = ReceivedChar;                       // Write the data to the TX buffer
}



