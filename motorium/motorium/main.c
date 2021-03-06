#define F_CPU 16000000UL

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

#include "main.h"

volatile int uartBufferIdx;
volatile char uartBuffer[20];
volatile char commandBuffer[20];

volatile int16_t _pulseCount;
volatile int16_t _pulseCountComparitor;

volatile uint8_t _timer0_accum;

uint8_t		_currentCommand;

int16_t		_targetBearing;
int16_t		_degrees;	//maybe not needed?

int main(void)
{
	_currentCommand = 0x00;
	_pulseCountComparitor = 0;
	_pulseCount = 0;
	
	init_OutputPins();	
	startUpIndicator();	
	
	init_spi_master();
	init_uart();
	
	#if MAGN_DOCALIBRATION  == 1
	// magncal_docalibrationclient(uart_putc, uart_getc);
	#endif
	
	uart_puts("Comms online.");
	TX_NEWLINE;
	
	lcdClearAndHome();
	send_string_SPI("Comms online.\n");
	_delay_ms(1000);
	lcdClearAndHome();
	
	uart_puts("Initialising sensors ...");
	TX_NEWLINE;	
	send_string_SPI("Init sensors...\n");
	_delay_ms(1000);
	lcdClearAndHome();
	
	// compass
	hmc5883l_init();
	
	uart_puts("Initialising drives ..");
	TX_NEWLINE;
	
	send_string_SPI("Init drives...\n");
	_delay_ms(1000);
	lcdClearAndHome();	
	
	drive_init();
	_delay_ms(1000);
	lcdClearAndHome();
	
	init_hwInterrupts();
	
	init_timer0();
		
	lcdGoTo(0, 1);
	send_string_SPI("H:\n");	

	sei();
		
	while (1)
	{
		if(_timer0_accum >= TIMER0_TOP){
			_timer0_accum = 0;
			updateState();
		}
	}
}

void updateState(void){	
	displayCurrentBearing();
	
	// if currently moving in forward or reverse - check if the distance has been reached ...
	if(_pulseCountComparitor > 0)
	{
		// currently moving in a straight line - forward or reverse
		if(_pulseCount >= _pulseCountComparitor){
			uart_puts("pulseCount >= pulseCountComparitor");
			TX_NEWLINE;
			
			char pulseStr[16];
			dec_to_str(pulseStr,_pulseCount, 12);
			uart_puts(pulseStr);
			TX_NEWLINE;
			
			allStop();
			signalCommandComplete();
			_pulseCountComparitor = 0;
			_pulseCount = 0;
		}
	}
	
	//if currently turning - check if the angle has been reached ...
	if(_currentCommand == TURNLEFT || _currentCommand == TURNRIGHT){
		checkTurnBearing(_currentCommand);
	}
	
	// finally check if we've received a new command or a HALT!
	if(commandBuffer[0] != 0){
		//if the command buffer is not empty ...
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
			OUTPUT_PORT |= ACTIVE_LED_MASK;			
			operation = *(tokens+idx)[0];
			idx++;
			val = safeStrtoi(*(tokens+idx),10);
			
			//cleanup
			int i;
			for(i=0;*(tokens+i);i++){
				free(*(tokens+i));
			}
			free(tokens);
			
			if(operation == STOP)
			{
				handled = 1;
				_pulseCountComparitor = 0;
				_pulseCount = 0;
				allStop();
			}
			
			if(operation == FORWARD)
			{
				handled = 1;
				if(val == 0){
					// TODO: This CURRENTLY limits forward direction to 35 cm - need to change to 'infinite' (i.e. keep going until told otherwise)
					_pulseCountComparitor = SINGLE_REVOLUTION_FWD;
					} else {
					_pulseCountComparitor = getPulseComparitorForDistance(val);
				}
				_pulseCount = 0;
				forward();
			}
			
			if(operation == REVERSE)
			{
				handled = 1;
				if(val == 0){
					_pulseCountComparitor = SINGLE_REVOLUTION_REV;
					} else {
					_pulseCountComparitor = getPulseComparitorForDistance(val);
				}
				_pulseCount = 0;
				reverse();
			}

			if(operation == TURNRIGHT)
			{
				handled = 1;
				turnRightBy(val);
				_pulseCount = 0;
			}
			if(operation == TURNLEFT)
			{
				handled = 1;
				turnLeftBy(val);
				_pulseCount = 0;
			}
			} else {
			uart_puts("bad command :");
			uart_puts(commandBuffer);
			send_string_SPI("bad command\n");
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
	}	
}

void startUpIndicator(void) {

	OUTPUT_PORT |= ACTIVE_LED_MASK;
	_delay_ms(1000);
	OUTPUT_PORT &= ~ACTIVE_LED_MASK;
	_delay_ms(1000);
	OUTPUT_PORT |= ACTIVE_LED_MASK;
	_delay_ms(1000);
	OUTPUT_PORT &= ~ACTIVE_LED_MASK;
	_delay_ms(1000);
	OUTPUT_PORT |= ACTIVE_LED_MASK;
	_delay_ms(1000);
	OUTPUT_PORT &= ~ACTIVE_LED_MASK;
	_delay_ms(1000);
	OUTPUT_PORT |= ACTIVE_LED_MASK;
	_delay_ms(1000);
	
	OUTPUT_PORT |= PORT_LED_MASK | STARBOARD_LED_MASK;
	_delay_ms(1000);
	OUTPUT_PORT &= ~PORT_LED_MASK;
	OUTPUT_PORT &= ~STARBOARD_LED_MASK;
}

void init_OutputPins(void){	
	OUTPUT_DDR |= PORT_LED_MASK | STARBOARD_LED_MASK | ACTIVE_LED_MASK | DEBUG_PIN_MASK;
	CONTROL_DDR |= PORT_MOTOR |STARBOARD_MOTOR | CMD_COMPLETE_PIN_MASK;
}

void init_hwInterrupts(void){
	// h/w interrupts on INT0 and INT1
	DDRD &= ~(1 << DDD2);		// Clear the PD2 & PD3 pin & enable PD2 & PD3 as input
	DDRD &= ~(1 << DDD3);
	PORTD |= (1 << PORTD2) | (1 << PORTD3);
	//trigger INT0, INT1 on rising edge
	EICRA |= (1 << ISC11) | (1<< ISC01);
	EIMSK |= (1 << INT0) | (1 << INT1);
}

void init_timer0(void){
	//timer 0 for 4 ms
	// Set the Timer Mode to CTC
	TCCR0A |= (1 << WGM01);
	OCR0A = 0xF9;	//249
	TCCR0B |= (1 << CS02);
	TIMSK0 |= (1 << OCIE0A);    //Set the ISR COMPA vect
		
	//TCCR0B |= (0 << CS01) | (0 << CS00);  // sets prescaler to 64 and stops the timer
	// NOTE: Timer0 NOT currently needed
	_timer0_accum = 0;
	TCCR0B |= (1 << CS02); // set prescaler to 256 and start the timer
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

void init_uart(void){
	/*Set baud rate */
	UBRR0H = (MYUBRR >> 8);
	UBRR0L = MYUBRR;
	
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);      // Enable receiver and transmitter
	UCSR0B |= (1 << RXCIE0);                    // Enable reciever interrupt
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);    // Set frame: 8data, 1 stp

	uartBufferIdx = 0;
	memset(uartBuffer, 0, sizeof(uartBuffer));
	//sei();                                      // interrups are enabled up in main loop
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
	_delay_ms(500);
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
	OUTPUT_PORT &= ~PORT_LED_MASK;
	OUTPUT_PORT &= ~STARBOARD_LED_MASK;
	
	uart_puts("motors stop");
	TX_NEWLINE;
	lcdClearAndHome();
	send_string_SPI("motors stop\n");
	_delay_ms(100);
	lcdGoTo(0, 1);
	send_string_SPI("H:\n");
	
	OCR1AH=0x0B;
	OCR1AL=0xB0;
	OCR1BH=0x0B;
	OCR1BL=0xB0;
}

void forward() {
	OUTPUT_PORT  |= PORT_LED_MASK | STARBOARD_LED_MASK;
	
	uart_puts("motors forward");
	TX_NEWLINE;
	lcdClearAndHome();
	send_string_SPI("motors forward\n");
	_delay_ms(100);
	lcdGoTo(0, 1);
	send_string_SPI("H:\n");
	
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
	lcdClearAndHome();
	send_string_SPI("motors reverse\n");
	_delay_ms(100);
	lcdGoTo(0, 1);
	send_string_SPI("H:\n");
	
	OCR1AH=0x0E;
	OCR1AL=0x10;
	OCR1BH=0x0E;
	OCR1BL=0x10;
}

void turnLeft(void){
	OUTPUT_PORT |= STARBOARD_LED_MASK;
	OCR1AH=0x0E;
	OCR1AL=0x10;
	OCR1BH=0x09;
	OCR1BL=0x60;
}

void turnRight(void){
	OUTPUT_PORT  |= PORT_LED_MASK;
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

void checkTurnBearing(uint8_t turnDirection) {
	
	char buffer[12];	char heading[4];	char target[4];
	
	int16_t	degreesToTurn = 0;
	
	int16_t sample1 = getHeadingInt();
	int16_t sample2 = getHeadingInt();
	int16_t sample3 = getHeadingInt();
	
	int16_t headingVal = (sample1 + sample2 + sample3)/3;
	
	if(turnDirection == TURNRIGHT){
		if(headingVal <= _targetBearing)
		{
			degreesToTurn = _targetBearing - headingVal;
		} else {
			degreesToTurn = (360-headingVal)+_targetBearing;
		}
	} else {
		//turn is left
		if(headingVal >= _targetBearing)
		{
			degreesToTurn = headingVal - _targetBearing;
		} else {
			degreesToTurn = 360 - (_targetBearing - headingVal);
		}
	}
	
	dec_to_str(target,_targetBearing, 3);
	dec_to_str(heading,headingVal, 3);	
	if(turnDirection == TURNRIGHT){
		strncpy(buffer, heading, 3);
		strncpy(buffer+3, " -> ", 4);
		strncpy(buffer+7, target, 3);
		buffer[11] = '\n';
	} else {
		strncpy(buffer, target, 3);
		strncpy(buffer+3, " <- ", 4);
		strncpy(buffer+7, heading, 3);
		buffer[11] = '\n';
	}
	
	if(abs(degreesToTurn) <=  BEARING_TOLERANCE)
	{
		// STOP the turn
		allStop();
		_currentCommand = STOP;
		signalCommandComplete();
		buffer[0] = ' ';
		buffer[1] = ' ';
		buffer[2] = ' ';
		buffer[3] = ' ';
		buffer[4] = ' ';
		buffer[5] = ' ';
		buffer[6] = ' ';
		buffer[7] = ' ';
		buffer[8] = ' ';
		buffer[9] = ' ';
		buffer[10] = ' ';
		buffer[11] = '\n';
	}
	
	lcdGoTo(7, 2);
	send_string_SPI(buffer);
	_delay_ms(100);
}

void turnRightTo(int16_t targetBearing){
	
	uart_puts("	-> turnRightTo");
	TX_NEWLINE;
	
	_targetBearing = targetBearing;
	
	int16_t sample1 = getHeadingInt();
	int16_t sample2 = getHeadingInt();
	int16_t sample3 = getHeadingInt();
	int16_t headingVal = (sample1 + sample2 + sample3)/3;
	
	uart_puts("Turning right ...");
	TX_NEWLINE;
	lcdClearAndHome();
	_delay_ms(100);
	send_string_SPI("Turning right ...\n");
	_delay_ms(100);
	lcdGoTo(0, 1);
	send_string_SPI("H:\n");
	_delay_ms(500);
	
	int16_t degreesToTurn = 0;
	if(headingVal <= targetBearing)
	{
		degreesToTurn = targetBearing - headingVal;
		} else {
		degreesToTurn = (360-headingVal)+targetBearing;
	}
	
	if(abs(degreesToTurn) >= BEARING_TOLERANCE){
		_currentCommand = TURNRIGHT;
		_degrees = degreesToTurn;
		
		//start the turn
		turnRight();
	}
	
	uart_puts("	<- turnRightTo");
	TX_NEWLINE;
}

void turnLeftTo(int16_t targetBearing){
	
	uart_puts("	-> turnLeftTo");
	TX_NEWLINE;
	
	_targetBearing = targetBearing;
	
	int16_t sample1 = getHeadingInt();
	int16_t sample2 = getHeadingInt();
	int16_t sample3 = getHeadingInt();
	int16_t headingVal = (sample1 + sample2 + sample3)/3;
	
	uart_puts("Turning left ...");
	TX_NEWLINE;
	lcdClearAndHome();
	send_string_SPI("Turning left ...\n");
	_delay_ms(100);
	lcdGoTo(0, 1);
	send_string_SPI("H:\n");
	_delay_ms(500);
	
	int16_t degreesToTurn = 0;
	if(headingVal >= targetBearing)
	{
		degreesToTurn = headingVal - targetBearing;
		} else {
		degreesToTurn = 360 - (targetBearing - headingVal);
	}
	
	if(abs(degreesToTurn) >= BEARING_TOLERANCE){
		_currentCommand = TURNLEFT;
		_degrees = degreesToTurn;
		
		//start the turn
		turnLeft();
	}
	
	uart_puts("	<- turnLeftTo");
	TX_NEWLINE;
}

void turnTo(int16_t targetBearing){
	
}

int16_t getPulseComparitorForDistance(int distanceInDecimeters){
	int16_t revsPerCentimetre = (int16_t)(SINGLE_REVOLUTION_FWD / MEASURED_DISTANCE_PER_REVOLUTION);
	return revsPerCentimetre * (distanceInDecimeters * 10);
}

void signalCommandComplete(void){
	CONTROL_PORT |= CMD_COMPLETE_PIN_MASK;
	_delay_ms(100);
	CONTROL_PORT &= ~CMD_COMPLETE_PIN_MASK;
	
	OUTPUT_PORT &= ~ACTIVE_LED_MASK;
}

ISR (INT0_vect)
{
	if( (PIND & (1<<PIND2)) == 0)    //is the pin set
	{
		_delay_us(55);  //transient spikes are noticed of around 20-50 microseconds when watching on logic analyzer - 55us to smooth/remove these
		if( (PIND & (1<<PIND2)) == 0) // is the pin still set
		{
			if(_pulseCountComparitor > 0){
				_pulseCount += 1;
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
	//event to be executed every 4ms here
	_timer0_accum += 1;
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


void lcdClearAndHome() {
	char lcdClear[4];
	lcdClear[0] = '~';
	lcdClear[1] = 'C';
	lcdClear[2] = '\n';
	
	char lcdHome[5];
	lcdHome[0] = '~';
	lcdHome[1] = 'G';
	lcdHome[2] = LCD_POS_BASE + 0;
	lcdHome[3] = LCD_POS_BASE + 0;
	lcdHome[4] = '\n';
	
	send_string_SPI(lcdClear);
	_delay_ms(100);
	send_string_SPI(lcdHome);
	_delay_ms(100);
}

void lcdGoTo(uint8_t x, uint8_t y) {
	char lcdPosition[5];
	lcdPosition[0] = '~';
	lcdPosition[1] = 'G';
	lcdPosition[2] = LCD_POS_BASE + x;
	lcdPosition[3] = LCD_POS_BASE + y;
	lcdPosition[4] = '\n';
	
	send_string_SPI(lcdPosition);
	_delay_ms(100);
}

void displayCurrentBearing(void) {	
	int16_t mxraw = 0;
	int16_t myraw = 0;
	int16_t mzraw = 0;
	double mx = 0;
	double my = 0;
	double mz = 0;
	char itmp[10];
	
	// read of compass for diagnostics
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
	
	if(_currentCommand != TURNLEFT && _currentCommand != TURNRIGHT){
		lcdGoTo(3, 1);
		send_string_SPI(distStr);
	}
	
}



