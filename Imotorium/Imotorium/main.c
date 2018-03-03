
#define F_CPU 16000000UL

#define UART_BAUD_RATE	9600

#define TX_NEWLINE {uart_putc(0x0d); uart_putc(0x0a);}

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>

#include "MotorControl.h"

#define FOSC 16000000                       // Clock Speed
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD -1)

#define LED_PORT	PORTC
#define LED_DDR		DDRC
#define ACTIVE_LED_MASK		(1<<PC2)

#define DEBUG_PIN_MASK		(1<<PC3)

// UART control
void uart_init(void);
void uart_puts(const char *s );
void uart_putc(unsigned char data);

int uartBufferIdx;
char uartBuffer[20];

volatile uint8_t commandFlag;
volatile uint8_t commandIdx;

int main(void)
{	
	LED_DDR |= ACTIVE_LED_MASK | DEBUG_PIN_MASK;
	
	_delay_ms(10000);
	LED_PORT |= ACTIVE_LED_MASK;
	_delay_ms(1000);
	LED_PORT &= ~ACTIVE_LED_MASK;
	_delay_ms(1000);
	
	uart_init();
	
	LED_PORT |= ACTIVE_LED_MASK;
	_delay_ms(1000);
	
	// h/w interrupts on INT0 and INT1
	DDRD &= ~(1 << DDD2);		// Clear the PD2 & PD3 pin & enable PD2 & PD3 as input
	DDRD &= ~(1 << DDD3);
	PORTD |= (1 << PORTD2) | (1 << PORTD3);

	//trigger INT0, INT1 on rising edge
	EICRA |= (1 << ISC11) | (1<< ISC01);
	EIMSK |= (1 << INT0) | (1 << INT1);
	
	commandFlag = 0;
	commandIdx = 0;	
	
	sei(); 
	
	LED_PORT |= DEBUG_PIN_MASK;	
	
	//start the first command straight away
	
	uartBuffer[0] = FORWARD;
	uartBuffer[1] = ':';
	uartBuffer[2] = '0';
	uartBuffer[3] = '0';	
	uartBuffer[4] = '\n';
	uart_puts(uartBuffer);
	
    while (1) 
    {
		if(commandFlag == 1){
			// time to send new command to motorium
			commandFlag = 0; //reset first
			if(commandIdx == 1){
				uartBuffer[0] = TURNRIGHT;
				uartBuffer[1] = ':';
				uartBuffer[2] = '4';
				uartBuffer[3] = '5';
				uartBuffer[4] = '\n';
				uart_puts(uartBuffer);
			}
			
			if(commandIdx == 2){
				uartBuffer[0] = TURNLEFT;
				uartBuffer[1] = ':';
				uartBuffer[2] = '4';
				uartBuffer[3] = '5';
				uartBuffer[4] = '\n';
				uart_puts(uartBuffer);
			}
			
			if(commandIdx == 3){
				uartBuffer[0] = REVERSE;
				uartBuffer[1] = ':';
				uartBuffer[2] = '0';
				uartBuffer[3] = '0';
				uartBuffer[4] = '\n';
				uart_puts(uartBuffer);
			}
			
			if(commandIdx == 4){
				uartBuffer[0] = TURNRIGHT;
				uartBuffer[1] = ':';
				uartBuffer[2] = '9';
				uartBuffer[3] = '0';
				uartBuffer[4] = '\n';
			}
			
			if(commandIdx == 5){
				uartBuffer[0] = TURNLEFT;
				uartBuffer[1] = ':';
				uartBuffer[2] = '9';
				uartBuffer[3] = '0';
				uartBuffer[4] = '\n';
			}
			
			if(commandIdx == 6){
				uartBuffer[0] = FORWARD;
				uartBuffer[1] = ':';
				uartBuffer[2] = '1';
				uartBuffer[3] = '0';
				uartBuffer[4] = '\n';
			}
		}
    }
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
                                     // Lets not forget to enable interrupts =P
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

ISR (USART_RX_vect)
{
	char receivedChar;
	receivedChar = UDR0;                       // Read data from the RX buffer
	
	//UDR0 = receivedChar;                       // Write the data to the TX buffer
}

ISR (INT0_vect)
{
	if( (PIND & (1<<PIND2)) == 0)    //is the pin set
	{
		_delay_ms(50);
		if( (PIND & (1<<PIND2)) == 0) // is the pin still set
		{
			commandFlag = 1;
			commandIdx = commandIdx+1;			
		}
		else
		{
			// do nothing because the input is invalid
		}
	}
}

ISR (INT1_vect)
{
	if( (PIND & (1<<PIND3)) == 0)    //is the pin set
	{
		_delay_ms(50);
		if( (PIND & (1<<PIND3)) == 0) // is the pin still set
		{

		}
		else
		{
			// do nothing because the input is invalid
		}
	}
}

