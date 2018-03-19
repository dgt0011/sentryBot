#ifndef MAIN_H_
#define MAIN_H_

#define OUTPUT_PORT				PORTC
#define OUTPUT_DDR				DDRC

#define CONTROL_PORT			PORTB
#define CONTROL_DDR				DDRB

#define PORT_LED_MASK			(1<<PC0)
#define STARBOARD_LED_MASK		(1<<PC1)
#define ACTIVE_LED_MASK			(1<<PC2)
#define DEBUG_PIN_MASK			(1<<PC3)

#define CMD_COMPLETE_PIN_MASK	(1<<PB0)
#define PORT_MOTOR				(1<<PB1)
#define STARBOARD_MOTOR			(1<<PB2)

#define DELAY					1000

#define FOSC					16000000                       // Clock Speed
#define BAUD					9600
#define MYUBRR					(FOSC/16/BAUD -1)
#define TX_NEWLINE				{uart_putc(0x0d); uart_putc(0x0a);}
	
#define MAGN_DOCALIBRATION 0 //enable or disable magnetometer calibration
#define magncal_getrawdata(mxraw, myraw, mzraw) hmc5883l_getrawdata(mxraw, myraw, mzraw); //set the function that return magnetometer raw values	

#define LCD_POS_BASE			0x60

void init_OutputPins(void);
void init_hwInterrupts(void);
void init_timer0(void);

int16_t getHeadingInt(void);
int16_t getPulseComparitorForDistance(int distanceInDecimeters);
void startUpIndicator(void);
void signalCommandComplete(void);
void checkTurnBearing(uint8_t turnDirection);
void updateState(void);

// diagnostics methods
void lcdClearAndHome(void);
void lcdGoTo(uint8_t x, uint8_t y);
void displayCurrentBearing(void);

// SPI communications
void send_string_SPI(char s[]);
void send_char_SPI(char c);
void init_spi_master(void);

// UART debugging
void init_uart(void);
void uart_puts(const char *s );
void uart_putc(unsigned char data);

const uint8_t BEARING_TOLERANCE = 5;

const int16_t SINGLE_REVOLUTION_FWD = 4850;
const int16_t SINGLE_REVOLUTION_REV = 4850;
const int16_t MEASURED_DISTANCE_PER_REVOLUTION = 35;	//centimeters

const uint8_t TIMER0_TOP = 125;	// 0.5 second for 4ms time period on Timer0

#endif /* MAIN_H_ */