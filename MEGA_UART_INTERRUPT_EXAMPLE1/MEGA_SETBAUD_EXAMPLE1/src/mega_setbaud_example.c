/**
 * \mainpage
 * \section board STK600 development board
 * \section intro Introduction
 * This example demonstrates how to use the avr libc utility setbaud to find
 * the correct settings for the UART baudrate registers.
 *
 * \section files Files:
 * - mega_setbaud_example.c: megaAVR STK600 setbaud example application
 *
 * \section exampledescription Brief description of the example application
 * This application will set up the UART according to the settings in the
 * conf_uart.h file, baudrate is calculated using the avr libc setbaud utility.
 * When initialization of the UART is done the application sends the letter 'A'
 * on the UART and expect to receive the same letter, hence the hardware must
 * be set up so that the TX and RX pin is shorted to create a loop back.
 *
 * The util/setbaud tool is a tool offered by the avr libc library, and is
 * a function for compile time calculation of the baudrate register of values.
 * In application where run-time baudrate change is not needed this is a tool
 * which easily calculates the best baudrate register settings while keeping the
 * flash footprint at a minimum.
 *
 * A common way of using the setbaud tool is:
 * \code
 * // Define the CPU clock frequency, e.g. 1MHz
 * #define F_CPU 1000000UL
 * // Define the target baudrate, e.g. 9600bps
 * #define BAUD 9600
 * // Set the accepted tolerance, e.g. 2%
 * #define BAUD_TOL 2
 *
 * // Load the calculated values into the correct registers
 * UBRR0H = UBRRH_VALUE;
 * UBRR0L = UBRRL_VALUE;
 *
 * //Then we need to take into account that we may need to set the 2X bit in
 * //the UART control register to achieve the correct baudrate.
 * #ifdef USE_2X
 * UCSR0A |= (1 << U2X0);
 * #endif
 *
 * // The last thing that needs to be done is to enable output on the TX pin,
 * // input on the RX pin and enable the RX and TX in the UART itself. In
 * // addition to setting the correct mode and bit length, parity and stop-bits.
 * \endcode
 *
 * \section compinfo Compilation Info
 * This software was written for the <A href="http://gcc.gnu.org/">GNU GCC</A>
 * for AVR. \n
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 */
#define _ASSERT_ENABLE_
#include "compiler.h"

#define TIMEOUT 250

// Set the correct BAUD and F_CPU defines before including setbaud.h
#include "conf_clock.h"
#include "conf_uart.h"
#include <util/setbaud.h>
#include <util/delay.h>
#include <avr/sleep.h> //used to put device to sleep during waiting
#include <avr/interrupt.h>

//used to print strings
#include "string.h"

//Pin 23 on ATMega328p
#define ADC_PIN			0
#define MAXNUMOFDATA	5

//register used as our variable
volatile int incrementer;
volatile int counter;
int inter, dataNum;
int dataBuffer[MAXNUMOFDATA];

static char buffer[16];

static char stringOut[20] = "Hello World!";

//setup the timer interrupt
static void init_timer_interrupt(void) {
	//TCCR2A &= 0xFE; //Make sure we are using CTC mode (reset timer when matches)
	//TCCR2A |= 0x02; 
	
	TCCR2B &= 0xF5; //Make sure we are using CTC mode and using 32K/128 prescaler
	TCCR2B |= 0x05;
	
	//TCCR2B &= 0xF7; //Make sure we are using CTC mode and using 32K/128
	//TCCR2B |= 0x07;
	
	TIMSK2 = _BV(TOIE2); //enable to overflow
	ASSR |= 0x20; //clocked using external 32Khz clock
	sei();
	//OCR2A = 128; //1 sec interrupt
}

/**
 * \brief Initialize the uart with correct baud rate settings
 *
 * This function will initialize the UART baud rate registers with the correct
 * values using the AVR libc setbaud utility. In addition set the UART to 8-bit,
 * 1 stop and no parity.
 */
static void uart_init(void)
{
#if defined UBRR0H
	/* These values are calculated by the setbaud tool based on the values
	defined in conf_clock.h and conf_uart.h. The only thing that the application
	need to do is to load these values into the correct registers.*/
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
#else
#error "Device is not supported by the driver"
#endif

	/* Check if the setbaud tool require that the 2x speed bit has to be set in
	order to reach the specified baudrate. */
#if USE_2X
	UCSR0A |= (1 << U2X0);
#endif

	// Enable RX and TX and set up port
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	// Set the TX pin as output
	UART_PORT_DDR |= (1 << UART_TX_PIN);
	// Set the RX pin as input
	//UART_PORT_DDR &= ~(1 << UART_RX_PIN);

	// 8-bit, 1 stop bit, no parity, asynchronous UART
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00) | (0 << USBS0) |
			(0 << UPM01) | (0 << UPM00) | (0 << UMSEL01) |
			(0 << UMSEL00);
}

/**
 * \brief Function for sending a char over the UART
 *
 * \param data the data to send over UART
 */
static void uart_putchar(uint8_t data)
{
	// Make sure that the UART buffer is empty
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}


//configure all ports as input and pull up enable
static void init_Ports(void) {
	PORTD = 0x00;
	DDRD = 0x00; 
	//DDRD = 0xff; //remove comment for output for Port D
	PORTB = 0xff;
	DDRB = 0x00;
	PORTC = 0xff;
	DDRC = 0x00;
	//set PUD to be 0 to enable pull up
	MCUCR &= 0xEF;
}

//adc conversion
static uint16_t adc_read(uint8_t adcx) {
	/* adcx is the analog pin we want to use.  ADMUX's first few bits are
	 * the binary representations of the numbers of the pins so we can
	 * just 'OR' the pin's number with ADMUX to select that pin.
	 * We first zero the four bits by setting ADMUX equal to its higher
	 * four bits. */
	ADMUX	&=	0xf0;
	ADMUX	|=	adcx;
 
	/* This starts the conversion. */
	ADCSRA |= _BV(ADSC);
 
	/* This is an idle loop that just wait around until the conversion
	 * is finished.  It constantly checks ADCSRA's ADSC bit, which we just
	 * set above, to see if it is still set.  This bit is automatically
	 * reset (zeroed) when the conversion is ready so if we do this in
	 * a loop the loop will just go until the conversion is ready. */
	while ( (ADCSRA & _BV(ADSC)) );
 
	/* Finally, we return the converted value to the calling function. */
	return ADC;
}

//Shut down stuff we don't need
static void shutdown_Peripherals(void) {
	//PRR = 0xEC; //only UART and ADC active
	PRR = 0xAD; //only Timer2 and UART active
}

/**
 * \brief Function for getting a char from the UART
 *
 * \note This function is blocking and will expect to receive something
 * on the UART.
 *
 * \retval uint8_t the data received from the UART
 * \retval 0       if timeout
 */
/*static uint8_t uart_getchar(void)
{
	uint8_t timeout = TIMEOUT;
	// Wait for RX to complete
	while ((!(UCSR0A & (1 << RXC0))) && timeout) {
		timeout--;
	}
	if (timeout) {
		return UDR0;
	} else {
		// No data, timeout
		return 0;
	}
}*/


//enable ADC, turns on stuff needed by ADC
static void enableADC(void) {
	PRR &= 0xFE; //turn on ADC power
	/* Enable the ADC */
	ADCSRA |= 0x80;
}

//disable ADC. This also puts the ADC stuff into idle mode
static void disableADC(void) {
	ADCSRA &= 0x7F; //disable ADC
	PRR |= 0x01; //turn off ADC power
}

//turn off XBee
static void turnOFFXBee(void) {
	PORTD |= 1 << PORTD7;
}

static void turnONXBee(void) {
	PORTD &= ~(1 << PORTD7);
}

ISR(TIMER2_OVF_vect) {
	cli();
	//Wake up the device and do ADC
	enableADC();
	dataBuffer[dataNum++] = adc_read(ADC_PIN);
	dataNum = dataNum%MAXNUMOFDATA;
	//disable ADC
	disableADC();
	
	/*if(!dataNum%MAXNUMOFDATA) { //send data
		//turn on the XBee
		turnONXBee();
		_delay_ms(15);
		for(inter = 0; inter < MAXNUMOFDATA; inter++) {
			itoa(dataBuffer[inter], buffer, 10);
			incrementer = 0;
			while(buffer[incrementer] != 0) {
				uart_putchar(buffer[incrementer]);
				incrementer++;
			}
		}
		//turn off the Xbee
		turnOFFXBee();
	}*/
	
	// DO the 1000 cycle calculation
	for(incrementer = 0; incrementer < 1000; incrementer++);
	
	//print the word and counter
	incrementer = 0;
	while(stringOut[incrementer] != 0) {
		uart_putchar(stringOut[incrementer]);
		incrementer++;
	}
	
	itoa(counter, buffer, 10);
	incrementer = 0;
	while(buffer[incrementer] != 0) {
		uart_putchar(buffer[incrementer]);
		incrementer++;
	}
	counter++;
	
	uart_putchar((char) '\r');
	uart_putchar((char) '\n');
	
	//clear the timer interrupt flag
	TIFR2 |= 1;
	sei();
	
	//go back to sleep
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	sleep_enable();
	sleep_bod_disable(); //disable Brown out Detection during sleep
	sleep_cpu(); //sleep the CPU
}

/**
 * \brief Example application on how to use the libc setbaud utility.
 *
 * This application shows how to use the avr libc setbaud utility to get the
 * correct values for the baud rate registers. It also performs a test where
 * it sends a character on the UART and check if the same char is received
 * back.
 *
 * \note This application assumes that the TX and RX pins have been externally
 * shorted to create a loop back.
 */
int main(void)
{
	
	//char stringOut[20] = "Hello World!";
	//char buffer[16];
	int i = 0, j= 0;
	
	counter = 0;
	dataNum = 0;
	
	//setup IOs as input and pull up enable
	//init_Ports();
	//shutdown_Peripherals();
	
	// Set up baud rate registers
	uart_init();
	
	//enable ADC
	enableADC();
	
	//setup the timer interrupt and enable global interrupt
	//cli();
	init_timer_interrupt();
	//sei();
	
	//turnONXBee();
	//_delay_ms(3000);
	//turnOFFXBee();
	
	// Write a set of letters to the UART
	for(;;) {
		//read ADC
		/*adc_read(0);
		
		// DO the 1000 cycle calculation
		for(incrementer = 0; incrementer < 1000; incrementer++);
		
		//print the word and counter
		i = 0;
		while(stringOut[i] != 0) {
			uart_putchar(stringOut[i]);
			i++;
		}
		
		itoa(j, buffer,10);
		i = 0;
		while(buffer[i] != 0) {
			uart_putchar(buffer[i]);
			i++;
		}
		
		uart_putchar('\r');
		uart_putchar('\n');
		
		j++;
	
		//sleep for 1 sec
		_delay_ms(1000);*/
	}
	//uart_putchar('A');
	// Check to see if send and receive works.
	//Assert(uart_getchar() == 'A');

	
	//while (true);
}
