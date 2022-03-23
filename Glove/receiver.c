// Final fucking attempt!!
// 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <util/delay.h>
#include "usb_mouse.h"

#define BAUDRATE 57600		// Baud rate for the radio transmitter
#define UBRRVAL ((F_CPU/(BAUDRATE*16UL))-1)		// Calculate the baud rate setting
#define SYNC 0XFF	// SYNC byte for packet
#define ADDR 0xAA	// Address byte for packet

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

// LED configuration and on/off for DEBUGGING and shit!
#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#define LED_CONFIG	(DDRD |= (1<<6))

volatile unsigned char sync;	// SYNC byte
volatile unsigned char addr;	// ADDRRESS byte
volatile unsigned char xAxis;	// X Axis of the mouse
volatile unsigned char yAxis;	// Y Axis of the mouse
volatile unsigned char wheel;	// Scroll position
volatile unsigned char click;	// Information for mouse button clicks

volatile unsigned char rapid_fire;	// Flag for rapid fire clicks
volatile unsigned char firetime;	// Timeout counter for rapid fire
volatile unsigned char step_x;		// Step for x axis movement
volatile unsigned char step_y;		// Step for y axis movement

/* Initialize UART: This shit better work mofo
 *		Set up the baud rate and USART settings for data transmission. */
void uart_init(void)
{
	// Baud rate setting
	UBRR1L = (uint8_t) UBRRVAL;				// Low byte cuz thats how we roll
	UBRR1H = (uint8_t) (UBRRVAL>>8);		// High byte cuz  I'm always high....
	
	// Enable Receiver and Interrupt on receive complete
	UCSR1B = (1<<RXEN1) | (1<<RXCIE1);
	
	// USART setting: SPI mode, odd parity check enabeld, 2-bit stop bits, and 9-bit data size (kids stuff)
	UCSR1C = (0<<UMSEL10) | (0<<UMSEL11) | (0<<UPM11) | (0<<UPM10) | (0<<USBS1) | (3<<UCSZ10);
}

/* Receive a byte from the transceiver:
 *		Load a byte from UDR. */
uint8_t uart_getchar(void)
{
	// Wait until the USART Receive Complete and then return the data from UDR this takes time lol
	while (!(UCSR1A & (1 << RXC1)));
	return UDR1;
}

/*  USART Receive Complete Interrupt Serivce Routine:
 *		Interrupt occurs when USART recieves data.
 *		Decode the recieved packets and perform action on the mouse cursor. */
ISR(USART1_RX_vect)
{
    // Receive messages in order if sync byte matches
	sync = uart_getchar();
    if (sync == SYNC)
	{
		// Receive further messages if the address matches
		addr = uart_getchar();
		if (addr == ADDR)
		{
			xAxis = uart_getchar();		// Cursor movement on x axis
			yAxis = uart_getchar();		// Cursor movement on y axis
			wheel = uart_getchar();		// Scroll movement
			click = uart_getchar();		// Click Status

	    	// If Port C.7 button is pressed (Left Click)
		 	if ((click & 0x80) == 0x80) {
				LED_ON;
				usb_mouse_buttons(1, 0, 0);
			}
			// If Port C.6 button is pressed (Middle Click)
			else if ((click & 0x40) == 0x40) {
				LED_ON;
				usb_mouse_buttons(0, 1, 0);
			}
			// If Port C.5 button is pressed (Right Click)
			else if ((click & 0x20) == 0x20) {
				LED_ON;
				usb_mouse_buttons(0, 0, 1);
			}
			// If Port C.7 button is released (Left Click)
			else if ((click & 0x01) == 0x01) {
				LED_OFF;
				usb_mouse_buttons(0, 0, 0);
			}
			// If Port C.6 button is released (Middle Click)
			else if ((click & 0x02) == 0x02) {
				LED_OFF;
				usb_mouse_buttons(0, 0, 0);
			}
			// If Port C.5 button isreleased (Right Click)
			else if ((click & 0x04) == 0x04) {
				LED_OFF;
				usb_mouse_buttons(0, 0, 0);
			}
			// If Port C.7 button is pressed in rapid fire mode (Repeated Left Clicks)
			else if ((click & 0x08) == 0x08) {
				LED_ON;
				if (rapid_fire == 1) rapid_fire = 0;
				else rapid_fire = 1;
				usb_mouse_buttons(rapid_fire, 0, 0);
			}

			// Move the mouse cursor
			usb_mouse_move(xAxis, yAxis, wheel);
		}
	}
}

// Finally solved this shit!!
int main(void)
{
	// Set CPU to run at 16 MHz. Thats like warp speed  Mr Spock
	CPU_PRESCALE(0);
	
	// Set up on-board LED
	LED_CONFIG;
	LED_OFF;

	// Initialize the usb interface and USART interface
	usb_init();
	while (!usb_configured())
	_delay_ms(1000);
	uart_init();

	// Initialize flag for rapid fire mode! Fuck yea mf
	rapid_fire = 0;

	// Enable interrupt
    sei();

	// An infinite while loop
	while (1) 
	{
	}
}
