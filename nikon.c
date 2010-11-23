// remote.c
// for NerdKits with ATmega168
// use with Nikon cameras

#define F_CPU 14745600

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include "../libnerdkits/delay.h"
#include "../libnerdkits/uart.h"

// PIN DEFINITIONS:
// PC5 -- trigger button (pulled to ground when pressed)
// PC4 -- IR LED anode
// PC3 -- Indicator LED anode
// PC2 -- Mode switch
// PC0 -- Potentiometer for selecting interval

enum MODES
{
	INTERVALOMETER = 0,
	REMOTE
} mode;

void adc_init() {
	// set analog to digital converter
	// for external reference (5v), single ended input ADC0
	ADMUX = 0;

	// set analog to digital converter
	// to be enabled, with a clock prescale of 1/128
	// so that the ADC clock runs at 115.2kHz.
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);

	// fire a conversion just to get the ADC warmed up
	ADCSRA |= (1<<ADSC);
}

uint16_t adc_read() {
	// read from ADC, waiting for conversion to finish
	// (assumes someone else asked for a conversion.)
	// wait for it to be cleared
	while (ADCSRA & (1<<ADSC)) {
		// do nothing... just hold your breath.
	}
	// bit is cleared, so we have a result.

	// read from the ADCL/ADCH registers, and combine the result
	// Note: ADCL must be read first (datasheet pp. 259)
	uint16_t result = ADCL;
	uint16_t temp = ADCH;
	result = result + (temp<<8);

	// set ADSC bit to get the *next* conversion started
	ADCSRA |= (1<<ADSC);

	return result;
}

int get_interval() {
	double x;
	long int interval;
	uint8_t i, x_avg;

	// take 100 samples and average them
	for (i=0; i<100; i++) {
		x = adc_read();
		// add this contribution to the average
		x_avg = x_avg + x/100;
	}

	//Decide on the interval time.  The Nerdkits crystal makes a clock that is a tiny bit
	//slow (1 usec = 1.017 usec), so I adjusted the interval times accordingly.
	if ( x <= 128 )
		interval = 14000; // 15 sec
	else if ( x <= 256 )
		interval = 28000; // 30 sec
	else if ( x <= 384 )
		interval = 58997; // 1 min
	else if ( x <= 512 )
		interval = 117994; // 2 min
	else if ( x <= 640 )
		interval = 176991; // 3 min
	else if ( x <= 768 )
		interval = 294985; // 5 min
	else if ( x <= 896 )
		interval = 589970; // 10 min
	else
		interval = 884955; // 15 min
	
	return interval;
}

void led_cycle() {
	//this function pulses the LED once at 38.4KHz
	// [ 1 / 38400 = 2.60416666EE-5s]
	// [ 2.6041666EE-5s ~= 26us ]
	// [ 26us / 2 = 13 us]
	PORTC |= (1<<PC4);
	delay_us(13);
	PORTC &= ~(1<<PC4);
	delay_us(13);
}

void pulse_for(int us) {
	int x;
	for (x = 0; x < us; x++)
		led_cycle();
}

void pulse_for2(int us) {
	int x;
	int times = us / 26; // one pulse takes 26us -> 13us on and 13us off
	for (x = 0; x < times; x++)
		led_cycle();
}

void click() {
//send the shutter release signal to the camera [---NIKON CAMERAS---]
	PORTC |= (1<<PC3); // turn on indicator LED
	// Fire Pattern Twice with IR LED
	uint8_t j;
	for ( j = 0; j < 2; j++ )
	{
		//~ //MCU timer is slow 1us (MCU) = 1.017us (ACTUAL)
		//~ pulse_for(77);   	//Wait for a start pulse (2000 usec)  [77 * 13us * 2 = 2002]
		//~ delay_us(27830); 	//- There must be no pulse for 27830 usec (pause)
		//~ pulse_for(15);   	//- Receive a pulse (390 usec) [15 * 13us * 2 = 390]
		//~ delay_us(1580);  	//- Pause (1580 usec)
		//~ pulse_for(16);   	//- Receive a second pulse (410 usec) [16 * 13us * 2 = 416]
		//~ delay_us(3580);  	//- Longer pause (3580 usec)
		//~ pulse_for(15);   	//- Receive the last pulse (400 usec) [15 * 13us * 2 = 390]
		//~ delay_us(63200); 	//The same waveform is repeated a second time after about 63,2 msec.

		//MCU timer is slow 1us (MCU) = 1.017us (ACTUAL) --
		pulse_for2(2002);   	//Wait for a start pulse (2000 usec)  [2000 / 26 = 76.9 -> 2002 / 26 = 77]
		delay_us(27830); 	//- There must be no pulse for 27830 usec (pause)
		pulse_for2(390);   	//- Receive a pulse (390 usec) [390 / 26 = 15]
		delay_us(1580);  	//- Pause (1580 usec)
		pulse_for2(416);   	//- Receive a second pulse (410 usec) [410 / 26 = 15.7 -> 416 / 26 = 16]
		delay_us(3580);  	//- Longer pause (3580 usec)
		pulse_for2(390);   	//- Receive the last pulse (400 usec) [400 / 26 = 15.3 -> 390 / 26 = 15]
		delay_us(63200); 	//The same waveform is repeated a second time after about 63,2 msec.
	}
	PORTC &= ~(1<<PC3); //turn off indicator LED
}

int main() {

	// LEDs as outputs
	DDRC |= (1<<PC4);
	DDRC |= (1<<PC3);

	// enable internal pullup on PC5(button), PC2(mode switch), and PC0(interval potentiometer)
	PORTC |= (1<<PC5);
	PORTC |= (1<<PC2);
	PORTC |= (1<<PC0);

	//Intialize the Analog to Digital Convertor
	adc_init();

	//start up the serial port
	uart_init();
	FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	stdin = stdout = &uart_stream;

	printf_P(PSTR("STARTING\n"));

	long int timer_interval;

	// loop forever!
	while (1) {
		//Wait for button press
		if ((PINC & (1<<PC5)) == 0) {
			
			//Check mode switch
			if ((PINC & (1<<PC2)) == 0)
				mode = REMOTE;
			
			if (mode == REMOTE) {	//Remote Mode
				click();
				printf_P(PSTR("CLICK_REMOTE\n"));
				//debounce
				delay_ms(500);
			} else  {		//Intervalometer Mode
				printf_P(PSTR("INTERVALOMETER_START\n"));
				mode = INTERVALOMETER;
				
				//Check Interval
				timer_interval = get_interval();
				printf_P(PSTR("TIMER_INTERVAL = %u\n"), timer_interval);
				
				while (1) {
					//snap a picture
					click();
					printf_P(PSTR("CLICK_INTERVALOMETER\n"));
					//delay set number of ms
					printf_P(PSTR("Waiting for %u\n"), timer_interval);	
					delay_ms(timer_interval);
				}
			}
		}
	}
	return 0;
}
