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

enum MODES {
	INTERVALOMETER = 0,
	REMOTE = 1
} mode = REMOTE;

enum CAMERA_TYPES {
	NIKON = 0,
	CANON = 1
} camera_type = NIKON;


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
	result = result + (temp << 8);

	// set ADSC bit to get the *next* conversion started
	ADCSRA |= (1<<ADSC);
	return result;
}


long int get_interval() {
	double sample;
	long int interval;
	uint8_t i, sample_avg;

	// take 100 samples and average them
	for (i=0; i<100; i++) {
		sample = adc_read();
		// add this contribution to the average
		sample_avg = sample_avg + (sample / 100);
	}

	//Decide on the interval time.  The Nerdkits crystal makes a clock that is a tiny bit
	//slow (1 usec = 1.017 usec), so I adjusted the interval times accordingly.
	if (sample <= 128) 
		interval = 14000; 	// 15 sec
	 else if (sample <= 256) 
		interval = 28000; 	// 30 sec
	 else if (sample <= 384) 
		interval = 58997; 	// 1 min
	 else if (sample <= 512) 
		interval = 117994; 	// 2 min
	 else if (sample <= 640) 
		interval = 176991; 	// 3 min
	 else if (sample <= 768) 
		interval = 294985; 	// 5 min
	 else if (sample <= 896) 
		interval = 589970; 	// 10 min
	 else 
		interval = 884955; 	// 15 min
	
	return interval;
}


void pulse_32k(){
	//this function pulses the LED at 32.7KHz
	// [ 1/ 32700 = 3.0581EE-5 ]
	// [ 3.0581EE-5 = 0.000030581 ]
	// [ 30.581us ~= 31us ]
	// [increase 31us to 32us to divide by 2 evenly ]
	// [ 32us / 2 = 16us ]
	PORTC |= (1<<PC4);		//ON
	delay_us(16);
	PORTC &= ~(1<<PC4);	//OFF
	delay_us(16);
}

void pulse_38k() {
	//this function pulses the LED once at 38.4KHz
	// [ 1 / 38400 = 2.60416666EE-5s]
	// [ 2.6041666EE-5s ~= 26us ]
	// [ 26us / 2 = 13 us]
	PORTC |= (1<<PC4);		//ON
	delay_us(13);
	PORTC &= ~(1<<PC4);	//OFF
	delay_us(13);
}

void pulse_38k_ms(uint16_t us) { 	//max us=65535 (2 ^ 16)
	uint16_t times = us / 26; 		// one pulse takes 26us -> 13us on and 13us off
	uint16_t i;
	for (i = 0; i < times; i++)
		pulse_38k();
}


void nikon_click() {
	//send the shutter release signal to the camera [---NIKON CAMERAS---]
	PORTC |= (1<<PC3); // turn on indicator LED
	
	// Fire Pattern Twice with IR LED
	uint8_t j;
	for (j = 0; j < 2; j++) {
		//MCU timer is slow 1us (MCU) = 1.017us (ACTUAL) --
		pulse_38k_ms(2002);   	//Wait for a start pulse (2000 usec)  [2000 / 26 = 76.9 -> 2002 / 26 = 77]
		delay_us(27830); 		//- There must be no pulse for 27830 usec (pause)
		pulse_38k_ms(390);   	//- Receive a pulse (390 usec) [390 / 26 = 15]
		delay_us(1580);  		//- Pause (1580 usec)
		pulse_38k_ms(416);   	//- Receive a second pulse (410 usec) [410 / 26 = 15.7 -> 416 / 26 = 16]
		delay_us(3580);  		//- Longer pause (3580 usec)
		pulse_38k_ms(390);   	//- Receive the last pulse (400 usec) [400 / 26 = 15.3 -> 390 / 26 = 15]
		delay_us(63200); 		//The same waveform is repeated a second time after about 63,2 msec.
	}
	PORTC &= ~(1 << PC3); //turn off indicator LED
}

void canon_click(){
	//send the shutter release signal to the camera [---CANON CAMERAS---]
	//This pattern is untested because I didn't have a Canon camera to use, but according to online documentation this *should* work for a Canon camera that has IR remote capabilities.
	PORTC |= (1<<PC3); // turn on indicator LED
	
	uint8_t j, k;
	// Fire Pattern Twice with IR LED
	for (j = 0; j < 2; j++) {
		for (k = 0; k < 16; k++) pulse_32k();	//16 pulses @ 32KHz
		delay_ms(7.21); 					// pause 7.21 ms
		for (k = 0; k < 16; k++) pulse_32k();	//16 pulses @ 32KHz
	}
	PORTC &= ~(1<<PC3); //turn off indicator LED
}

void click() {
	if (camera_type == NIKON) 
		nikon_click();
	else if (camera_type == CANON)
		canon_click();
}


int main() {
	long int timer_interval;

	// LEDs as outputs
	DDRC |= (1<<PC4);	//IR LED
	DDRC |= (1<<PC3);	//Indicator LED

	//enable internal pullup resistors
	PORTC |= (1<<PC5);	// button
	PORTC |= (1<<PC2); // mode switch
	PORTC |= (1<<PC0); // interval potentiometer

	adc_init();

	//start up the serial port
	uart_init();
	FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	stdin = stdout = &uart_stream;
	//allow time to initialize
	delay_ms(100);
	
	printf_P(PSTR("STARTING\n"));

	// loop forever!
	while (1) {
		
		//Wait for button press
		if ((PINC & (1<<PC5)) == 0) {
			
			//Check mode switch
			if ((PINC & (1<<PC2)) == 0)
				mode = REMOTE;
			 else 
				mode = INTERVALOMETER;
			

			if (mode == REMOTE) {
				click();
				printf_P(PSTR("CLICK_REMOTE\n"));

				//debounce
				delay_ms(1000);
			
			} else if (mode == INTERVALOMETER) {
				printf_P(PSTR("INTERVALOMETER_START\n"));
				mode = INTERVALOMETER;
				
				timer_interval = get_interval();

				printf_P(PSTR("TIMER_INTERVAL = %u ms\n"), timer_interval);
				
				while (1) {
					click();
					printf_P(PSTR("CLICK_INTERVALOMETER\n"));
					
					//printf_P(PSTR("Waiting for %u ms\n"), timer_interval);
					if((timer_interval / 1000) <60) {
						printf_P(PSTR("Waiting for %u seconds\n"), timer_interval / 1000);
					} else {
						printf_P(PSTR("Waiting for %u minutes\n"), (timer_interval / 1000) / 60);
					}	
					delay_ms(timer_interval);
				}
			}
		}
	}
	return 0;
}
