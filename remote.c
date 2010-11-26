// remote.c
// for NerdKits with ATmega168
// use with Nikon (and possibly Canon) cameras
// NOTE: this file best viewed with 1 tab set to 4 spaces

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

#define TRIGGER_PIN 		PC5
#define IR_LED_PIN 			PC4
#define INDICATOR_LED_PIN 	PC3
#define MODE_SWITCH_PIN 	PC2
#define POTENTIOMETER_PIN 	PC0

enum MODES {
	INTERVALOMETER = 0,
	REMOTE = 1
} mode = REMOTE;

enum CAMERA_TYPES {
	NIKON = 0,
	CANON = 1
} camera_type = NIKON;


void adc_init(uint8_t pin) {
	// set analog to digital converter for external reference (5v), single ended input ADC0
	//ADMUX = 0;
	ADMUX = pin;

	// set analog to digital converter to be enabled, with a clock prescale of 1/128
	// so that the ADC clock runs at 115.2kHz.
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	// fire a conversion just to get the ADC warmed up
	ADCSRA |= (1 << ADSC);
}

uint16_t adc_read() {
	// read from ADC, waiting for conversion to finish (assumes someone else asked for a conversion.)
	while (ADCSRA & (1 << ADSC)) {
		// wait for bit to be cleared
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

uint16_t get_sample() {

	uint16_t sample_avg = 0;
	uint16_t sample = 0;
	uint8_t i = 0;
	
	// take 100 samples and average them
	for (i=0; i<100; i++) {
		sample = adc_read();
		sample_avg = sample_avg + (sample / 100);
	}
	return sample_avg;
}

unsigned int account_for_slow_clock(unsigned int ms) {
	//printf_P(PSTR("1Converting %lu ms\r\n"), ms);
	unsigned int us = ms * 1000;	//convert to us
	//printf_P(PSTR("2Converting %lu us\r\n"), us);
	us = us - (us * 0.017);		//account for slow clock
	//printf_P(PSTR("3Converting %lu us\r\n"), us);
	//printf_P(PSTR("4Converting %lu ms\r\n"), us/1000);
	return us / 1000;			//convert back to ms
}


unsigned int get_interval_ms() {
	unsigned int interval = 0;
	uint16_t sample = get_sample();

	//Decide on the interval time.
	if (sample <= 128) 
		interval = 15000u; 	// 15 ses
	 else if (sample <= 256) 
		interval = 30000u; 	// 30 sec
	 else if (sample <= 384) 
		interval = 60000u; 	// 1 min
	 else if (sample <= 512) 
		interval = 120000U; 	// 2 min
	 else if (sample <= 640) 
		interval = 180000u; 	// 3 min
	 else if (sample <= 768) 
		interval = 300000u; 	// 5 min
	 else if (sample <= 896) 
		interval = 600000u; 	// 10 min
	 else 
		interval = 900000u; 	// 15 min

	printf_P(PSTR("Actual Inverval: %lu ms\r\n"), interval);
	 
	//The Nerdkits crystal makes a clock that is a tiny bit slow (1 usec = 1.017 usec)
	interval = account_for_slow_clock(interval);
	 
	printf_P(PSTR("Converted Interval: %lu ms\r\n"), interval);
	return interval;
}


void pulse_32k(){
	//this function pulses the LED once at 32.7KHz
	// [ 1/ 32700 = 3.0581EE-5 ]
	// [ 3.0581EE-5 = 0.000030581 ]
	// [ 30.581us ~= 31us ]
	// [increase 31us to 32us to divide by 2 evenly ]
	// [ 32us / 2 = 16us ]
	PORTC |= (1 << IR_LED_PIN);		//ON
	delay_us(16);
	PORTC &= ~(1 << IR_LED_PIN);	//OFF
	delay_us(16);
}

void pulse_38k() {
	//this function pulses the LED once at 38.4KHz
	// [ 1 / 38400 = 2.60416666EE-5s]
	// [ 2.6041666EE-5s ~= 26us ]
	// [ 26us / 2 = 13 us]
	PORTC |= (1 << IR_LED_PIN);		//ON
	delay_us(13);
	PORTC &= ~(1 << IR_LED_PIN);	//OFF
	delay_us(13);
}

void pulse_38k_ms(uint16_t us) { 	//max us=65535 (2 ^ 16)
	uint16_t times = us / 26; 		// one pulse takes 26us -> 13us on and 13us off
	uint16_t i = 0;
	for (i = 0; i < times; i++)
		pulse_38k();
}

void nikon_click() {
	//send the shutter release signal to the camera [---NIKON CAMERAS---]
	// Fire Pattern Twice with IR LED
	uint8_t j = 0;
	for (j = 0; j < 2; j++) {
		pulse_38k_ms(2002);   	//Wait for a start pulse (2000 usec)  [2000 / 26 = 76.9 -> 2002 / 26 = 77]
		delay_us(27830); 		//- There must be no pulse for 27830 usec (pause)
		pulse_38k_ms(390);   	//- Receive a pulse (390 usec) [390 / 26 = 15]
		delay_us(1580);  		//- Pause (1580 usec)
		pulse_38k_ms(416);   	//- Receive a second pulse (410 usec) [410 / 26 = 15.7 -> 416 / 26 = 16]
		delay_us(3580);  		//- Longer pause (3580 usec)
		pulse_38k_ms(390);   	//- Receive the last pulse (400 usec) [400 / 26 = 15.3 -> 390 / 26 = 15]
		delay_us(63200); 		//The same waveform is repeated a second time after about 63,2 msec.
	}
}

void canon_click(){
	//send the shutter release signal to the camera [---CANON CAMERAS---]
	//This pattern is untested because I didn't have a Canon camera to use, but according to online 
	//documentation this *should* work for a Canon camera that has IR remote capabilities.
	uint8_t i, j = 0;
	// Fire Pattern Twice with IR LED
	for (i = 0; i < 2; i++) {
		for (j = 0; j < 16; j++) pulse_32k();	//16 pulses @ 32KHz
		delay_ms(7.21); 					// pause 7.21 ms
		for (j = 0; j < 16; j++) pulse_32k();		//16 pulses @ 32KHz
	}
}

void click() {
	PORTC |= (1 << INDICATOR_LED_PIN); // turn on indicator LED

	if (camera_type == NIKON)
		nikon_click();
	else if (camera_type == CANON)
		canon_click();

	PORTC &= ~(1 << INDICATOR_LED_PIN); //turn off indicator LED
}

int main() {
	unsigned int timer_interval_ms = 0;

	// LEDs as outputs
	DDRC |= (1 << IR_LED_PIN);
	DDRC |= (1 << INDICATOR_LED_PIN);

	//enable internal pullup resistors
	PORTC |= (1 << TRIGGER_PIN);
	PORTC |= (1 << MODE_SWITCH_PIN);
	PORTC |= (1 << POTENTIOMETER_PIN);

	adc_init(0); //use PC0 for ADC conversion (Potentiometer)
	
	//start up the serial port
	uart_init();
	FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	stdin = stdout = &uart_stream;
	
	printf_P(PSTR("STARTING\r\n"));
	while (1) {
		//Wait for button press
		if ((PINC & (1 << TRIGGER_PIN)) == 0) {
			
			//Check mode switch
			if ((PINC & (1 << MODE_SWITCH_PIN)) == 0)
				mode = REMOTE;
			 else 
				mode = INTERVALOMETER;

			if (mode == REMOTE) {
				click();
				printf_P(PSTR("CLICK_REMOTE\r\n"));

				//debounce
				delay_ms(1000);
			
			} else if (mode == INTERVALOMETER) {
				printf_P(PSTR("INTERVALOMETER_START\r\n"));
				mode = INTERVALOMETER;
				
				timer_interval_ms = get_interval_ms();
				
				while (1) {
					click();
					printf_P(PSTR("CLICK_INTERVALOMETER\r\n"));
					printf_P(PSTR("TIMER_INTERVAL = %lu ms\r\n"), timer_interval_ms);
					
					if ((timer_interval_ms / 1000) < 60) 
						printf_P(PSTR("Waiting for %lu seconds\r\n"), timer_interval_ms / 1000);
					else 
						printf_P(PSTR("Waiting for %lu minutes\r\n"), (timer_interval_ms / 1000) / 60);
					
					uint8_t segments = timer_interval_ms / 65535;
					printf_P(PSTR("Segments %u\r\n"), segments);
					
					uint16_t overflow = timer_interval_ms % 65535;					
					printf_P(PSTR("Overflow %u\r\n"), overflow);
					
					uint8_t i;
					for (i = 0; i < segments; i++) {
						delay_ms(65535);
					}
					delay_ms(overflow);
					//delay_ms(timer_interval_ms);
				}
			}
		}
	}
	return 0;
}
