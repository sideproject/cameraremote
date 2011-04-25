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
#include <avr/sleep.h>

#include "../libnerdkits/delay.h"
#include "../libnerdkits/uart.h"

void click();

// PIN DEFINITIONS:
#define TRIGGER_PIN 			PD2	//trigger button (pulled to ground when pressed)
#define IR_LED_PIN 				PC4	//IR LED anode
#define IR_INDICATOR_LED_PIN 	PC3	//Intervaolmeter indicator LED - flashes while waiting for next interval click
#define TIMER_INDICATOR_LED_PIN	PC2	//Indicator LED anode (turns on when IR LED is flashing)
#define MODE_SWITCH_PIN 		PC1	//Mode switch
#define POTENTIOMETER_PIN 		PC5	//Potentiometer for selecting interval

enum MODES {
	INTERVALOMETER = 0,
	REMOTE = 1
} mode = REMOTE;

enum CAMERA_TYPES {
	NIKON = 0,
	CANON = 1
} camera_type = NIKON;


// the_time will store the elapsed time in milliseconds (1000 = 1 second)
//
// This variable is marked "volatile" because it is modified
// by an interrupt handler.  Without the "volatile" marking,
// the compiler might just assume that it doesn't change in
// the flow of any given function (if the compiler doesn't
// see any code in that function modifying it -- sounds
// reasonable, normally!).
//
// But with "volatile", it will always read it from memory
// instead of making that assumption.
volatile int32_t timer_interval_ms = 0UL;
volatile uint32_t the_time_ms = 0UL;
volatile uint8_t remainder_counter = 0U;
volatile int32_t count_down_ms = 5000L;
volatile uint8_t fire = 0U;

/*void realtimeclock_setup() {
	// setup Timer0:
	//   CTC (Clear Timer on Compare Match mode)
	//   TOP set by OCR0A register
	TCCR0A |= (1<<WGM01);
	// clocked from CLK/1024
	// which is 14745600/1024, or 14400 increments per second
	TCCR0B |= (1<<CS02) | (1<<CS00);
	// set TOP to 143 because it counts 0, 1, 2, ... 142, 143, 0, 1, 2 ...
	// so 0 through 143 equals 144 events
	OCR0A = 143;
	// enable interrupt on compare event (14400 / 144 = 100 per second)
	TIMSK0 |= (1<<OCIE0A);
}

// when Timer0 gets to its Output Compare value,
// one one-hundredth of a second has elapsed (0.01 seconds).
SIGNAL(SIG_OUTPUT_COMPARE0A) {
	//add 10 instead of 1 because the event happens every .01 seconds and we need to track every .001 seconds
	the_time_ms += 10;
}*/


void adc_init() {
	// set analog to digital converter for external reference (5v), single ended input ADC0
	ADMUX = 5;

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

	uint16_t sample_avg = 0U;
	uint16_t sample = 0U;
	uint8_t i = 0U;

	// take 100 samples and average them
	for (i=0U; i<100U; i++) {
		sample = adc_read();
		sample_avg = sample_avg + (sample / 100U);
	}
	return sample_avg;
}

uint32_t get_interval_ms() {

	uint32_t interval = 0U;
	uint16_t sample = get_sample();

	//Decide on the interval time.
	if (sample <= 128U)
		interval = 15000U; 	// 15 ses
	else if (sample <= 256U)
		interval = 30000U; 	// 30 sec
	else if (sample <= 384U)
		interval = 60000U; 	// 1 min
	else if (sample <= 512U)
		interval = 120000U; 	// 2 min
	else if (sample <= 640U)
		interval = 180000U; 	// 3 min
	else if (sample <= 768U)
		interval = 300000U; 	// 5 min
	else if (sample <= 896U)
		interval = 600000U; 	// 10 min
	else
		interval = 900000U; 	// 15 min

	return interval;
}

void SetupTimer2(){

	//Timer2 Settings: Timer Prescaler /1024, mode 0
	TCCR2A = 0;
	TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20; //set 1024 prescaler
	//TCCR2B = 1<<CS22 | 1<<CS20 | 0<<CS20; //set 256 prescaler
	//TCCR2B = 1<<CS22 | 1<<CS20 | 1<<CS20; //set 128 prescaler
	//TCCR2B = 1<<CS22 | 0<<CS21 | 0<<CS20; //set 64 prescaler
	//TCCR2B = 0<<CS22 | 1<<CS21 | 1<<CS20; //set 32 prescaler
	//TCCR2B = 0<<CS22 | 1<<CS21 | 0<<CS20; //set 8 prescaler

	//Timer2 Overflow Interrupt Enable
	TIMSK2 = 1<<TOIE2;

	//load the timer for its first cycle
	TCNT2=0;
}

//Timer2 overflow interrupt vector handler
ISR(TIMER2_OVF_vect) {
	//prescaler math: http://www.atmel.com/dyn/resources/prod_documents/doc2505.pdf	
	//(14,745,600 / 1024) / 256 = 56.25 (divide by 256 because timer2 is an 8bit timer)
	//there will be 56.25 overflows each second
	//1,000ms / 56.25 overflows = 17.7778ms each overflow
	
	remainder_counter++;
	the_time_ms += 17UL; //we can't add 17.7778
	count_down_ms -= 17L;
	
	//56.25 * 17.7778 = 1000ms
	//for each overflow, we gain 17ms (17 * 56 = 952), we need to make up the difference
	//for each 57th (56.25) overflow, make up the difference of 48ms
	if (remainder_counter >= 57U) {
		the_time_ms += 44UL;
		count_down_ms -= 44L;
		
		remainder_counter = 0U;
	}
}

// this code will be called anytime that PCINT18 switches (hi to lo, or lo to hi)
ISR(PCINT2_vect) {
	if (mode == REMOTE) {
		click();
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		cli();
		sleep_enable();
		sei();
		sleep_cpu();
		sleep_disable();
		sei();
	} else if (mode == INTERVALOMETER) {
		timer_interval_ms = get_interval_ms();
		the_time_ms = 0UL;
		remainder_counter = 0U;
		count_down_ms = 5000UL;
	}
}



void pulse_32k() {
	//this function pulses the LED once at 32.7KHz
	// [ 1/ 32700 = 3.0581EE-5 ]
	// [ 3.0581EE-5 = 0.000030581 ]
	// [ 30.581us ~= 31us ]
	// [increase 31us to 32us to divide by 2 evenly ]
	// [ 32us / 2 = 16us ]
	PORTC |= (1 << IR_LED_PIN);		//ON
	delay_us(16U);
	PORTC &= ~(1 << IR_LED_PIN);	//OFF
	delay_us(16U);
}

void pulse_38k() {
	//this function pulses the LED once at 38.4KHz
	// [ 1 / 38400 = 2.60416666EE-5s]
	// [ 2.6041666EE-5s ~= 26us ]
	// [ 26us / 2 = 13 us]
	PORTC |= (1 << IR_LED_PIN);		//ON
	delay_us(13U);
	PORTC &= ~(1 << IR_LED_PIN);	//OFF
	delay_us(13U);
}

void pulse_38k_us(uint16_t us) { 	//max us=65535 (2 ^ 16)
	uint16_t times = us / 26U; 		// one pulse takes 26us -> 13us on and 13us off
	uint16_t i = 0U;
	for (i = 0U; i < times; i++)
		pulse_38k();
}

void nikon_click() {
	//send the shutter release signal to the camera [---NIKON CAMERAS---]
	// Fire Pattern Twice with IR LED
	uint8_t j = 0U;
	for (j = 0U; j < 2U; j++) {
		pulse_38k_us(2002U);   	//Wait for a start pulse (2000 usec)  [2000 / 26 = 76.9 -> 2002 / 26 = 77]
		delay_us(27830U); 		//- There must be no pulse for 27830 usec (pause)
		pulse_38k_us(390U);   	//- Receive a pulse (390 usec) [390 / 26 = 15]
		delay_us(1580U);  		//- Pause (1580 usec)
		pulse_38k_us(416U);   	//- Receive a second pulse (410 usec) [410 / 26 = 15.7 -> 416 / 26 = 16]
		delay_us(3580U);  		//- Longer pause (3580 usec)
		pulse_38k_us(390U);   	//- Receive the last pulse (400 usec) [400 / 26 = 15.3 -> 390 / 26 = 15]
		delay_us(63200U); 		//The same waveform is repeated a second time after about 63,2 msec.
	}
}

void canon_click() {
	//send the shutter release signal to the camera [---CANON CAMERAS---]
	//This pattern is untested because I didn't have a Canon camera to use, but according to online
	//documentation this *should* work for a Canon camera that has IR remote capabilities.
	uint8_t i, j = 0U;
	// Fire Pattern Twice with IR LED
	for (i = 0U; i < 2U; i++) {
		for (j = 0U; j < 16U; j++) pulse_32k();	//16 pulses @ 32KHz
		delay_us(7210U);     					//pause 7.21 ms [7.21ms * 1000 = 7210us]
		for (j = 0U; j < 16U; j++) pulse_32k();	//16 pulses @ 32KHz
	}
}

void click() {
	printf_P(PSTR("CLICK()\r\n"));

	PORTC |= (1 << IR_INDICATOR_LED_PIN); // turn on indicator LED

	if (camera_type == NIKON)
		nikon_click();
	else if (camera_type == CANON)
		canon_click();

	PORTC &= ~(1 << IR_INDICATOR_LED_PIN); //turn off indicator LED
}

void blink_timer_led_times(uint16_t times) {
	uint16_t x = 0U;
	for (x = 0U; x < times; x++) {
		PORTC |= (1 << TIMER_INDICATOR_LED_PIN);
		delay_ms(200U);
		PORTC &= ~(1 << TIMER_INDICATOR_LED_PIN);
		delay_ms(200U);
	}
}

int main() {

	// LEDs as outputs
	DDRC |= (1 << IR_LED_PIN);
	DDRC |= (1 << IR_INDICATOR_LED_PIN);
	DDRC |= (1 << TIMER_INDICATOR_LED_PIN);

	//enable internal pullup resistors
	PORTC |= (1 << MODE_SWITCH_PIN);
	PORTC |= (1 << POTENTIOMETER_PIN);
	PORTD |= (1 << TRIGGER_PIN);
	
	// Pin change interrupt control register - enables interrupt vectors
	// Bit 2 = enable PC vector 2 (PCINT23..16)
	// Bit 1 = enable PC vector 1 (PCINT14..8)
	// Bit 0 = enable PC vector 0 (PCINT7..0)
	PCICR |= (1 << PCIE2);

	// Pin change mask registers decide which pins are enabled as triggers
	PCMSK2 |= (1 << PCINT18);

	adc_init();

	//start up the serial port
	uart_init();
	FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	stdin = stdout = &uart_stream;

	printf_P(PSTR("STARTING\r\n"));
	
	//Check mode switch
	if ((PINC & (1 << MODE_SWITCH_PIN)) == 0) {
		mode = REMOTE;
		blink_timer_led_times(1U);
	} else {
		mode = INTERVALOMETER;
		blink_timer_led_times(2U);
		SetupTimer2();
		timer_interval_ms = get_interval_ms();
	}
	delay_ms(500U);
	
	printf_P(PSTR("Mode %u ms\r\n"), mode);
	
	while (1) {
		if (mode == INTERVALOMETER) {
		
			set_sleep_mode(SLEEP_MODE_PWR_SAVE);		
			cli();
			sleep_enable();
			sei();
			sleep_cpu();
			sleep_disable();
			sei();

			if (the_time_ms >= timer_interval_ms) {
				the_time_ms = 0UL;
				remainder_counter = 0UL;
				click();
				count_down_ms = 5000UL;
				printf_P(PSTR("Waiting for: %lu ms\r\n"), timer_interval_ms);
			} else {		
				//while waiting for next intervalometer click, flash a green LED to show it is still on
				if (the_time_ms > 0UL && (count_down_ms <= 0L)) {		//turn on every 5 seconds
					count_down_ms = 5000UL;
					blink_timer_led_times(1U);
				}
			}
		}
		else {
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			cli();
			sleep_enable();
			sei();
			sleep_cpu();
			sleep_disable();
			sei();
		}
	}
	
	return 0;
}
