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
#include "../libnerdkits/lcd.h"
#include "../libnerdkits/uart.h"

// PIN DEFINITIONS:
// PC5 -- trigger button (pulled to ground when pressed)
// PC4 -- IR LED anode
// PC3 -- Indicator LED anode
// PC2 -- Mode switch
// PC0 -- Potentiometer for selecting interval

// Declare variables
uint8_t mode=0;
uint8_t x_avg, i, j, k; 
uint16_t last_sample = 0;
long int interval, timer_interval;
double x;

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
  while(ADCSRA & (1<<ADSC)) {
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

void led_cycle(){
    //this function pulses the LED at 32KHz
    PORTC |= (1<<PC4);
    delay_us(16);
    PORTC &= ~(1<<PC4);
    delay_us(16);
}

int get_interval(){
// take 100 samples and average them!
for(i=0; i<100; i++) {
  x = adc_read();
  // add this contribution to the average
  x_avg = x_avg + x/100;
}
//Decide on the interval time.  The Nerdkits crystal makes a clock that is a tiny bit
//slow (1 usec = 1.017 usec), so I adjusted the interval times accordingly. 
//These times can easily be changed to suit your needs. 
if( x <= 128 )
  interval = 29499; // 30 sec
else if( x <= 256 )
  interval = 58997; // 1 min
else if( x <= 384 )
  interval = 117994; // 2 min
else if( x <= 512 )
  interval = 176991; // 3 min
else if( x <= 640 )
  interval = 235998; // 4 min
else if( x <= 768 )
  interval = 294985; // 5 min
else if( x <= 896 )
  interval = 589970; // 10 min 
else
  interval = 884955; // 15 min
return interval;
}

void click(){
//send the shutter release signal to the camera [---CANON CAMERAS---]
//16 pulses at 32 KHz with delay in between of 7.21 ms
//This pattern is untested because I didn't have a Canon camera to use, but
//according to online documentation this *should* work for a Canon camera 
//that has IR remote capabilities.
	PORTC |= (1<<PC3); // turn on indicator LED
  	// Fire Pattern Twice with IR LED
  	for ( j = 0; j < 2; j++ )
  	{
		for (k = 0; k < 16; k++)
		led_cycle();
		delay_ms(7.21); // wait
		for (k = 0; k < 16; k++)
		led_cycle();
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
  
  // start up the LCD
  lcd_init();
  FILE lcd_stream = FDEV_SETUP_STREAM(lcd_putchar, 0, _FDEV_SETUP_WRITE);
  lcd_home();
  //Intialize the Analog to Digital Convertor
  adc_init();
  // start up the serial port
  uart_init();
  FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
  stdin = stdout = &uart_stream;

  // loop forever!
  while(1) {
    //Wait for button press
    if((PINC & (1<<PC5)) == 0){
      //Check mode switch
      if((PINC & (1<<PC2)) == 0){
        mode = 1;
      }
      if(mode){ 
        //Remote Mode
        click();
	//Write to LCD
	lcd_write_string(PSTR("CLICK"));
      } else  {
        //Intervalometer Mode
        mode = 0;
        //Check Interval
        timer_interval = get_interval();
        //Loop forever...again!
        while(1) {
	  //snap a picture
	  click();
          // write message to LCD
          lcd_home();
          lcd_write_string(PSTR("Interval Mode: "));
          lcd_line_two();
	  lcd_write_int16(x);
          lcd_write_string(PSTR(" of 1024   "));
          lcd_line_three();
          fprintf_P(&lcd_stream, PSTR("Delay: "));
	  lcd_write_int16(interval);
          lcd_write_string(PSTR(" ms"));
	  //delay set number of ms
  	  delay_ms(timer_interval);
	}
      }
    }  
  } 
  return 0;
}
