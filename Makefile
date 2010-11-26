GCCFLAGS=-g -Os -Wall -mmcu=atmega168 
LINKFLAGS=-Wl,-u,vfprintf -lprintf_flt -Wl,-u,vfscanf -lscanf_flt -lm
AVRDUDEFLAGS=-c avr109 -p m168 -b 115200 -P /dev/ttyUSB0
LINKOBJECTS=../libnerdkits/delay.o ../libnerdkits/lcd.o ../libnerdkits/uart.o

all:	remote-upload

remote.hex:	remote.c
	make -C ../libnerdkits
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -o remote.o remote.c ${LINKOBJECTS}
	avr-objcopy -j .text -O ihex remote.o remote.hex
	
remote.ass:	remote.hex
	avr-objdump -S -d remote.o > remote.ass
	
remote-upload:	remote.hex
	avrdude ${AVRDUDEFLAGS} -U flash:w:remote.hex:a

clean:
	rm *.o
	rm *.hex
