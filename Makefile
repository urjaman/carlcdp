# AVR-GCC Makefile
PROJECT=carlcdp
SOURCES=main.c uart.c console.c lib.c appdb.c commands.c hd44780.c lcd.c timer.c backlight.c buttons.c adc.c relay.c tui.c saver.c tui-other.c dallas.c tui-modules.c tui-calc.c tui-temp.c batlvl.c time.c i2c.c rtc.c tui-alarm.c powermgmt.c i2c-uart.c cron.c
DEPS=Makefile buttons.h i2c-uart.h main.h cron.h uart.h
CC=avr-gcc
OBJCOPY=avr-objcopy
MMCU=atmega328p
#AVRBINDIR=~/avr-tools/bin/
AVRDUDEMCU=m328
AVRDUDECMD=avrdude -p $(AVRDUDEMCU) -c atmelice_isp -B 5
#DFLAGS=-DALARMCLOCK
CFLAGS=-mmcu=$(MMCU) -Os -g -Wall -W -pipe -mcall-prologues -std=gnu99 -Wno-main $(DFLAGS)

all: $(PROJECT).hex size

$(PROJECT).hex: $(PROJECT).out
	$(AVRBINDIR)$(OBJCOPY) -j .text -j .data -O ihex $(PROJECT).out $(PROJECT).hex

$(PROJECT).out: $(SOURCES) timer-ll.o adc-ll.o
	$(AVRBINDIR)$(CC) $(CFLAGS) -flto -fwhole-program -flto-partition=none  -I./ -o $(PROJECT).out  $(SOURCES) timer-ll.o adc-ll.o -lc -lm

timer-ll.o: timer-ll.c timer.c main.h
	$(AVRBINDIR)$(CC) $(CFLAGS) -I./ -c -o timer-ll.o timer-ll.c

adc-ll.o: adc-ll.c adc.c main.h
	$(AVRBINDIR)$(CC) $(CFLAGS) -I./ -c -o adc-ll.o adc-ll.c


asm: $(SOURCES)
	$(AVRBINDIR)$(CC) -S $(CFLAGS) -I./ -o $(PROJECT).S $(SOURCES)

objdump: $(PROJECT).out
	$(AVRBINDIR)avr-objdump -xd $(PROJECT).out | less

program: $(PROJECT).hex
	$(AVRBINDIR)$(AVRDUDECMD) -U flash:w:$(PROJECT).hex

size: $(PROJECT).out
	$(AVRBINDIR)avr-size $(PROJECT).out

clean:
	-rm -f *.o
	-rm -f $(PROJECT).out
	-rm -f $(PROJECT).hex
	-rm -f $(PROJECT).S

backup:
	$(AVRBINDIR)$(AVRDUDECMD) -U flash:r:backup.bin:r
