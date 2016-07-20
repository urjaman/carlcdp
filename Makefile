# AVR-GCC Makefile
PROJECT=carlcdp
SOURCES=main.c uart.c console.c lib.c appdb.c commands.c hd44780.c lcd.c timer.c backlight.c buttons.c adc.c relay.c tui.c saver.c tui-other.c dallas.c tui-modules.c tui-calc.c tui-temp.c batlvl.c time.c i2c.c rtc.c tui-alarm.c poweroff.c i2c-uart.c cron.c
DEPS=Makefile buttons.h i2c-uart.h main.h cron.h uart.h
CC=avr-gcc
OBJCOPY=avr-objcopy
MMCU=atmega328p
#AVRBINDIR=~/avr-tools/bin/
AVRDUDEMCU=m328
AVRDUDECMD=sudo avrdude -p $(AVRDUDEMCU) -c avrispmkII -P usb
REMOTEHOST=sempron
AVRDUDECMD_REMOTE=/usr/avr/bin/avrdude -p $(AVRDUDEMCU) -c avrispmkII -P usb
DFLAGS=-DALARMCLOCK
CFLAGS=-mmcu=$(MMCU) -Os -g -Wall -W -pipe -mcall-prologues -std=gnu99 -Wno-main $(DFLAGS)

all: $(PROJECT).hex

$(PROJECT).hex: $(PROJECT).out
	$(AVRBINDIR)$(OBJCOPY) -j .text -j .data -O ihex $(PROJECT).out $(PROJECT).hex
 
$(PROJECT).out: $(SOURCES) timer-ll.o adc-ll.o
	$(AVRBINDIR)$(CC) $(CFLAGS) -flto -fwhole-program -flto-partition=none -mrelax -I./ -o $(PROJECT).out  $(SOURCES) timer-ll.o adc-ll.o -lc -lm

timer-ll.o: timer-ll.c timer.c main.h
	$(AVRBINDIR)$(CC) $(CFLAGS) -I./ -c -o timer-ll.o timer-ll.c

adc-ll.o: adc-ll.c adc.c main.h
	$(AVRBINDIR)$(CC) $(CFLAGS) -I./ -c -o adc-ll.o adc-ll.c
	

asm: $(SOURCES)
	$(AVRBINDIR)$(CC) -S $(CFLAGS) -I./ -o $(PROJECT).S $(SOURCES)

objdump: $(PROJECT).out
	$(AVRBINDIR)avr-objdump -xd $(PROJECT).out | less 

program: $(PROJECT).hex
	cp $(PROJECT).hex /tmp && cd /tmp && $(AVRBINDIR)$(AVRDUDECMD) -U flash:w:$(PROJECT).hex

program-remote: $(PROJECT).hex
	scp $(PROJECT).hex $(REMOTEHOST):/tmp && ssh $(REMOTEHOST) "$(AVRDUDECMD_REMOTE) -U flash:w:/tmp/$(PROJECT).hex"

program-p4-remote: $(PROJECT).hex
	#cat password $(PROJECT).hex > remote-data.txt
	cat $(PROJECT).hex | ssh Admin@p4 "/cygdrive/c/WinAVR-20100110/bin/$(AVRDUDECMD_REMOTE) -V -U flash:w:-:i"

size: $(PROJECT).out
	$(AVRBINDIR)avr-size $(PROJECT).out

clean:
	-rm -f *.o
	-rm -f $(PROJECT).out
	-rm -f $(PROJECT).hex
	-rm -f $(PROJECT).S

backup:
	$(AVRBINDIR)$(AVRDUDECMD) -U flash:r:backup.bin:r
