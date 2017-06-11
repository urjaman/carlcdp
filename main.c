#include "main.h"
#include <avr/power.h>
#include "uart.h"
#include "console.h"
#include "appdb.h"
#include "lib.h"
#include "timer.h"
#include "backlight.h"
#include "lcd.h"
#include "buttons.h"
#include "adc.h"
#include "relay.h"
#include "tui.h"
#include "saver.h"
#include "dallas.h"
#include "batlvl.h"
#include "i2c.h"
#include "powermgmt.h"

#ifdef ENABLE_UARTIF
#define RECVBUFLEN 64
const unsigned char prompt[] PROGMEM = "\x0D\x0AM328>";
unsigned char recvbuf[RECVBUFLEN];
unsigned char token_count;
unsigned char* tokenptrs[MAXTOKENS];

static void uartif_run(void) {
	void(*func)(void);
	if (getline_mc(recvbuf,RECVBUFLEN)) {
		tokenize(recvbuf,tokenptrs, &token_count);
		if (token_count) {
			func = find_appdb(tokenptrs[0]);
			func();
		}
		sendstr_P((PGM_P)prompt);
	}
}
#else
static void uartif_run(void) { }
#endif


static void noints(void) {
	EIMSK = 0;
	EIFR = 0x03;
	PCICR = 0;
	PCMSK2 = 0;
	PCMSK1 = 0;
	PCMSK0 = 0;
	sei();
}

void mini_mainloop(void) {
	timer_run();
	adc_run();
	backlight_run();
	tui_alarm_run();
	batlvl_run();
	relay_run();
	dallas_run();
	uartif_run();
}

void main (void) __attribute__ ((noreturn));

void main(void) {
	cli();
	pm_init();
	clock_prescale_set(clock_div_1);
	noints();
	i2c_init(); // It will need to be before uart_init eventually
	uart_init();
	lcd_init();
	backlight_init();
	timer_init(); // must be after backlight init
	buttons_init();
	adc_init();
	batlvl_init();
	relay_init();
	dallas_init();
	tui_init();
	saver_load_settings();
#ifdef ENABLE_UARTIF
	sendstr_P((PGM_P)prompt); // initial prompt
#endif
	for(;;) {
		mini_mainloop();
		tui_run();
	}
}


