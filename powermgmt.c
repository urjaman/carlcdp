#include "main.h"
#include "timer.h"
#include "backlight.h"
#include "buttons.h"
#include "lcd.h"
#include "relay.h"
#include "tui.h"
#include "i2c.h"
#include "rtc.h"
#include "saver.h"
#include "powermgmt.h"
#include <avr/wdt.h>

/* This is sort of a TUI-related but not just UI module, so decided to call it just poweroff. */
/* What we are doing would be called suspend-to-ram in the PC world, but for UI simplicity I'll call it poweroff... */

EMPTY_INTERRUPT(INT0_vect);
EMPTY_INTERRUPT(INT1_vect);

static void do_poweroff(void) {
	extern uint8_t timer_time_valid;
	uint8_t rlymode = relay_get_mode();
	// We will save settings so that a power failure in the long sleep wont hurt that much...
	saver_save_settings();
	backlight_simple_set(-1);
	DDRD &= ~_BV(5); // CONTRAST-DRIVE OFF
	relay_set(RLY_MODE_OFF);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	while (buttons_get()); // We will wake up immediately if we sleep while user is holding button...
	EIMSK = 3;
	sleep_mode(); // Bye bye cruel world... atleast for a while.
	// and we're alive again (it might have been months or years, think of that :P)
	EIMSK = 0;
	timer_time_valid = 0;
	DDRD |= _BV(5); // CONTRAST-DRIVE ON
	relay_set(rlymode);
	set_sleep_mode(SLEEP_MODE_IDLE);
}

void tui_poweroff(void) {
	uint8_t rtc_fail=0;
	struct mtm rtc_before;
	struct mtm rtc_after;
	if (!tui_are_you_sure()) return;
	lcd_clear();
	lcd_puts_P(PSTR("POWER OFF IN 1s!"));
	for(uint8_t i=0;i<10;i++) timer_delay_ms(100);
	if (rtc_read(&rtc_before)) rtc_fail = 1;
	do_poweroff();
	if (rtc_read(&rtc_after)) rtc_fail = 1;
	if (rtc_fail) {
		tui_gen_message(PSTR("SLEEP DURATION"), PSTR("UNKNOWN :("));
	} else {
		uint32_t passed = mtm2linear(&rtc_after) - mtm2linear(&rtc_before);
		lcd_clear();
		lcd_puts_P(PSTR("SLEEP DURATION:"));
		tui_time_print(passed);
		tui_waitforkey();
	}
}

void pm_init(void) {
	/* This sequence both turns off WDT reset mode and sets the period to 125ms */
	wdt_reset();
	MCUSR &= ~_BV(WDRF);
	WDTCSR = _BV(WDCE) | _BV(WDE);
	WDTCSR = _BV(WDIF) | _BV(WDP1) | _BV(WDP0);
	DDRC |= _BV(2);
	PORTC &= ~_BV(2);
}

extern volatile uint16_t subsectimer;
extern volatile uint8_t timer_run_todo;
ISR(WDT_vect) {
	/* Add 125ms to the system timers... You're not supposed to do this ;p */
	WDTCSR |= _BV(WDIF);
	const uint16_t addcnt = SSTC/8;
	uint24_t ss = subsectimer;
	ss += addcnt;
	if (ss >= SSTC) {
		ss -= SSTC;
		timer_run_todo++;
	}
	subsectimer = ss;
}

#define LPM_TIMEOUT 5
extern volatile uint16_t adc_isr_out_cnt;
extern uint32_t timer_idle_since;

void low_power_mode(void) {
	cli();
	if (adc_isr_out_cnt < 5) goto idle;
	uint32_t diff = timer_get() - timer_idle_since;
	if (diff < LPM_TIMEOUT) goto idle;
	PORTC |= _BV(2);
	SMCR = _BV(SM1) | _BV(SE); /* sleep enable and set mode */
	DDRD &= ~_BV(5); // CONTRAST-DRIVE OFF
	EIFR = 3;
	EIMSK = 3;
	WDTCSR |= _BV(WDIE); /* Enable WDT for timing. */
	sei();
	sleep_cpu();
	EIMSK = 0;
	SMCR = 0; /* sleep disable */
	WDTCSR &= ~_BV(WDIE); /* Stop the WDT timing. */
	DDRD |= _BV(5); // CONTRAST-DRIVE ON
	PORTC &= ~_BV(2);
	return;

idle:
	SMCR = _BV(SE);
	sei();
	sleep_cpu();
	SMCR = 0; /* sleep_disable */
}
