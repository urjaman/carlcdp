#include "main.h"
#include "backlight.h"
#include "buttons.h"
#include "timer.h"

static uint8_t btn_lock = 0;

void buttons_init(void) {
	PORTD |= _BV(2) | _BV(3); // enable pull-ups
}

uint8_t buttons_get_v(void) {
	if (btn_lock) return BUTTON_NONE;
	uint8_t rv = (uint8_t)~PIND;
	rv = (uint8_t)rv & (uint8_t)(_BV(2)|_BV(3));
	rv = (uint8_t)rv >> (uint8_t)2;
	return rv;
}

uint8_t buttons_get(void) {
	uint8_t v = buttons_get_v();
	if (!v) return 0;
	backlight_activate();
	timer_delay_ms(100);
	timer_delay_ms(100);
	for(;;) {
		uint8_t sv;
		timer_delay_ms(40);
		sv = buttons_get_v();
		if (sv == v){
			return v;
		}
		v = sv;
	}
}

void buttons_lock(uint8_t lock) {
	btn_lock = lock;
}
