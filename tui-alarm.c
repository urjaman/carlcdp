#include "main.h"
#include "timer.h"
#include "lcd.h"
#include "relay.h"
#include "backlight.h"
#include "buttons.h"
#include "tui.h"
#include "saver.h"

// These are stored by saver (ifdef ALARMCLOCK), thus not static.
uint8_t tui_alarm_enabled = 0;
uint8_t tui_alarm_snooze_time = 8;
uint8_t tui_alarm_autosnooze_time = 30;
uint16_t tui_alarm_set_time = 0;
// This needs to be restored to tui_alarm_set_time, thus not static.
uint16_t tui_alarm_time = 0;

static uint8_t tui_alarm_active = 0;
static uint32_t tui_alarm_active_start;

static uint16_t tui_get_mindaytime(void) {
	struct mtm tm;
	timer_get_time(&tm);
	return (tm.hour*60+tm.min);
}

static void tui_alarm_setstate(uint8_t t) {
	if (t) {
		backlight_simple_set(16);
#ifdef ALARMCLOCK
		if (t==2) {
			relay_set(RLY_MODE_ON);
		} else { // 0.15s Bzzt (10% duty), 10ms period
			for (uint8_t z = 0;z<15;z++) {
				relay_set(RLY_MODE_ON);
				timer_delay_ms(1);
				relay_set(RLY_MODE_OFF);
				timer_delay_ms(9);
			}
		}
#endif
	} else {
#ifdef ALARMCLOCK
		relay_set(RLY_MODE_OFF);
#endif
		backlight_simple_set(0);
	}
}
// Alarm intensity levels for now:
// 0: 1 state  of low beep		<1min (min=90s)
// 1: 2 states of low beep		<2min
// 2: 3 states of low beep		<3min
// 3: 3 states of high beep		rest

static void tui_alarm_active_f(uint8_t init) {
	static uint8_t alarm_laststate = 0;
	static uint8_t alarm_baseline = 0;
	if (init) {
		backlight_lock(1);
		buttons_lock(1);
		alarm_baseline = timer_get_5hz_cnt();
		alarm_laststate = 255;
		tui_alarm_active = 1;
		tui_alarm_active_start = timer_get();
	}
	uint8_t intensity = 0;
	uint16_t intensity_tmp = (timer_get() - tui_alarm_active_start)/90;
	if (intensity_tmp>3) intensity = 3;
	else intensity = intensity_tmp;
	
	uint8_t alarm_state = timer_get_5hz_cnt() - alarm_baseline;
	if (alarm_state == alarm_laststate) return;
	if (alarm_state>=5) {
		alarm_baseline = timer_get_5hz_cnt();
		alarm_state = 0;
	}
	// 3 units of beep, 2 of nonbeep. State 0 1 2 ; 3 4, >=5 => 0.
	switch (alarm_state) {
		case 2: if (intensity<2) { tui_alarm_setstate(0); break; } /* FALL-THROUGH */
		case 1: if (intensity<1) { tui_alarm_setstate(0); break; } /* FALL-THROUGH */
		case 0:
			tui_alarm_setstate((intensity==3)?2:1);
			break;
		case 3:
		case 4:
			tui_alarm_setstate(0);
			break;
	}
	alarm_laststate = alarm_state;
}

static void tui_alarm_deactivate(void) {
	buttons_lock(0);
	backlight_lock(0);
#ifdef ALARMCLOCK
	relay_set(RLY_MODE_OFF);
#endif
	tui_alarm_active = 0;
}

static void tui_alarm_snooze(void) {
	if (tui_alarm_active) {
		tui_alarm_deactivate();
		tui_alarm_enabled = 1;
		tui_alarm_time = tui_get_mindaytime() + tui_alarm_snooze_time;
	}
}

void tui_alarm_run(void) {
	if (tui_alarm_active) {
		if (tui_get_mindaytime() >= (tui_alarm_time + tui_alarm_autosnooze_time)) {
			tui_alarm_snooze();
			return;
		}
		buttons_lock(0);
		if (buttons_get_v()) {
			tui_alarm_snooze();
		} else {
			buttons_lock(1);
			tui_alarm_active_f(0);
		}
	} else {
		if (!timer_get_1hzp()) return;
		if (tui_alarm_enabled) {
			if (tui_get_mindaytime() == tui_alarm_time) {
				tui_alarm_active_f(1);
			}
		}
	}
}

// *** Menu subsysten start **** /


const unsigned char tui_am_name[] PROGMEM = "ALARM MENU";
const unsigned char tui_am_s1_off[] PROGMEM = "SET ALARM";
const unsigned char tui_am_s1_on[] PROGMEM = "DISABLE ALARM";
const unsigned char tui_am_s2[] PROGMEM = "SET SNOOZE";
const unsigned char tui_am_s3[] PROGMEM = "SET AUTOSNOOZE";

PGM_P const tui_am_off_table[] PROGMEM = {
    (PGM_P)tui_am_s1_off,
    (PGM_P)tui_am_s2,
    (PGM_P)tui_am_s3,
    (PGM_P)tui_exit_menu,
};

PGM_P const tui_am_on_table[] PROGMEM = {
    (PGM_P)tui_am_s1_on,
    (PGM_P)tui_am_s2,
    (PGM_P)tui_am_s3,
    (PGM_P)tui_exit_menu,
};

static uint8_t tui_alarm_time_printer(unsigned char* buf, int32_t val) {
	tui_num_helper(buf,val/60);
	buf[2] = ':';
	tui_num_helper(buf+3,val%60);
	buf[5] = 0;
	return 5;
}

static uint16_t tui_alarm_time_pick(uint16_t start) {
	start = tui_gen_adjmenu(PSTR("SET HOURS"),tui_alarm_time_printer,0,1439,start,60);
	start = tui_gen_adjmenu(PSTR("SET MINUTES"),tui_alarm_time_printer,0,1439,start,1);
	return start;
}

void tui_alarm_menu(void) {
	PGM_P alarm_is = PSTR("ALARM IS");
	uint8_t sel = 0;
	for (;;) {
		if (tui_alarm_enabled) {
			sel = tui_gen_listmenu((PGM_P)tui_am_name, tui_am_on_table, 4, sel);
		} else {
			sel = tui_gen_listmenu((PGM_P)tui_am_name, tui_am_off_table, 4, sel);
		}
		switch (sel) {
			case 0:
				if (tui_alarm_enabled) {
					if (!tui_are_you_sure()) break;
					tui_alarm_enabled = 0;
					tui_gen_message(alarm_is,PSTR("DISABLED"));
				} else {
					tui_alarm_set_time = tui_alarm_time_pick(tui_alarm_set_time);
					tui_alarm_time = tui_alarm_set_time;
					tui_alarm_enabled = 1;
					tui_gen_message(alarm_is,PSTR("ENABLED"));
				}
				saver_save_settings(); // People will assume an alarm clock to remember stuff
				return;
			case 1:
				tui_alarm_snooze_time = tui_gen_adjmenu((PGM_P)tui_am_s2,tui_alarm_time_printer,2,30,tui_alarm_snooze_time,1);
				saver_save_settings(); // People will assume an alarm clock to remember stuff
				break;
			case 2:
				tui_alarm_autosnooze_time = tui_gen_adjmenu((PGM_P)tui_am_s3,tui_alarm_time_printer,2,60,tui_alarm_autosnooze_time,1);
				saver_save_settings(); // People will assume an alarm clock to remember stuff
				break;
			default:
				return;
		}
	}
}

uint8_t tui_alarm_mod_str(uint8_t *buf) {
	buf[0] = 'A';
	buf[1] = ':';
	if (tui_alarm_enabled) {
		return tui_alarm_time_printer(buf+2,tui_alarm_time)+2;
	} else {
		buf[2] = 'O';
		buf[3] = 'F';
		buf[4] = 'F';
		return 5;
	}
}
