#include "main.h"
#include "relay.h"
#include "adc.h"
#include "buttons.h"
#include "timer.h"
#include "lcd.h"
#include "lib.h"
#include "backlight.h"
#include "saver.h"
#include "tui.h"
#include "time.h"

#include "dallas.h"


uint8_t tui_mp_mods[4][TUI_MODS_MAXDEPTH];
static uint8_t tui_mp_modidx[4];
static uint8_t tui_force_draw;
static uint8_t tui_next_refresh; 
static uint8_t tui_refresh_interval=TUI_DEFAULT_REFRESH_INTERVAL; // by default 1s

uint8_t tui_pollkey(void) {
	uint8_t v = buttons_get();
	mini_mainloop();
	return v;
}

uint8_t tui_waitforkey(void) {
	uint8_t v;
	for(;;) {
		v = tui_pollkey();
		if (v) return v;
	}
}

void tui_gen_menuheader(unsigned char* line, unsigned char* buf, PGM_P header) {
	uint8_t y,z;
	strcpy_P((char*)buf, header);
	memset(line,'=',16);
	line[16] = 0;
	y = strlen((char*)buf);
	z = (16 - y) / 2;
	memcpy(&(line[z]),buf,y);
	lcd_gotoxy(0,0);
	lcd_puts(line);
}

int32_t tui_gen_menupart(unsigned char* line, unsigned char* buf, printval_func_t *printer, int32_t min, int32_t max, int32_t start, int32_t step, uint8_t delay) {
	int32_t idx=start;
	uint8_t z,y;
	for (;;) {
		uint8_t key;
		y = printer(buf,idx);
		memset(line,' ',16);
		z = (16 - y) / 2;
		memcpy(&(line[z]),buf,y);
		lcd_gotoxy(0,1);
		lcd_puts(line);
		if (delay) timer_delay_ms(delay);
		key = tui_waitforkey();
		switch (key) {
			case BUTTON_S1:
				if ((idx+step) > max) idx = min;
				else idx += step;
				break;
			case BUTTON_S2:
				if ((idx-step) < min) idx = max;
				else idx -= step;
				break;
			case BUTTON_BOTH:
				return idx;
		}
	}
}
static PGM_P const * tui_pgm_menu_table;
static uint8_t tui_pgm_menupart_printer(unsigned char* buf, int32_t val) {
	strcpy_P((char*)buf, (PGM_P)pgm_read_word(&(tui_pgm_menu_table[val])));
	return strlen((char*)buf);
}

uint8_t tui_pgm_menupart(unsigned char* line, unsigned char* buf, PGM_P const menu_table[], uint8_t itemcnt, uint8_t start) {
	tui_pgm_menu_table = menu_table;
	return tui_gen_menupart(line,buf,tui_pgm_menupart_printer,0,itemcnt-1,start,1,50);
}

int32_t tui_gen_adjmenu(PGM_P header, printval_func_t *printer,int32_t min, int32_t max, int32_t start, int32_t step) {
	unsigned char line[17];
	unsigned char buf[17];
	tui_gen_menuheader(line,buf,header);
	return tui_gen_menupart(line,buf,printer,min,max,start,step,0);
}


uint8_t tui_gen_listmenu(PGM_P header, PGM_P const menu_table[], uint8_t itemcnt, uint8_t start) {
	unsigned char line[17];
	unsigned char buf[17];
	tui_gen_menuheader(line,buf,header);
	return tui_pgm_menupart(line,buf,menu_table,itemcnt,start);
}


static uint8_t tui_voltmenu_printer(unsigned char* buf, int32_t val) {
	adc_print_dV(buf,(uint16_t)val);
	return strlen((char*)buf);
}

static uint16_t tui_gen_voltmenu(PGM_P header, uint16_t start) {
	return adc_from_dV( 
		tui_gen_adjmenu(header,tui_voltmenu_printer,0,1600,adc_to_dV(start),1)
		);
}

static uint8_t tui_nummenu_printer(unsigned char* buf, int32_t val) {
	return uint2str(buf,(uint16_t)val);
}

uint16_t tui_gen_nummenu(PGM_P header, uint16_t min, uint16_t max, uint16_t start) {
	return (uint16_t)tui_gen_adjmenu(header,tui_nummenu_printer,min,max,start,1);
}


static void tui_gen_message_start(PGM_P l1) {
	lcd_clear();
	lcd_gotoxy((16 - strlen_P(l1))/2,0);
	lcd_puts_P(l1);
}

static void tui_gen_message_end(void) {
	timer_delay_ms(100);
	tui_waitforkey();
}

void tui_gen_message(PGM_P l1, PGM_P l2) {
	tui_gen_message_start(l1);
	lcd_gotoxy((16 - strlen_P(l2))/2,1);
	lcd_puts_P(l2);
	tui_gen_message_end();
}

void tui_gen_message_m(PGM_P l1, const unsigned char* l2m) {
	tui_gen_message_start(l1);
	lcd_gotoxy((16 - strlen((const char*)l2m))/2,1);
	lcd_puts(l2m);
	tui_gen_message_end();
}

const unsigned char tui_q_s1[] PROGMEM = "NO";
const unsigned char tui_q_s2[] PROGMEM = "YES";

PGM_P const tui_q_table[] PROGMEM = {
    (PGM_P)tui_q_s1,
    (PGM_P)tui_q_s2,
};

uint8_t tui_are_you_sure(void) {
	return tui_gen_listmenu(PSTR("ARE YOU SURE?"), tui_q_table, 2, 0);
}


//Generic Exit Menu Item
const unsigned char tui_exit_menu[] PROGMEM = "EXIT MENU";


const unsigned char tui_rm_s1[] PROGMEM = "OFF";
const unsigned char tui_rm_s2[] PROGMEM = "ON";
const unsigned char tui_rm_s3[] PROGMEM = "AUTO";
PGM_P const tui_rm_table[] PROGMEM = {
    (PGM_P)tui_rm_s1,
    (PGM_P)tui_rm_s2,
    (PGM_P)tui_rm_s3,
};

static void tui_relaymenu(void) {
	uint8_t sel;
	sel = tui_gen_listmenu(PSTR("RELAY MODE:"), tui_rm_table, 3, relay_get_mode());
	relay_set(sel);
}


const unsigned char tui_blsm_name[] PROGMEM = "DISPLAY CFG";
const unsigned char tui_blsm_s1[] PROGMEM = "BL BRIGHTNESS";
const unsigned char tui_blsm_s2[] PROGMEM = "BL DRV BRIGHT";
const unsigned char tui_blsm_s3[] PROGMEM = "BL TIMEOUT";
const unsigned char tui_blsm_s4[] PROGMEM = "LCD CONTRAST";
PGM_P const tui_blsm_table[] PROGMEM = { // BL Settings Menu
	(PGM_P)tui_blsm_s1,
	(PGM_P)tui_blsm_s2,
	(PGM_P)tui_blsm_s3,
	(PGM_P)tui_blsm_s4,
	(PGM_P)tui_exit_menu
};


static uint8_t tui_contrast_set_util_printer(unsigned char* buf, int32_t val) {
	backlight_set_contrast((uint8_t)val);
	return uchar2str(buf,(uint8_t)val);
}

static void tui_contrast_set_util(void) {
	tui_gen_adjmenu((PGM_P)tui_blsm_s4,tui_contrast_set_util_printer,CONTRAST_MIN,CONTRAST_MAX,backlight_get_contrast(),1);
}

static void tui_blsettingmenu(void) {
	uint8_t sel = 0;
	for(;;) {
		sel = tui_gen_listmenu((PGM_P)tui_blsm_name, tui_blsm_table, 5, sel);
		switch (sel) {
			case 0: {
			uint8_t v = tui_gen_nummenu((PGM_P)tui_blsm_s1, 0, 16, backlight_get());
			backlight_set(v);
			}
			break;

			case 1: {
			uint8_t v = tui_gen_nummenu((PGM_P)tui_blsm_s2, 0, backlight_get(), backlight_get_dv());
			backlight_set_dv(v);
			}
			break;

			case 2: {
			uint8_t v = tui_gen_nummenu((PGM_P)tui_blsm_s3, 1, 255, backlight_get_to());
			backlight_set_to(v);
			}
			break;

			case 3:
			tui_contrast_set_util();
			break;

			default:
			return;
		}
	}
}



const unsigned char tui_setclock_name[] PROGMEM = "SET CLOCK";
void tui_set_clock(void) {
	struct mtm tm;
	timer_get_time(&tm);
	tm.sec = 0;
	uint16_t year = tui_gen_nummenu(PSTR("YEAR"),TIME_EPOCH_YEAR,TIME_EPOCH_YEAR+130,tm.year+TIME_EPOCH_YEAR);
	uint8_t month = tui_gen_nummenu(PSTR("MONTH"),1,12,tm.month);
	year = year - TIME_EPOCH_YEAR;
	if ((tm.month != month)||(tm.year != year)) { // Day count in the month possibly changed, cannot start from old day.
		tm.day = 1;
	}
	tm.day = tui_gen_nummenu(PSTR("DAY"),1,month_days(year,month-1),tm.day);
	tm.year = year;
	tm.month = month;
	tm.hour = tui_gen_nummenu(PSTR("HOURS"), 0, 23, tm.hour);
	tm.min = tui_gen_nummenu(PSTR("MINUTES"),0, 59, tm.min);
	timer_set_time(&tm);
}


const unsigned char tui_sm_name[] PROGMEM = "SETTINGS";
const unsigned char tui_sm_s1[] PROGMEM = "RLY VOLTTHRES";
const unsigned char tui_sm_s2[] PROGMEM = "RLY AUTO KEEPON";
// BL Settings menu (3)
const unsigned char tui_sm_s4[] PROGMEM = "TUI CONFIG";
// Set Clock (5)
const unsigned char tui_sm_s6[] PROGMEM = "SAVE SETTINGS";
const unsigned char tui_sm_s7[] PROGMEM = "LOAD SETTINGS";
#ifdef ALARMCLOCK
const unsigned char tui_sm_s8[] PROGMEM = "SET RELAY MODE";
#else
const unsigned char tui_sm_s8[] PROGMEM = "ALARM MENU";
#endif
const unsigned char tui_sm_s9[] PROGMEM = "ADC CALIBRATION";

// Exit Menu (9)


PGM_P const tui_sm_table[] PROGMEM = { // Settings Menu
    (PGM_P)tui_sm_s1,
    (PGM_P)tui_sm_s2,
    (PGM_P)tui_blsm_name,
    (PGM_P)tui_sm_s4,
    (PGM_P)tui_setclock_name,
    (PGM_P)tui_sm_s6,
    (PGM_P)tui_sm_s7,
    (PGM_P)tui_sm_s8,
    (PGM_P)tui_sm_s9,
    (PGM_P)tui_exit_menu
};

static void tui_settingsmenu(void) {
	uint8_t sel = 0;
	for(;;) {
		sel = tui_gen_listmenu((PGM_P)tui_sm_name, tui_sm_table, 10, sel);
		switch (sel) {
			case 0: {
			uint16_t v = tui_gen_voltmenu((PGM_P)tui_sm_s1, relay_get_autovoltage());
			relay_set_autovoltage(v);
			}
			break;

			case 1: {
			uint8_t v = tui_gen_nummenu((PGM_P)tui_sm_s2, 1, 255, relay_get_keepon());
			relay_set_keepon(v);
			}
			break;

			case 2:
			tui_blsettingmenu();
			break;

			case 3:
			tui_config_menu();
			break;

			case 4:
			tui_set_clock();
			break;

			case 5:
			saver_save_settings();
			tui_gen_message((PGM_P)tui_sm_name,PSTR("SAVED"));
			return;

			case 6:
			saver_load_settings();
			tui_gen_message((PGM_P)tui_sm_name,PSTR("LOADED"));
			return;
			
			case 7:
#ifndef ALARMCLOCK
			tui_alarm_menu();
#else
			tui_relaymenu();
#endif
			break;

			case 8:
			tui_adc_calibrate();
			break;
			
			default:
			return;
		}
	}
}

#ifdef ALARMCLOCK
const unsigned char tui_mm_s1[] PROGMEM = "ALARM MENU";
#else
const unsigned char tui_mm_s1[] PROGMEM = "SET RELAY MODE";
#endif
// Settings Menu (2)
const unsigned char tui_mm_s3[] PROGMEM = "OTHERS";
// Exit Menu (4)

PGM_P const tui_mm_table[] PROGMEM = {
    (PGM_P)tui_mm_s1,
    (PGM_P)tui_sm_name,
    (PGM_P)tui_mm_s3,
    (PGM_P)tui_exit_menu
};

static void tui_mainmenu(void) {
	uint8_t sel=0;
	for (;;) {
		sel = tui_gen_listmenu(PSTR("MAIN MENU"), tui_mm_table, 4, sel);
		switch (sel) {
			case 0:
#ifdef ALARMCLOCK
				tui_alarm_menu();
#else
				tui_relaymenu();
#endif
				break;
			case 1:
				tui_settingsmenu();
				break;
			case 2:
				tui_othermenu();
				break;
			default:
				return;
		}
	}
}

static void tui_draw_mainpage(uint8_t forced) {
	uint8_t d;
	unsigned char line[17];
	unsigned char buf[16];
	line[16]=0;
	if (!forced) {
		static uint8_t drawcnt=0;
		drawcnt++;
		if ((drawcnt&1)==0) {
			uint8_t i;
			for (i=0;i<4;i++) {
				uint8_t c = tui_mp_modidx[i] + 1;
				if (c >= TUI_MODS_MAXDEPTH) c=0;
				if (tui_mp_mods[i][c] == 255) c=0;
				tui_mp_modidx[i] = c;
			}
		}
	}
	// Line 0
	memset(line,' ',16);
	d = tui_run_mod(tui_mp_mods[0][tui_mp_modidx[0]],buf,10);
	memcpy(line,buf,d);
	d = tui_run_mod(tui_mp_mods[1][tui_mp_modidx[1]],buf,15-d);
	memcpy(&(line[16-d]),buf,d);
	lcd_gotoxy(0,0);
	lcd_puts(line);
	// Line 1
	memset(line,' ',16);
	d = tui_run_mod(tui_mp_mods[2][tui_mp_modidx[2]],buf,10);
	memcpy(line,buf,d);
	d = tui_run_mod(tui_mp_mods[3][tui_mp_modidx[3]],buf,15-d);
	memcpy(&(line[16-d]),buf,d);
	lcd_gotoxy(0,1);
	lcd_puts(line);
	tui_force_draw = 0;
	if (!forced) {
		tui_refresh_interval = tui_update_refresh_interval();
		tui_next_refresh = timer_get_5hz_cnt()+tui_refresh_interval;
	}
}

void tui_init(void) {
	uint8_t i;
	for(i=0;i<4;i++) {
		uint8_t n;
		for(n=1;n<TUI_MODS_MAXDEPTH;n++) {
			tui_mp_mods[i][n] = 255;
		}
	}
	tui_mp_mods[0][0] = 0; // Main Bat
#ifdef ALARMCLOCK
	tui_mp_mods[1][0] = 3; // Clock
	tui_mp_mods[2][0] = 10; // Date
	tui_mp_mods[3][0] = 2; // Relay State
#else
	tui_mp_mods[1][0] = 6; // Temp 0
	tui_mp_mods[2][0] = 1; // Sec Bat
	tui_mp_mods[3][0] = 2; // Relay State
#endif
	tui_draw_mainpage(0);
}


void tui_run(void) {
	uint8_t k = buttons_get();
	if (k & BUTTON_S2) {
		tui_mainmenu();
		tui_draw_mainpage(0);
		return;
	} else {
		if (tui_force_draw) {
			tui_draw_mainpage(tui_force_draw);
			return;
		} else {
			if (timer_get_5hzp()) {
				signed char update = (signed char)(timer_get_5hz_cnt() - tui_next_refresh);
				if (update>=0) {
					tui_draw_mainpage(0);
					return;
				}
			}
		}
	}
}


void tui_activate(void) {
	tui_force_draw=1;
}
