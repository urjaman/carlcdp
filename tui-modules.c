#include "main.h"
#include "tui.h"
#include "adc.h"
#include "relay.h"
#include "timer.h"
#include "dallas.h"
#include "batlvl.h"

#define TUI_MOD_CNT 10
static uint8_t tui_mbv_mod(uint8_t* buf, uint8_t max);
static uint8_t tui_sbv_mod(uint8_t* buf, uint8_t max);
static uint8_t tui_rlst_mod(uint8_t* buf, uint8_t max);
static uint8_t tui_clk24_mod(uint8_t* buf, uint8_t max);
static uint8_t tui_dif_mod(uint8_t* buf, uint8_t ml);
static uint8_t tui_amp_mod(uint8_t* buf, uint8_t ml);
static uint8_t tui_temp_mod(uint8_t* buf, uint8_t ml, uint8_t idx);
static uint8_t tui_soc_mod(uint8_t* buf, uint8_t ml, uint8_t c, uint8_t soc);


static const unsigned char nullstr[] PROGMEM = "-NONE-";
static const unsigned char mbistr[] PROGMEM = "MAIN BAT VOLTS";
static const unsigned char sbistr[] PROGMEM = "SEC BAT VOLTS";
static const unsigned char rlststr[] PROGMEM = "RELAY STATE";
static const unsigned char clk24str[] PROGMEM = "CLOCK/24H";
static const unsigned char diffstr[] PROGMEM = "BAT VOLT DIFF";
static const unsigned char ampstr[] PROGMEM = "AMPS OVER RELAY";
static const unsigned char temp0str[] PROGMEM = "TEMP SENSOR 0";
static const unsigned char temp1str[] PROGMEM = "TEMP SENSOR 1";
static const unsigned char mbsocstr[] PROGMEM = "MAIN BAT SoC";
static const unsigned char sbsocstr[] PROGMEM = "SEC BAT SoC";

PGM_P const tui_mods_table[] PROGMEM = {
    (PGM_P)nullstr,
    (PGM_P)mbistr,
    (PGM_P)sbistr,
    (PGM_P)rlststr,
    (PGM_P)clk24str,
    (PGM_P)diffstr,
    (PGM_P)ampstr,
    (PGM_P)temp0str,
    (PGM_P)temp1str,
    (PGM_P)mbsocstr,
    (PGM_P)sbsocstr,
};

uint8_t tui_select_mod(uint8_t sel) {
	sel++;
	if (sel>TUI_MOD_CNT) sel=0;
	sel = tui_gen_listmenu(PSTR("PICK MODULE"), tui_mods_table, TUI_MOD_CNT+1, sel);
	return sel-1;
}

uint8_t tui_run_mod(uint8_t mod, uint8_t *p, uint8_t ml) {
	switch (mod) {
		default: return 0;
		case 0: return tui_mbv_mod(p,ml);
		case 1: return tui_sbv_mod(p,ml);
		case 2: return tui_rlst_mod(p,ml);
		case 3: return tui_clk24_mod(p,ml);
		case 4: return tui_dif_mod(p,ml);
		case 5: return tui_amp_mod(p,ml);
		case 6: return tui_temp_mod(p,ml,0);
		case 7: return tui_temp_mod(p,ml,1);
		case 8: return tui_soc_mod(p,ml,'M',batlvl_get_mb());
		case 9: return tui_soc_mod(p,ml,'S',batlvl_get_sb());
	}
}

static const unsigned char tui_modsetm_s1[] PROGMEM = "SET UL MODULE \x01";
static const unsigned char tui_modsetm_s2[] PROGMEM = "SET UR MODULE \x02";
static const unsigned char tui_modsetm_s3[] PROGMEM = "SET DL MODULE \x03";
static const unsigned char tui_modsetm_s4[] PROGMEM = "SET DR MODULE \x04";
const unsigned char tui_update_rate_cfg[] PROGMEM = "UPDATE RATE CFG"; // external to tui-temp.c

PGM_P const tui_modsetm_table[] PROGMEM = { // TUI config menu 
    (PGM_P)tui_modsetm_s1,
    (PGM_P)tui_modsetm_s2,
    (PGM_P)tui_modsetm_s3,
    (PGM_P)tui_modsetm_s4,
    (PGM_P)tui_update_rate_cfg,
    (PGM_P)tui_exit_menu
};


void tui_config_menu(void) {
	uint8_t sel=0, d=0;
	for(;;) {
		sel = tui_gen_listmenu(PSTR("TUI CONFIG"), tui_modsetm_table, 6, sel);
		switch (sel) {
			case 0:
			case 1:
			case 2:
			case 3:
				d = tui_gen_nummenu(PSTR("PICK DEPTH"),0,TUI_MODS_MAXDEPTH-1,d);
				tui_mp_mods[sel][d] = tui_select_mod(tui_mp_mods[sel][d]);
				break;
			case 4:
				tui_refresh_interval_menu();
				break;
			
			default:
				return;
		}
	}
}

static uint8_t tui_modfinish(uint8_t*b, uint8_t*t, uint8_t ml, uint8_t l) {
	if (ml>l) ml=l;
	memcpy(b,t,ml);
	return ml;
}	

static uint8_t tui_mbv_mod(uint8_t* buf, uint8_t ml) {
	uint8_t mb[9];
	mb[0] = 'M';
	mb[1] = ':';
	adc_print_v(&(mb[2]),adc_read_mb());
	return tui_modfinish(buf,mb,ml,8);
}

static uint8_t tui_sbv_mod(uint8_t* buf, uint8_t ml) {
	uint8_t mb[9];
	mb[0] = 'S';
	mb[1] = ':';
	adc_print_v(&(mb[2]),adc_read_sb());
	return tui_modfinish(buf,mb,ml,8);
}

static uint8_t tui_soc_mod(uint8_t* buf, uint8_t ml, uint8_t c, uint8_t soc) {
	uint8_t i;
	uint8_t mb[6];
	mb[0] = c;
	mb[1] = ':';
	mb[2] = soc/100;
	soc = soc%100;
	mb[3] = soc/10;
	mb[4] = soc%10;
	mb[5] = '%';
	for(i=2;i<5;i++) mb[i] |= 0x30;
	if (mb[2] == 0x30) { mb[2] = ' ';
		if (mb[3] == 0x30) mb[3] = ' ';
	}
	return tui_modfinish(buf,mb,ml,6);
}

static uint8_t tui_rlst_mod(uint8_t* buf, uint8_t ml) {
	uint8_t mb[5];
	mb[0] = 'R';
	mb[1] = ':';
	switch (relay_get_mode()) {
		default:
		case RLY_MODE_OFF:
			mb[2] = '0';
			break;
		case RLY_MODE_ON:
			mb[2] = '1';
			break;
		case RLY_MODE_AUTO:
			mb[2] = 'A';
			break;
	}
	mb[3] = 0x1A; // "=>"-like char
	switch (relay_get()) {
		default:
		case RLY_MODE_OFF:
			mb[4] = '0';
			break;
		case RLY_MODE_ON:
			mb[4] = '1';
			break;
	}
	return tui_modfinish(buf,mb,ml,5);
}


static void tui_drawtime(unsigned char* line, uint8_t h, uint8_t m) {
	line[0] = h/10|0x30;
	line[1] = h%10|0x30;
	line[2] = ':';
	line[3] = m/10|0x30;
	line[4] = m%10|0x30;
}

static uint8_t tui_clk24_mod(uint8_t* buf, uint8_t ml) {
	struct mtm tm;
	uint8_t mb[5];
	timer_get_time(&tm);
	tui_drawtime(mb,tm.hour,tm.min);
	return tui_modfinish(buf,mb,ml,5);
}


static uint8_t tui_dif_mod(uint8_t* buf, uint8_t ml) {
	uint16_t mbv,sbv;
	uint8_t mb[8];
	mbv = adc_read_mb();
	sbv = adc_read_sb();
	if (mbv >= sbv) {
		mb[0] = '+';
		mbv = mbv - sbv;
	} else {
		mb[0] = '-';
		mbv = sbv - mbv;
	}
	adc_print_v(&(mb[1]), mbv);
	if (mb[1] == ' ') {
		mb[1] = mb[0];
		return tui_modfinish(buf,&(mb[1]),ml,6);
	} else {
		return tui_modfinish(buf,mb,ml,7);
	}
}

static uint8_t tui_amp_mod(uint8_t* buf, uint8_t ml) {
	uint16_t amps;
	int16_t mbv;
	uint8_t mb[4];
	if (!relay_get()) {
		mb[0] = '0';
		mb[1] = 'A';
		return tui_modfinish(buf,mb,ml,2);
	}
	mbv = adc_read_diff();
	if (mbv >= 0) {
		mb[0] = '+';
	} else {
		mb[0] = '-';
	}
	mbv = abs(mbv);
	// This is very rough amp measrement assuming that the relay has 0.006 ohm resistance.
	// Measurement granularity is 2.6A and margin of error >5A, so...
	amps = ((mbv*13)+10)/20;
	if (amps>99) { // Shows hearts xD
		mb[1] = 0x9D;
		mb[2] = 0x9D;
		mb[3] = 'A';
		return tui_modfinish(buf,mb,ml,4);
	}
	if (amps>9) {
		mb[1] = (amps/10) | 0x30;
		mb[2] = (amps%10) | 0x30;
		mb[3] = 'A';
		return tui_modfinish(buf,mb,ml,4);
	}
	mb[1] = (amps) | 0x30;
	mb[2] = 'A';
	return tui_modfinish(buf,mb,ml,3);
}

uint8_t tui_temp_printer(unsigned char* mb, int32_t val) {
	int16_t t = val;
	uint16_t tt = abs(t);
	uint8_t acv[3];
	// -XX.XC - negative == -55.0C .. -00.5C ; when -9.5 .. -0.5 we cut the extra zero
	// XX.X*C - positive == 00.0*C .. 84.5C ; when 0.0 ... 9.5 we cut the extra zero
	// --.-*C - other values
	acv[0] = (tt/100)|0x30; tt = tt%100;
	acv[1] = (tt/10 )|0x30;
	acv[2] = (tt%10 )|0x30;
	mb[5] = 'C';
	if ((t>=-550)&&(t<0)) {
		t = tt;
		mb[0] = '-';
		mb[1] = acv[0];
		mb[2] = acv[1];
		mb[3] = '.';
		mb[4] = acv[2];
		if (acv[0] == '0') {
			mb[1] = '-';
			for (uint8_t i=0;i<5;i++) mb[i] = mb[i+1];
			return 5;
		} else {
			return 6;
		}
	}
	mb[4] = 0xB0;
	mb[2] = '.';
	if ((t>=0)&&(t < 845)) {
		mb[0] = acv[0];
		mb[1] = acv[1];
		mb[3] = acv[2];
		if (acv[0] == '0') {
			for (uint8_t i=0;i<5;i++) mb[i] = mb[i+1];
			return 5;
		} else {
			return 6;
		}
	}
	mb[0] = mb[1] = mb[3] = '-';
	return 6;
}
	
static uint8_t tui_temp_mod(uint8_t* buf, uint8_t ml, uint8_t idx) {
	int16_t t = dallas_temp_get(idx); 
	uint8_t mb[6];
	uint8_t x = tui_temp_printer(mb,t);
	return tui_modfinish(buf,mb,ml,x);
}

