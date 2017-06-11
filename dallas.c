#include "main.h"
#include "timer.h"
#include "dallas.h"
#include "buttons.h"
#include "console.h"
#include "uart.h"
// Devices=1 is special
#define D1W_MAX_DEVICES 1
#define D1W_MAX_TEMPDEVS 1
// 0 is special, once per sec until something is found, then never.
#define D1W_SCAN_INTERVAL 0

#define D1WPN 0
#define D1WPIN PIND
#define D1WPORT PORTD
#define D1WDDR DDRD

static uint32_t d1w_last_scan_time=0;
static uint8_t d1w_rescan; // Must re-scan when we get the opportunity
static uint8_t d1w_run; // Must run state machine when system free
static uint8_t d1w_tmproffs; // First unread temp sensor idx when state is D1W_TMPR
static uint8_t d1w_tmpidx;
enum d1w_s_t {
	D1W_IDLE=0,
	D1W_TEMPQ, // state in which we will issue convert
	D1W_TEMPR // state in which we wait for convert and read result
} d1w_state=D1W_IDLE;

static uint8_t d1w_devicecnt=0;
static uint8_t d1w_devices[D1W_MAX_DEVICES][8];
static int16_t d1w_temps[D1W_MAX_TEMPDEVS];

static inline void adie_off(void) {
	ADCSRA =  ADCSRA & ~(_BV(ADIE)|_BV(ADIF));
}

static inline void adie_on(void) {
	ADCSRA = (ADCSRA |  _BV(ADIE)) & ~_BV(ADIF);
}

static void d1w_wait(void) {
	uint8_t v=0;
	asm volatile (
	"0: in %0, %1 \n\t"
	"cpi %0, 60\n\t"
	"brlo 0b\n\t"
	"cpi %0, 64\n\t"
	"brsh 0b\n\t"
	: "=r" (v)
	: "I" (_SFR_IO_ADDR(TCNT0))
	);
}

static void d1w_up(void) {
	cli();
	D1WPORT |=  _BV(D1WPN);
	asm volatile ("nop\n\t");
	D1WDDR  &= ~_BV(D1WPN);
	sei();
}

static void d1w_letgo(void) {
	D1WDDR  &= ~_BV(D1WPN);
	D1WPORT |=  _BV(D1WPN);
}

static void d1w_down(void) {
	D1WPORT &= ~_BV(D1WPN);
	D1WDDR  |=  _BV(D1WPN);
}

static uint8_t d1w_sense(void) {
	return (D1WPIN&_BV(D1WPN))?1:0;
}

static void d1w_wait_up(void) {
	d1w_letgo();
	while (!d1w_sense());
	cli();
	D1WDDR  |=  _BV(D1WPN);
	asm volatile ("nop\n\t");
	D1WDDR  &= ~_BV(D1WPN);
	sei();
}

static uint8_t d1w_reset(void) {
	uint8_t rv;
	uint8_t i;
	d1w_wait_up();
	d1w_wait();
	d1w_down();
	for(i=0;i<15;i++) d1w_wait();
	adie_off();
	d1w_up();
	for(i=0;i<3;i++) d1w_wait();
	rv = !d1w_sense();
	adie_on();
	return rv;
}

static void d1w_sendbyte(uint8_t b) {
	uint8_t bc;
	d1w_wait_up();
	for(bc=0;bc<8;bc++) {
		adie_off();
		d1w_wait();
		d1w_down();
		if (b&0x01) {
			_delay_us(8);
			d1w_up();
		}
		b = b>>1;
		d1w_wait();
		d1w_wait();
		d1w_up();
		adie_on();
		_delay_us(1);
	}
}

static uint8_t d1w_recvbyte(void) {
	uint8_t b=0;
	uint8_t bc;
	for(bc=0;bc<8;bc++) {
		adie_off();
		d1w_wait_up(); // safety if slave pulls down longer than expected
		d1w_wait();
		_delay_us(4); // always atleast 4us pause between bits
		d1w_down();
		_delay_us(8);
		d1w_letgo();
		b = b>>1;
		d1w_wait();
		if (d1w_sense()) b |= 0x80;
		d1w_wait();
		adie_on();
		_delay_us(1);
	}
	return b;
}

static uint8_t d1w_rbit(void) {
	uint8_t r;
	d1w_wait_up();
	adie_off();
	d1w_wait();
	d1w_down();
	_delay_us(8);
	d1w_letgo();
	d1w_wait();
	r = d1w_sense();
	d1w_wait();
	d1w_wait();
	adie_on();
	return r;
}


#if D1W_MAX_DEVICES > 1
static void d1w_w0(void) {
	d1w_wait_up();
	d1w_wait(); // Sync to 0
	d1w_down(); // Down at 0us
	d1w_wait();
	d1w_wait();
	d1w_up();
	_delay_us(6);
}

static void d1w_w1(void) {
	d1w_wait_up();
	d1w_wait(); // Sync to 0
	d1w_down(); // Down at 0us
	_delay_us(8);
	d1w_up();
	d1w_wait();
	d1w_wait();
	_delay_us(6);
}

static uint8_t d1w_search_rom(uint8_t * bitPattern, uint8_t lastDeviation) {
    uint8_t currentBit = 1;
    uint8_t newDeviation = 0;
    uint8_t bitMask = 0x01;
    uint8_t bitA;
    uint8_t bitB;

    // Send SEARCH ROM command on the bus.
    d1w_sendbyte(0xF0);
    
    // Walk through all 64 bits.
    while (currentBit <= 64) {
        // Read bit from bus twice.
        bitA = d1w_rbit();
        bitB = d1w_rbit();

        if (bitA && bitB) {
            // Both bits 1 (Error).
            return 0xFF;
        } else if (bitA ^ bitB) {
            // Bits A and B are different. All devices have the same bit here.
            // Set the bit in bitPattern to this value.
            if (bitA) {
                (*bitPattern) |= bitMask;
            } else {
                (*bitPattern) &= ~bitMask;
            }
        } else {
            // If this is where a choice was made the last time,
            // a '1' bit is selected this time.
            if (currentBit == lastDeviation) {
                (*bitPattern) |= bitMask;
            } else if (currentBit > lastDeviation) {
                (*bitPattern) &= ~bitMask;
                newDeviation = currentBit;
            } else if ( !(*bitPattern & bitMask))  {
                newDeviation = currentBit;
            } else {
            // Nothing
            }
        }
                
        
        // Send the selected bit to the bus.
        if ((*bitPattern) & bitMask) {
            d1w_w1();
        } else {
            d1w_w0();
        }
        // Increment current bit.    
        currentBit++;
        // Adjust bitMask and bitPattern pointer.    
        bitMask <<= 1;
        if (!bitMask) {
            bitMask = 0x01;
            bitPattern++;
        }
    }
    return newDeviation;
}
#endif

static uint8_t d1w_crc8(uint8_t inData,uint8_t seed) {
    return _crc_ibutton_update(seed,inData);
}

uint8_t d1w_romcheck(uint8_t* romValue) {
    uint8_t i;
    uint8_t crc8 = 0;
    for (i=0;i<7;i++) {
        crc8 = d1w_crc8(*romValue, crc8);
        romValue++;
    }
    if (crc8 == (*romValue)) return 0;
    dprint("ROM CRC fail");
    return 1;
}

static void d1w_scan(void) {
	uint8_t d=0;
	uint8_t cnt=0;
	dprint("Scanning..");
	memset(d1w_devices,0,D1W_MAX_DEVICES*8);
	for(;;) {
		if (!d1w_reset()) break;
		dprint("Reset OK");
#if D1W_MAX_DEVICES > 1
		d = d1w_search_rom(d1w_devices[cnt],d);
		if (d==0xFF) {
			dprint("ROM search err");
			d1w_rescan = 1;
			break;
		}
		cnt++;
		dprint("cnt++");
		if (d==0) break;
		if (cnt==D1W_MAX_DEVICES) break;
		memcpy(d1w_devices[cnt],d1w_devices[cnt-1],8);
#else
		uint8_t i;
		d1w_sendbyte(0x33); // read rom
		for(i=0;i<8;i++) d1w_devices[0][i] = d1w_recvbyte();
		cnt=1;
		break;
#endif
	}
	dprint("ROM check...");
	for(d=0;d<cnt;d++) {
		if (d1w_romcheck(d1w_devices[d])) {
			cnt = 0;
			d1w_rescan = 1;
		}
	}
	dprint("Done");
	d1w_last_scan_time = timer_get();
	d1w_devicecnt = cnt;
}
static void d1w_matchrom(uint8_t idx) {
#if D1W_MAX_DEVICES > 1
	uint8_t i;
	d1w_reset();
	d1w_sendbyte(0x55);
	for(i=0;i<8;i++) d1w_sendbyte(d1w_devices[idx][i]);
#else
	idx=idx;
	d1w_reset();
	d1w_sendbyte(0xCC);
#endif
}

static void d1w_temp_convert(void) {
	d1w_matchrom(d1w_tmproffs);
	d1w_sendbyte(0x44);
}

static uint8_t d1w_temp_crc(uint8_t* buf) {
    uint8_t i;
    uint8_t crc8 = 0;
    for (i=0;i<8;i++) {
        crc8 = d1w_crc8(*buf, crc8);
        buf++;
    }
    if (crc8 == (*buf))
	    return 0;
    return 1;
}

static void d1w_temp_read(void) {
	int16_t lvt = -999;
	int16_t bvt = -999;
	int16_t temp = -999;
	uint8_t i, retries=40;
	uint8_t buf[9];

retry:
	d1w_matchrom(d1w_tmproffs);
	d1w_sendbyte(0xBE);
	for(i=0;i<9;i++) buf[i] = d1w_recvbyte();
	if (d1w_temp_crc(buf)) {
		int16_t ft = ((buf[1]<<8) | buf[0])*5;
		if ((ft>=-300)&&(ft<=40)&&(ft!=-5)) {
			lvt = ft; // -30 ... +4
			if ((ft >= (d1w_temps[d1w_tmpidx]-10))&&(ft <= (d1w_temps[d1w_tmpidx]+10))) {
				bvt = ft;
			}
		}
		if ((!timer_get_todo())&&(!buttons_get_v())&&(bvt == -999)&&(retries)) {
			retries--;
			goto retry;
		} else {
			if (bvt != -999) temp = bvt;
			else if (lvt != -999) temp = lvt;
			else temp = d1w_temps[d1w_tmpidx];
			goto exit;
		}
	}
	temp = ((buf[1]<<8) | buf[0])*5;
exit:
	d1w_temps[d1w_tmpidx] = temp;
}

void dallas_init(void) {
	uint8_t i;
	for (i=0;i<D1W_MAX_TEMPDEVS;i++) d1w_temps[i] = -999;
	D1WDDR  &= ~_BV(D1WPN);
	D1WPORT |=  _BV(D1WPN);
	d1w_rescan=1;
}

void dallas_run(void) {
	if (timer_get_idle()) return; /* No point in polling the temperatures all the time... */
	d1w_run += timer_get_1hzp();
	if (buttons_get_v()) goto exit; // Will not run when User is using the device.
#if D1W_SCAN_INTERVAL
	if ((d1w_state==D1W_IDLE)&&(timer_get() > (d1w_last_scan_time+D1W_SCAN_INTERVAL))) d1w_rescan=1;
#else
	if ((d1w_state==D1W_IDLE)&&(d1w_devicecnt == 0)&&(d1w_run)) d1w_rescan = 1;
#endif
	if (d1w_rescan) {
		d1w_rescan=0;
		d1w_scan();
		d1w_run++;
		d1w_state = D1W_IDLE;
		goto exit;
	}

	if ((d1w_run)&&(d1w_state==D1W_IDLE)) {
		d1w_run=0;
		d1w_tmproffs = 0;
		d1w_tmpidx = 0;
		d1w_state = D1W_TEMPQ;
	}

	if ((d1w_state == D1W_TEMPQ)||(d1w_state == D1W_TEMPR)) {
		if ((d1w_tmproffs>=d1w_devicecnt)||(d1w_tmpidx >= D1W_MAX_TEMPDEVS)) {
			d1w_state = D1W_IDLE;
			for(;d1w_tmpidx<D1W_MAX_TEMPDEVS;d1w_tmpidx++) d1w_temps[d1w_tmpidx] = -999;
		} else {
			if (d1w_devices[d1w_tmproffs][0] != 0x10) {
				d1w_tmproffs++;
				d1w_state = D1W_TEMPQ;
			} else {
				if (d1w_state == D1W_TEMPR) {
					if (d1w_rbit()) {
						d1w_temp_read();
						d1w_state = D1W_TEMPQ;
						d1w_tmproffs++;
						d1w_tmpidx++;
					}
				} else {
					d1w_temp_convert();
					d1w_state = D1W_TEMPR;
				}
			}
		}
	}
exit:
	if ((d1w_run)||(d1w_rescan)||(d1w_state!=D1W_IDLE)) timer_set_waiting();
}

int16_t dallas_temp_get(uint8_t idx) {
	if (idx>=D1W_MAX_TEMPDEVS) return -999;
	return d1w_temps[idx];
}
