/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <avr/wdt.h>

#include "led.h"
#include "radio.h"
#include "gps.h"
#include "temperature.h"

#include "libturbohab.h"
#include "cmp.h"

// 30kHz range on COARSE, 3kHz on FINE

char s[100];
uint8_t sb[100];
uint32_t EEMEM ticks = 0;

char debug[100];

#define HB_BUF_LEN 100
uint8_t hb_buf[HB_BUF_LEN] = {0};
uint8_t hb_buf_ptr = 0;
uint8_t hb_buf_out[3*HB_BUF_LEN] = {0};


static bool read_bytes(void *data, size_t sz, FILE *fh) {
    return fread(data, sizeof(uint8_t), sz, fh) == (sz * sizeof(uint8_t));
}

static bool file_reader(cmp_ctx_t *ctx, void *data, size_t limit) {
    return read_bytes(data, limit, (FILE *)ctx->buf);
}

static size_t file_writer(cmp_ctx_t *ctx, const void *data, size_t count) {

	if (hb_buf_ptr+count > HB_BUF_LEN)
		return -1;
		
	for (uint16_t i = 0; i < count; i++)
	{
		hb_buf[hb_buf_ptr] = *((uint8_t*)data);
		data++;
		hb_buf_ptr++;
	}
	return count;
    //return fwrite(data, sizeof(uint8_t), count, (FILE *)ctx->buf);
}

void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );

	/* Put data into buffer, sends the data */
	UDR0 = data;
}

void TxStr(char *str)
{
	while(*str)
	{
		USART_Transmit(*str);
		str++;
	}

}


int main()
{
    // Disable, configure, and start the watchdog timer
    wdt_disable();
    wdt_reset();
    wdt_enable(WDTO_8S);

    // Start and configure all hardware peripherals
    sei();
    led_init();
    temperature_init();
    radio_init();
    gps_init();
    radio_enable();

    // Set the radio shift and baud rate
    _radio_dac_write(RADIO_COARSE, RADIO_CENTER_FREQ_434630);
    _radio_dac_write(RADIO_FINE, 0);
    radio_set_shift(RADIO_SHIFT_425);
    radio_set_baud(RADIO_BAUD_50);

    // Radio chatter
    for(uint8_t i = 0; i < 5; i++)
    {
        radio_chatter();
        wdt_reset();
    }
	
	
    
    int32_t lat = 0, lon = 0, alt = 0;
    uint8_t hour = 0, minute = 0, second = 0, lock = 0, sats = 0;

	uint8_t toggle = 0;
	
    while(true)
    {
        led_set(LED_GREEN, 1);

        // Get the current system tick and increment
        uint32_t tick = eeprom_read_dword(&ticks) + 1;

        // Get temperature from the TMP100
        float temperature;
		if (toggle == 0)
		    temperature = temperature_read();

        // Check that we're in airborne <1g mode
        if( gps_check_nav() != 0x06 ) led_set(LED_RED, 1);

        // Get information from the GPS
        gps_check_lock(&lock, &sats);
        if( lock == 0x02 || lock == 0x03 || lock == 0x04 )
        {
            gps_get_position(&lat, &lon, &alt);
            gps_get_time(&hour, &minute, &second);
        }

        led_set(LED_GREEN, 0);

        // Format the telemetry string & transmit
        double lat_fmt = (double)lat / 10000000.0;
        double lon_fmt = (double)lon / 10000000.0;
        alt /= 1000;

		if (toggle==0) //rtty
		{
			toggle = 1;
			set_fsk();
		
			sprintf(s, "UUUX$$JOEY,%lu,%02u:%02u:%02u,%02.7f,%03.7f,%ld,%.1f,%u,%x",
				tick, hour, minute, second, lat_fmt, lon_fmt, alt, temperature,
				sats, lock);
			s[3] = 0x80;  //null with 7n2
			//radio_chatter();
			radio_transmit_sentence(s);
			//radio_chatter();
		
		}
		else  //binary
		{
			toggle++;
			if (toggle > 3)
				toggle = 0;
			set_afsk();
		
			memset((void*)hb_buf,0,100);
			memset((void*)hb_buf_out,0,300);
		
			cmp_ctx_t cmp;
			hb_buf_ptr = 0;
			cmp_init(&cmp, (void*)hb_buf, file_reader, file_writer);
		
			//cmp_write_array(&cmp, 2);
			//cmp_write_str(&cmp, "Hello", 5);
			//cmp_write_str(&cmp, "MessagePack", 11);
			
			cmp_write_map(&cmp, 6);
			
			
			cmp_write_uint(&cmp, 0);
			cmp_write_str(&cmp, "JOEY", 4);
			
			cmp_write_uint(&cmp, 1);
			cmp_write_uint(&cmp, tick);
			
			cmp_write_uint(&cmp, 2);
			cmp_write_uint(&cmp, (uint32_t)hour*(3600) + (uint32_t)minute*60 + (uint32_t)second);
			
			cmp_write_uint(&cmp, 3);
			cmp_write_array(&cmp, 3);
			cmp_write_sint(&cmp, lat);
			cmp_write_sint(&cmp, lon);
			cmp_write_sint(&cmp, alt);
			
			cmp_write_uint(&cmp, 4);
			cmp_write_uint(&cmp, sats);
			
			cmp_write_uint(&cmp, 5);
			cmp_write_uint(&cmp, lock);
			
			//cmp_write_uint(&cmp, 10);
			//cmp_write_uint(&cmp, 120);
		

			uint16_t l = channel_encode(hb_buf,hb_buf_out,328,INT_C_328,3);
			
			//snprintf(debug,100,"LEN: %d",l);
			//TxStr(debug);
			radio_transmit_sentence_binary(hb_buf_out,l);
			
			
			
			if (toggle == 0)
				set_fsk();
		
		}

        led_set(LED_RED, 0);
        eeprom_update_dword(&ticks, tick);
        wdt_reset();
        //_delay_ms(500);
    }

    return 0;
}
