/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/crc16.h>
#include "stdbool.h"
#include "led.h"
#include "radio.h"

uint16_t _radio_shift = 0x0000;

volatile uint16_t _dac_value = 0;
volatile uint8_t sample = 0;
volatile int32_t _transition_delta = 0;
volatile uint16_t _transition_start = 0;
volatile bool transition_complete = true;

// RTTY stuff
volatile uint8_t systicks = 0;
volatile uint8_t _txbyte = 0;
volatile uint8_t _txptr = 0;
volatile bool byte_complete = false;

volatile uint16_t bits_remain = 0;
volatile uint8_t *binary_seq;
volatile uint8_t out_mask = 0x80;

volatile uint8_t radio_mode = 0;   //0-fsk; 1-afsk

uint8_t EEMEM step[50] = {4, 7, 11, 15, 19, 23, 28, 32, 37, 42, 47, 52, 57, 
    62, 67, 73, 78, 84, 90, 95, 101, 107, 113, 119, 125, 130, 136, 142, 148,
    154, 160, 165, 171, 177, 182, 188, 193, 198, 203, 208, 213, 218, 223, 227, 
    232, 236, 240, 244, 248, 251};

#define SIN_FULL_LEN 250
#define SIN_HALF_LEN 125	 //todo: use symmetry (not really much point in going 1/2 -> 1/4 sine)
const static uint8_t sin_table[SIN_HALF_LEN] = {128, 131, 134, 137, 140, 143, 146, 
	149, 152, 156, 159, 162, 165, 168, 171, 173, 176, 179, 182, 185, 188, 190, 193, 
	196, 198, 201, 203, 206, 208, 211, 213, 215, 218, 220, 222, 224, 226, 228, 230, 
	231, 233, 235, 236, 238, 239, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 
	250, 251, 251, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 251, 251, 250, 
	250, 249, 248, 247, 246, 245, 244, 243, 242, 241, 239, 238, 236, 235, 233, 231, 
	230, 228, 226, 224, 222, 220, 218, 215, 213, 211, 208, 206, 203, 201, 198, 196, 
	193, 190, 188, 185, 182, 179, 176, 173, 171, 168, 165, 162, 159, 156, 152, 149, 
	146, 143, 140, 137, 134, 131};/*, 128, 124, 121, 118, 115, 112, 109, 106, 103, 99, 
	96, 93, 90, 87, 84, 82, 79, 76, 73, 70, 67, 65, 62, 59, 57, 54, 52, 49, 47, 44, 
	42, 40, 37, 35, 33, 31, 29, 27, 25, 24, 22, 20, 19, 17, 16, 14, 13, 12, 11, 10, 
	9, 8, 7, 6, 5, 5, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 5, 5, 6, 7, 8, 9, 10, 
	11, 12, 13, 14, 16, 17, 19, 20, 22, 24, 25, 27, 29, 31, 33, 35, 37, 40, 42, 44, 
	47, 49, 52, 54, 57, 59, 62, 65, 67, 70, 73, 76, 79, 82, 84, 87, 90, 93, 96, 99, 
	103, 106, 109, 112, 115, 118, 121, 124}; */

#define FREQ_HIGH 4
#define FREQ_LOW 3	
	
volatile uint8_t sin_phase;
volatile uint8_t sin_phase_inc = FREQ_HIGH;


void set_afsk()
{
	radio_mode = 1;	
}

void set_fsk()
{
	radio_mode = 0;	
}

/**
 * Initialise the radio subsystem including the dual 16 bit 
 * DAC.
 */
void radio_init(void)
{
    // Configure the slave select pin and set it high
    RADIO_DDR |= _BV(RADIO_SS) | _BV(RADIO_MOSI) | _BV(RADIO_SCK);
    RADIO_DDR &= ~(_BV(RADIO_MISO));
    RADIO_PORT |= _BV(RADIO_SS);

    // Set MSB first, sample on rising edge, 
    // clock idles low
    SPCR &= ~(_BV(DORD) | _BV(CPOL) | _BV(CPHA));

    // Enable SPI, set master mode and fosc/2, giving an SPI
    // interface SCK speed of 8MHz
    SPSR |= _BV(SPI2X);
    SPCR &= ~(_BV(SPR0) | _BV(SPR1));
    SPCR |= _BV(MSTR) | _BV(SPE);

    // Set up TIMER0 to tick once per symbol and interrupt
    // CTC mode
    TCCR0A |= _BV(WGM01);

    // Prescale by 1024
    TCCR0B |= _BV(CS02) | _BV(CS00);

    // Interrupt on compare match with OCR0A
    set_baud_50();

    // Set up TIMER2 for the DSP (!) stuff
    // No clock prescale to get 62.5kHz sample rate with an 8 bit timer
    TCCR2B |= _BV(CS20);

    // Do not interrupt on overflow for now
    TIMSK2 &= ~(_BV(TOIE2));
	
	//now turn on interrupts :P
	TIMSK2 |= _BV(TOIE2);

    // Turn off the DAC
    _radio_dac_off();

    // Enable global interrupts
    sei();
}

void set_baud_50(void)
{
    OCR0A = 156;
}

void set_baud_300(void)
{
    OCR0A = 25;
}

/**
 * Enable the power amplifier on the Micrel radio
 */
void radio_enable(void)
{
    RADIO_EN_DDR |= _BV(RADIO_EN);
    RADIO_EN_PORT |= _BV(RADIO_EN);
    _delay_ms(100);
}

/**
 * Disable the power amp on the Micrel radio to shut it down
 */
void radio_disable(void)
{
    RADIO_EN_DDR |= _BV(RADIO_EN);
    RADIO_EN_PORT &= ~(_BV(RADIO_EN));
}

/**
 * Write a value to one of the DAC channels
 */
void _radio_dac_write(uint8_t channel, uint16_t value)
{
    // Take SS low
    RADIO_PORT &= ~(_BV(RADIO_SS));

    // Write cmd then value to the SPI data register
    SPDR = 0x30 | (channel & 0x01);
    while(!(SPSR & _BV(SPIF)));
    SPDR = value >> 8;
    while(!(SPSR & _BV(SPIF)));
    SPDR = value & 0xFF;
    while(!(SPSR & _BV(SPIF)));

    // Raise SS to signal end of transaction
    RADIO_PORT |= _BV(RADIO_SS);

    // Update the DAC value
    if(channel == RADIO_FINE) _dac_value = value;
}

/**
 * Power down the DAC and set the outputs to a high-Z state
 */
void _radio_dac_off(void)
{
    // Take SS low
    RADIO_PORT &= ~(_BV(RADIO_SS));

    // Write cmd then value to the SPI data register
    SPDR = 0x4F;
    while(!(SPSR & _BV(SPIF)));
    SPDR = 0;
    while(!(SPSR & _BV(SPIF)));
    SPDR = 0;
    while(!(SPSR & _BV(SPIF)));

    // Raise SS to signal end of transaction
    RADIO_PORT |= _BV(RADIO_SS);
}

/**
 * Transmit the given string over the radio terminated with a checksum
 * and a newline, compatible with the UKHAS habitat listener and parser.
 */
void radio_transmit_sentence(char* string)
{
    radio_transmit_string(string);
    
    // Calculate the checksum and send it
    uint16_t checksum = radio_calculate_checksum(string);
    char cs[7];
    sprintf(cs, "*%04X\n", checksum);
    radio_transmit_string(cs);
}

void radio_transmit_sentence_binary(uint8_t* string, uint16_t bits)
{
    TIMSK2 |= _BV(TOIE2);
	bits_remain = bits;
	binary_seq = string;
	out_mask = 0x80;
	TIMSK0 |= _BV(OCIE0A);
	while (bits_remain & 0xFF00);
	while (bits_remain & 0x00FF);
	//while(1)
	//{
	//	_delay_ms(10);
	//	cli();
	//	if (bits_remain == 0)
	//		break;
	//	sei();
	//}
	
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = 'J';
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = (bits_remain&0xFF) + 48;


	//TIMSK0 &= ~(_BV(OCIE0A));

}


/**
 * Transmit a null terminated string over the radio link.
 */
void radio_transmit_string(char* string)
{
    while(*string)
    {
        _txbyte = *string;
        _txptr = 0;
        byte_complete = false;
        TIMSK0 |= _BV(OCIE0A);
        while(!byte_complete);
        wdt_reset();
        string++;
    }
}

/**
 * Set the radio shift
 */
void radio_set_shift(uint16_t shift)
{
    _radio_shift = shift;
}

/**
 * Set the baud rate by setting the compare value for TIMER0
 */
void radio_set_baud(uint8_t baud)
{
    OCR0A = baud;
}

/**
 * Transition using the FIR filter
 */
void _radio_transition(uint16_t target)
{
    if( _dac_value == target ) return;

    // Update the global target and initial
    _transition_start = _dac_value;
    _transition_delta = (int32_t)target - (int32_t)_transition_start;
    transition_complete = false;

    // Start the DSP timer interrupting
    TIMSK2 |= _BV(TOIE2);
}

/**
 * Transmit a single bit at a pointer, also coping with start and 
 * stop bits.
 */
void _radio_transmit_bit(uint8_t data, uint8_t ptr)
{
	if (radio_mode){
		if(ptr == 0)
			sin_phase_inc = FREQ_LOW; //_radio_transition(0);
		else if(ptr >= 1 && ptr <= 7)
			if( (data >> (ptr - 1)) & 1 )
				sin_phase_inc = FREQ_HIGH; //_radio_transition(_radio_shift);
			else
				sin_phase_inc = FREQ_LOW; //_radio_transition(0);
		else
			sin_phase_inc = FREQ_HIGH; //_radio_transition(_radio_shift);
	}
	else
	{
		if(ptr == 0)
			_radio_dac_write(RADIO_FINE, (uint16_t)0); //_radio_transition(0);
		else if(ptr >= 1 && ptr <= 7)
			if( (data >> (ptr - 1)) & 1 )
				_radio_dac_write(RADIO_FINE, (uint16_t)_radio_shift); //_radio_transition(_radio_shift);
			else
				_radio_dac_write(RADIO_FINE, (uint16_t)0); //_radio_transition(0);
		else
			_radio_dac_write(RADIO_FINE, (uint16_t)_radio_shift); //_radio_transition(_radio_shift);
	}
}



/**
 * Calculate the checksum for the radio string excluding any $ signs
 * at the start.
 */
uint16_t radio_calculate_checksum(char* data)
{
    uint16_t i = 0;
    uint16_t crc = 0xFFFF;
   
    
    for (i=0; i < strlen(data); i++) {
        if (data[i] == '$') break;
    }
    for (; i < strlen(data); i++) {
        if (data[i] != '$') crc = _crc_xmodem_update(crc,(uint8_t)data[i]);
    }
    return crc;
}

/**
 * Make some noise on the radio on the 2FSK tones as a pre/post amble
 * such that Joey can be easily identified.
 */
void radio_chatter(void)
{
    _radio_dac_write(RADIO_FINE, 0x0000);
    _delay_ms(200);
    _radio_dac_write(RADIO_FINE, 0xFFFF);
    _delay_ms(200);
    _radio_dac_write(RADIO_FINE, 0x0000);
    _delay_ms(200);
    _radio_dac_write(RADIO_FINE, 0xFFFF);
    _delay_ms(200);
}

/**
 * Interrupt handle for the radio timer, when we reach the systicks
 * limit we should transmit the next bit
 */
ISR(TIMER0_COMPA_vect)
{
    if( systicks < 1 )
    {
        systicks++;
    }
    else
    {
		if (bits_remain > 0)   //binary protocol
		{
		
			if (radio_mode){
				if (*binary_seq & out_mask)
					sin_phase_inc = FREQ_HIGH;
				else
					sin_phase_inc = FREQ_LOW;
			}else{			
				if (*binary_seq & out_mask)
					_radio_dac_write(RADIO_FINE, (uint16_t)_radio_shift); //_radio_transition(_radio_shift);
				else
					_radio_dac_write(RADIO_FINE, (uint16_t)0); //_radio_transition(0);
			}
			
			//if (*binary_seq & out_mask)
			//	led_set(LED_RED, 1);
			//else
			//	led_set(LED_RED, 0);
				
			out_mask >>= 1;
			if (out_mask == 0)
			{
				out_mask = 0x80;
				binary_seq++;
				wdt_reset();
			}
			bits_remain--;
		
		}		
		else    //rtty protocol
		{
			if( _txptr < 10 )
			{
				_radio_transmit_bit(_txbyte, _txptr);
				_txptr++;
			} else {
				TIMSK0 &= ~(_BV(OCIE0A));
				byte_complete = true;
			}
		} 
        systicks = 0;
    }
}

/**
 * Interrupt handler for the DSP timer. Read out the next step
 * response value and write it to the DAC.
 */
ISR(TIMER2_OVF_vect)
{

	if (radio_mode)
	{
		sin_phase += sin_phase_inc;
		if ( sin_phase >= SIN_FULL_LEN)	//this only works as i know it wont overflow as sin_phase_inc is 3 or 4
			sin_phase -= SIN_FULL_LEN;
			
		if ( sin_phase >= SIN_HALF_LEN)
			_radio_dac_write(RADIO_FINE, (uint16_t)(255-(sin_table[sin_phase-SIN_HALF_LEN])) << 8);
		else
			_radio_dac_write(RADIO_FINE, ((uint16_t)sin_table[sin_phase]) << 8);
	}
	else
	{
		
		if( sample < DSP_SAMPLES )
		{
			int32_t d = _transition_delta * (int32_t)(eeprom_read_byte(&step[sample]));
			d /= 256;
			d += (int32_t)_transition_start;
			_radio_dac_write(RADIO_FINE, (uint16_t)d);
			sample++;
		} else {
			TIMSK2 &= ~(_BV(TOIE2));
			sample = 0;
			transition_complete = true;
		}
	
	}
	
}
