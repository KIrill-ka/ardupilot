// -------------------------------------------------------------
// SBUS PPM Encoder v2
// -------------------------------------------------------------
// 
// Original code and PPM encoder functionality derived from PPM_Encoder.h
// By: John Arne Birkeland - 2012
// APM v1.x adaptation and "difficult" receiver testing by Olivier ADLER
//
// Author of S.BUS code: Kirill A. Kornilov

// -------------------------------------------------------------
// ARDUPPM OPERATIONAL DESCRIPTION
// -------------------------------------------------------------

// APM 2.x LED STATUS:
// -------------------
// RX - OFF         = No input signal detected
// RX - SLOW TOGGLE = Input signal OK
// TX - OFF         = PPM output disabled
// TX - FAST TOGGLE = PPM output enabled
// ERR - OFF 		= No erorrs
// ERR - ON 		= No input signal
// ERR - TOGGLE (uneven) = S.bus errors / skipped frames (this may be due to USB processing)

// -------------------------------------------------------------

#ifndef _PPM_ENCODER_SBUS_H_
#define _PPM_ENCODER_SBUS_H_

#include <avr/io.h>

// -------------------------------------------------------------

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>


// -------------------------------------------------------------
// SERVO INPUT FILTERS AND PARAMETERS
// -------------------------------------------------------------
// Using both filters is not recommended and may reduce servo input resolution

//#define _AVERAGE_FILTER_            // Average filter to smooth servo input capture jitter
#define _JITTER_FILTER_ 2           // Cut filter to remove servo input capture jitter (1 unit = 0.5us)
#define INPUT_ERROR_LED_DELAY 50
// -------------------------------------------------------------

#ifndef F_CPU
#define F_CPU             16000000UL
#endif

#ifndef true
#define true                1
#endif

#ifndef false
#define false               0
#endif

#ifndef bool
#define bool                _Bool
#endif

// Version stamp for firmware hex file ( decode hex file using <avr-objdump -s file.hex> and look for "ArduPPM" string )
const char ver[15] = "ArduPPMsbus-v2"; 

//#define SBUS_DEBUG
#ifdef SBUS_DEBUG
// PB0 is sbus input. Connect all servo pins to logic analyzer to see errors in correlation with sbus signal.
#define ERR_START (1<<PB1)
#define ERR_PARITY (1<<PB2)
#define ERR_STOP (1<<PB3)
#define ERR_PIN_CHANGE (1<<PB4)
#define ERR_TIMEOUT (1<<PB5)
#define ERR_FERR (1<<PB6)
#define ERR_INT_SKIP (1<<PB7)

#define DEBUG_ERROR(X)  PINB |= X
#else
#define DEBUG_ERROR(X)
#endif


// -------------------------------------------------------------
// FAILSAFE MODE
// -------------------------------------------------------------

//#define _APM_FAILSAFE_   // Used to spesify APM 800us channel loss fail safe values, remove to use normal fail safe values (stand alone encoder board)

//#define _THROTTLE_LOW_FAILSAFE_INDICATION //if set, throttle is set to low when a single channel is lost
//#define _THROTTLE_LOW_RECOVERY_POSSIBLE //if set, throttle low recovers from being low when the single channel comes back, only makes sense together with _THROTTLE_LOW_FAILSAFE_INDICATION

#if defined _THROTTLE_LOW_RECOVERY_POSSIBLE && !defined _THROTTLE_LOW_FAILSAFE_INDICATION
#error failsafe recovery is only possible with throttle_low_failsafe_indication defined as well
#endif

// -------------------------------------------------------------
// SERVO LIMIT VALUES
// -------------------------------------------------------------

// Number of Timer1 ticks in one microsecond
#define ONE_US                F_CPU / 8 / 1000 / 1000

// 400us PPM pre pulse
#define PPM_PRE_PULSE         ONE_US * 400

// Servo minimum position
#define PPM_SERVO_MIN         ONE_US * 900 - PPM_PRE_PULSE

// Servo center position
#define PPM_SERVO_CENTER      ONE_US * 1500 - PPM_PRE_PULSE

// Servo maximum position
#define PPM_SERVO_MAX         ONE_US * 2100 - PPM_PRE_PULSE

// Throttle default at power on
#define PPM_THROTTLE_DEFAULT  ONE_US * 1100 - PPM_PRE_PULSE

// Throttle during failsafe
#define PPM_THROTTLE_FAILSAFE ONE_US * 900 - PPM_PRE_PULSE

// Channel loss failsafe
#define PPM_CHANNEL_LOSS ONE_US * 800 - PPM_PRE_PULSE

// CH5 power on values (mode selection channel)
#define PPM_CH5_MODE_4        ONE_US * 1555 - PPM_PRE_PULSE

// -------------------------------------------------------------
// PPM OUTPUT SETTINGS
// -------------------------------------------------------------
// #define _POSITIVE_PPM_FRAME_    // Switch to positive pulse PPM
// (the actual timing is encoded in the length of the low between two pulses)

// Number of servo input channels
#define SERVO_CHANNELS       12

// PPM period 18.5ms - 26.5ms (54hz - 37Hz) 
#define PPM_PERIOD            ONE_US * ( 22500 - ( SERVO_CHANNELS * 1500 ) )

// Size of ppm[..] data array ( servo channels * 2 + 2)
#define PPM_ARRAY_MAX         (SERVO_CHANNELS*2 + 2)

#ifdef _APM_FAILSAFE_
// -------------------------------------------------------------
// APM FAILSAFE VALUES
// -------------------------------------------------------------
volatile uint16_t failsafe_ppm[ PPM_ARRAY_MAX ] =                               
{
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 1
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 2
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 3 (throttle)
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 4
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 5
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 6
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 7
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 8
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 9
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 10
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 11
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 12
    PPM_PRE_PULSE,
    PPM_PERIOD
};
#else
// -------------------------------------------------------------
// SERVO FAILSAFE VALUES
// -------------------------------------------------------------
volatile uint16_t failsafe_ppm[ PPM_ARRAY_MAX ] =                               
{
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 1
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 2
    PPM_PRE_PULSE,
    PPM_THROTTLE_FAILSAFE,    // Channel 3 (throttle)
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 4
    PPM_PRE_PULSE,
    PPM_CH5_MODE_4,           // Channel 5
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 6
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 7
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 8
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 8
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 8
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 8
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 8
    PPM_PRE_PULSE,
    PPM_PERIOD
};
#endif

// -------------------------------------------------------------
// Data array for storing ppm (8 channels) pulse widths.
// -------------------------------------------------------------
volatile uint16_t ppm[ PPM_ARRAY_MAX ] =                                
{
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 1 
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 2
    PPM_PRE_PULSE,
    PPM_THROTTLE_DEFAULT,     // Channel 3 (throttle)
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 4
    PPM_PRE_PULSE,
    PPM_CH5_MODE_4,           // Channel 5
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 6
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 7
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 8
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 9
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 10
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 11
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 12
    PPM_PRE_PULSE,
    PPM_PERIOD
};

// -------------------------------------------------------------
// Data arraw for storing ppm timeout (missing channel detection)
// -------------------------------------------------------------
#define PPM_TIMEOUT_VALUE 40 // ~1sec before triggering missing channel detection
volatile uint8_t ppm_timeout[ PPM_ARRAY_MAX ];

// Servo input channel connected status
#define SERVO_INPUT_CONNECTED_VALUE 100
volatile uint8_t servo_input_connected[ PPM_ARRAY_MAX ];

#ifdef _THROTTLE_LOW_RECOVERY_POSSIBLE
// count the channels which have been once connected but then got disconnected
volatile uint8_t disconnected_channels;
#endif

// AVR parameters for PhoneDrone and APM2 boards using ATmega32u2
#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)

#define SERVO_DDR             DDRB
#define SERVO_PORT            PORTB
#define SERVO_INPUT           PINB
#define SERVO_INT_VECTOR      PCINT0_vect
#define SERVO_INT_MASK        PCMSK0
#define SERVO_INT_CLEAR_FLAG  PCIF0
#define SERVO_INT_ENABLE      PCIE0
#define SERVO_TIMER_CNT       TCNT1

#define PPM_DDR               DDRC
#define PPM_PORT              PORTC
#define PPM_OUTPUT_PIN        PC6
#define PPM_INT_VECTOR        TIMER1_COMPA_vect
#define PPM_COMPARE           OCR1A
#define PPM_COMPARE_FLAG      COM1A0
#define PPM_COMPARE_ENABLE    OCIE1A
#define PPM_COMPARE_FORCE_MATCH    FOC1A

#define    USB_DDR            DDRC
#define    USB_PORT           PORTC
#define    USB_PIN            PC2

// true if we have received a USB device connect event
static bool usb_connected;

// USB connected event
void EVENT_USB_Device_Connect(void)
{
    // Toggle USB pin high if USB is connected
    USB_PORT |= (1 << USB_PIN);

    usb_connected = true;

    // this unsets the pin connected to PA1 on the 2560
    // when the bit is clear, USB is connected
    PORTD &= ~1;
}

// USB disconnect event
void EVENT_USB_Device_Disconnect(void)
{
    // toggle USB pin low if USB is disconnected
    USB_PORT &= ~(1 << USB_PIN);

    usb_connected = false;

    // this sets the pin connected to PA1 on the 2560
    // when the bit is clear, USB is connected
    PORTD |= 1;
}

// AVR parameters for ArduPilot MEGA v1.4 PPM Encoder (ATmega328P)
#elif defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)

#define SERVO_DDR             DDRD
#define SERVO_PORT            PORTD
#define SERVO_INPUT           PIND
#define SERVO_INT_VECTOR      PCINT2_vect
#define SERVO_INT_MASK        PCMSK2
#define SERVO_INT_CLEAR_FLAG  PCIF2
#define SERVO_INT_ENABLE      PCIE2
#define SERVO_TIMER_CNT       TCNT1

#define PPM_DDR               DDRB
#define PPM_PORT              PORTB
#define PPM_OUTPUT_PIN        PB2
#define PPM_INT_VECTOR        TIMER1_COMPB_vect
#define PPM_COMPARE           OCR1B
#define PPM_COMPARE_FLAG      COM1B0
#define PPM_COMPARE_ENABLE    OCIE1B
#define PPM_COMPARE_FORCE_MATCH    FOC1B

#else
#error NO SUPPORTED DEVICE FOUND! (ATmega16u2 / ATmega32u2 / ATmega328p)
#endif
    
// Used to indicate invalid SERVO input signals
volatile uint8_t servo_input_errors = 0;

// Used to indicate if PPM generator is active
volatile bool ppm_generator_active = false;

// Used to indicate a brownout restart
volatile bool brownout_reset = false;

#ifdef _THROTTLE_LOW_FAILSAFE_INDICATION
// Used to force throttle fail-safe mode (RTL)
volatile bool throttle_failsafe_force = false;
#endif

// ------------------------------------------------------------------------------
// PPM GENERATOR START - TOGGLE ON COMPARE INTERRUPT ENABLE
// ------------------------------------------------------------------------------
void ppm_start( void )
{
        // Prevent reenabling an already active PPM generator
        if( ppm_generator_active ) return;
        
        // Store interrupt status and register flags
        volatile uint8_t SREG_tmp = SREG;

        // Stop interrupts
        cli();

        // Make sure initial output state is low
        PPM_PORT &= ~(1 << PPM_OUTPUT_PIN);
        
        // Wait for output pin to settle
        //_delay_us( 1 );

        // Set initial compare toggle to maximum (32ms) to give other parts of the system time to start
        SERVO_TIMER_CNT = 0;
        PPM_COMPARE = 0xFFFF;
        TIFR1 |= 1<<OCF1A;

        // Set toggle on compare output
        TCCR1A = (1 << PPM_COMPARE_FLAG);

        
        #if defined (_POSITIVE_PPM_FRAME_)
        // Force output compare to reverse polarity
        TCCR1C |= (1 << PPM_COMPARE_FORCE_MATCH);
        #endif

        // Enable output compare interrupt
        TIMSK1 |= (1 << PPM_COMPARE_ENABLE);

        // Indicate that PPM generator is active
        ppm_generator_active = true;

        #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)        
        // Turn on TX led if PPM generator is active
        PORTD &= ~( 1<< PD5 );
        #endif

        // Restore interrupt status and register flags
        SREG = SREG_tmp;
}

// ------------------------------------------------------------------------------
// PPM GENERATOR STOP - TOGGLE ON COMPARE INTERRUPT DISABLE
// ------------------------------------------------------------------------------
void ppm_stop( void )
{
        // Store interrupt status and register flags
        volatile uint8_t SREG_tmp = SREG;

        // Stop interrupts
        cli();

        // Disable output compare interrupt
        TIMSK1 &= ~(1 << PPM_COMPARE_ENABLE);

        // Reset TIMER1 registers
        TCCR1A = 0;
        TCCR1B = 0;

        // Indicate that PPM generator is not active
        ppm_generator_active = false;

        #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)        
        // Turn off TX led if PPM generator is off
        PORTD |= ( 1<< PD5 );
        #endif
        
        // Restore interrupt status and register flags
        SREG = SREG_tmp;
}

// ------------------------------------------------------------------------------
// Watchdog Interrupt (interrupt only mode, no reset)
// ------------------------------------------------------------------------------
ISR( WDT_vect ) // If watchdog is triggered then enable missing signal flag and copy power on or failsafe positions
{
    // Use failsafe values if PPM generator is active or if chip has been reset from a brown-out
    if ( ppm_generator_active || brownout_reset )
    {
        // Copy failsafe values to ppm[..]
        //for( uint8_t i = 0; i < PPM_ARRAY_MAX; i++ )
        //{
        //  ppm[ i ] = failsafe_ppm[ i ];
        //}
        
        // Set Throttle Low & leave other channels at last value
        ppm[5] = failsafe_ppm[ 5 ];
        #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)        
        PORTC &= ~(1<<PINC7); /* turn on */
        #endif
    }

    if( brownout_reset )
    {
        // Start PPM generator
        ppm_start();
        brownout_reset = false;
    }


    
}


static volatile uint8_t sbus_data[25];
static uint8_t sbus_bit_parity;
static uint8_t sbus_shift_reg;
static uint8_t sbus_bit_count;
static volatile uint8_t sbus_state;
static uint8_t sbus_byte;

#define SBUS_NONE 0
#define SBUS_TIMER_OVF 1
#define SBUS_DATA 2
#define SBUS_STOP_BIT 3
#define SBUS_NEXT_BYTE 4
#define SBUS_FRAME_READY 5

static inline void sbus_err(void) {
                 servo_input_errors = 1;
                 sbus_state = SBUS_NONE;
                 TIMSK0 = (1<<TOIE0);
                 TCNT0 = 0;
}

#define push_bit(a, b)  asm ( \
                        "lsr %1\n\t" \
                        "ror %0" \
                        : "=r" (a), "=r" (b) \
                        : "0" (a), "1" (b) \
                        : "cc" \
            );
ISR(TIMER0_COMPA_vect)
{
 uint8_t bit;
 uint8_t bc;
 uint8_t b;
 uint8_t x;

 switch(sbus_state) {
         case SBUS_DATA:
                 bit = SERVO_INPUT; // we need only bit 0
                 sbus_bit_parity ^= bit;
                 bc = sbus_bit_count;
                 OCR0A += 20;
                 if(bc != 8) {
#if 0
                  sbus_shift_reg = (sbus_shift_reg<<1) | bit;
#else
                  //sbus_shift_reg >>= 1;
                  //if(bit) sbus_shift_reg |= 0x80;
                  x = sbus_shift_reg;
                  push_bit(x, bit);
                  sbus_shift_reg = x;
#endif
                  sbus_bit_count = bc + 1;
                  return;
                 } 
                 if(!(sbus_bit_parity&1)) {
                  sbus_err();
                  DEBUG_ERROR(ERR_PARITY);
                  return;
                 }
                 sbus_state = SBUS_STOP_BIT;
                 return;
         case SBUS_STOP_BIT: /* first stop bit */
                 if(SERVO_INPUT & 1) { 
                  DEBUG_ERROR(ERR_STOP);
                  sbus_err();
                  return;
                 }
                 TCNT0 = 0;
                 TIMSK0 = (1<<TOIE0);
                 //PCIFR |= (1<<PCIF0);
                 SERVO_INT_MASK = 1;
                 b = sbus_byte;
                 sbus_data[b++] = ~sbus_shift_reg;
                 sbus_byte = b;
                 if(b == 25) {
                  sbus_state = SBUS_FRAME_READY;
                  SERVO_INT_MASK = 0;
                 } else sbus_state = SBUS_NEXT_BYTE;
 }
}

ISR(TIMER0_COMPB_vect)
{
 switch(sbus_state) {
         case SBUS_NONE:
                 TIMSK0 = 0;
                 sbus_state = SBUS_TIMER_OVF; /* this is what we exect, minimal pause between 
                                                 frames has passed, we are ready to accept the
                                                 next one */
                 break;
         case SBUS_FRAME_READY:
                 TCNT0 = 0; /* previous frame is not processed yet: wait for some time, then retry */
                 OCR0B = 200; /* 100 us */
                 break;
         default: /* SBUS_NEXT_BYTE: timed out waiting for the next byte */
                 TIMSK0 = 0;
                 sbus_state = SBUS_TIMER_OVF;
                 servo_input_errors = 1;
                 DEBUG_ERROR(ERR_TIMEOUT);
                 break;
 }
                 
 //PCIFR |= (1<<PCIF0);
 SERVO_INT_MASK = 1;
}

ISR(TIMER0_OVF_vect)
{
 switch(sbus_state) {
         case SBUS_DATA:
         case SBUS_STOP_BIT:
                 DEBUG_ERROR(ERR_INT_SKIP); /* COMPA interrupt handler is late */
                 sbus_err();
                 break;
         default: /* SBUS_FRAME_READY, SBUS_NEXT_BYTE, SBUS_NONE: continue to wait until COMPB int */
                 TCNT0 = 0;
                 OCR0B = 64; /* + 256 = 160 us */
                 TIFR0 = (1<<OCF0B);
                 TIMSK0 = (1<<OCIE0B);
 }
}

// ------------------------------------------------------------------------------
// SERVO/PPM INPUT - PIN CHANGE INTERRUPT
// ------------------------------------------------------------------------------
ISR( SERVO_INT_VECTOR )
{
 TCNT0 = 0;
 switch(sbus_state) {
         case SBUS_TIMER_OVF:
                 sbus_byte = 0;
         case SBUS_NEXT_BYTE:
                 if(SERVO_INPUT & 1) {
                  OCR0A = 25; /* 1+1/4 bit length before we sample the first data bit.
                                 This is approximately at the middle of bit
                                 due to interrupt latency and handler prologue */
                  TIFR0 = (1<<OCF0A)|(1<<TOV0);
                  TIMSK0 = (1<<OCIE0A)|(1<<TOIE0); 
                  SERVO_INT_MASK = 0;
                  sbus_bit_count = 0;
                  sbus_bit_parity = 0;
                  sbus_state = SBUS_DATA;
                  return;
                 }
                 sbus_state = SBUS_NONE;
                 DEBUG_ERROR(ERR_START);
         default: /* SBUS_FRAME_READY: not expecting pin change */
                 SERVO_INT_MASK = 0;
                 TIFR0 = (1<<TOV0);
                 TIMSK0 = (1<<TOIE0); 
                 servo_input_errors = 1; /* next frame came too early or glitch */
                 DEBUG_ERROR(ERR_PIN_CHANGE);
                 return;
 }
}


void sbus_frame_task(void)
{
    /* 
     * When sbus_frame_task becomes fast enough, we might move it to
     * interrupt handler (as the original ppm encoder code does).
     * Then USB events won't spoil RC signal.
     */
    #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
    static uint8_t led_delay = 0;
    #endif
#if 0
    uint8_t byte = 1;
    uint8_t bit = 7;
    uint8_t sdata;
#elif 0
    uint8_t byte = 1;
    uint8_t ch_bits_shift = 0;
#else
    volatile uint8_t *frame_pos = sbus_data+1;
    uint8_t ch_bits_shift = 8; /* force reload of sdata */
    uint8_t sdata;
    uint8_t tmp;
#endif
 
    uint8_t ppm_channel;
    uint8_t err;


    if(sbus_state != SBUS_FRAME_READY) return;
    if(sbus_data[0] != 0xf || sbus_data[24] != 0) {
     DEBUG_ERROR(ERR_FERR);
     PORTC &= ~(1<<PINC7); /* turn on */
     servo_input_errors = 1;
     sbus_state = SBUS_NONE;
     return;
    }
            

    wdt_reset(); 
    for(ppm_channel = 1; ppm_channel < SERVO_CHANNELS*2; ppm_channel+=2) {
            uint16_t servo_width;
#if 0
            uint16_t mask;
            servo_width = 0;
            for(mask = 1; !(mask&0x800); mask <<= 1) {
            // sbus bit order (wire):
            // 0 1 2 3 4 5 6 7 | 8 9 a 0 1 2 3 4 | 5 6 ....
            // \ ch1                 / \ ch 2 
            // we read MSB first, so LSB of first channel is MSB of the first data byte
            // FIXME: it's probably faster to shift servo_width, not mask
             if(bit == 7) {
              sdata = sbus_data[byte++];
              bit = 0;
             } else {
              sdata <<= 1;
              bit++;
             }
             if(sdata & 0x80) servo_width |= mask;
            }
#elif 0
            if(ch_bits_shift < 6) {
             servo_width = (sbus_data[byte] >> ch_bits_shift) | ((uint16_t)sbus_data[byte+1] << (8-ch_bits_shift));
             byte += 1;
             ch_bits_shift += 3;
            } else {
             servo_width = (sbus_data[byte] >> ch_bits_shift) | ((uint16_t)sbus_data[byte+1]<<(8-ch_bits_shift)) | (((uint16_t)sbus_data[byte+2] << 8) << (8-ch_bits_shift));
             byte += 2;
             ch_bits_shift -= 5;
            }

            servo_width &= 0x7FF;
#else


asm (
	"mov %A2, %1"	"\n\t"
	"ld %1, %a0+"	"\n\t"
	"mov %B2, %1"	"\n\t"
	"cpi %3, 8"	"\n\t"
	"brne 1f"	"\n\t"
	"mov %A2, %B2"	"\n\t"
	"ld %1, %a0+"	"\n\t"
	"mov %B2, %1"	"\n\t"
	"clr %3"	"\n\t"
	"rjmp 3f"	"\n\t"
	"1:	mov __tmp_reg__, %3"	"\n\t"
	"2:	lsr %B2"	"\n\t"
	"ror %A2"	"\n\t"
	"dec __tmp_reg__"	"\n\t"
	"brne 2b"	"\n\t"
	"3: andi %B2, 7"	"\n\t"
	"cpi %3, 6"	"\n\t"
	"brlo 5f"	"\n\t"
	"ld %1, %a0+"	"\n\t"
	"mov __tmp_reg__, %1"	"\n\t"
	"ldi %4, 8"	"\n\t"
	"sub %4, %3"	"\n\t"
	"1:	lsl __tmp_reg__"	"\n\t"
	"dec %4"	"\n\t"
	"brne 1b"	"\n\t"
	"or %B2, __tmp_reg__"	"\n\t"
	"andi %B2, 7"	"\n\t"
	"subi %3, 8"	"\n\t"
	"5:	subi %3, -3"
:"=e" (frame_pos), "=r" (sdata), "=r" (servo_width), "=a" (ch_bits_shift), "=a" (tmp)
:"0" (frame_pos), "1" (sdata), "3" (ch_bits_shift)
:"cc");
#endif

#define SBUS_CALIBRATION_OFFSET 5
            // (880 + servo_width*5/8)*2 - PPM_PRE_PULSE
            servo_width = 1761+SBUS_CALIBRATION_OFFSET-PPM_PRE_PULSE+servo_width+(servo_width >> 2);
            
            servo_input_connected[ ppm_channel ] = SERVO_INPUT_CONNECTED_VALUE;
            ppm_timeout[ ppm_channel ] = 0;

        #ifdef _THROTTLE_LOW_FAILSAFE_INDICATION
            // Check for forced throttle fail-safe
            if( throttle_failsafe_force ) {
                if( ppm_channel == 5 )
                {
                    // Force throttle fail-safe
                    servo_width = PPM_THROTTLE_FAILSAFE;
                }
            }
        #endif
    
        #ifdef _AVERAGE_FILTER_
            // Average filter to smooth input jitter
            servo_width += ppm[ ppm_channel ];
            servo_width >>= 1;
        #endif

        #ifdef _JITTER_FILTER_
            // 0.5us cut filter to remove input jitter
            int16_t ppm_tmp = ppm[ ppm_channel ] - servo_width;
            if( ppm_tmp <= _JITTER_FILTER_ && ppm_tmp >= -_JITTER_FILTER_ ) servo_width = ppm[ppm_channel];
        #endif

            cli();
            ppm[ ppm_channel ] = servo_width;
            sei();
        
    }
                  
    sbus_state = SBUS_NONE;
    

    if( !ppm_generator_active ) ppm_start();

    #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
   
    err = servo_input_errors; 
    if( ++led_delay == 0 ) {
     PIND |= ( 1<< PD4 );
     if(!err) PORTC |= 1<<PINC7; /* turn off */
     else servo_input_errors = 0;
    }
    if(err) PORTC &= ~(1<<PINC7); /* turn on */

    #endif    

}
// ------------------------------------------------------------------------------


// ------------------------------------------------------------------------------
// PPM OUTPUT - TIMER1 COMPARE INTERRUPT
// ------------------------------------------------------------------------------
// Current active ppm channel
volatile uint8_t ppm_out_channel = PPM_ARRAY_MAX - 1;
ISR( PPM_INT_VECTOR, ISR_NOBLOCK )  
{
    // ------------------------------------------------------------------------------
    // !! NESTED INTERRUPT !!
    // - ALL VARIABLES SHOULD BE GLOBAL VOLATILE 
    // - ACCESSING VARIABLES >8BIT MUST BE DONE ATOMIC USING CLI/SEI
    /* // ------------------------------------------------------------------------------    */

    // Update timing for next compare toggle with either current ppm input value, or fail-safe value if there is a channel timeout.
    if( ppm_timeout[ ppm_out_channel ] > PPM_TIMEOUT_VALUE )
    {
         // Channel 1-4?
        if( ppm_out_channel < 8 )
        {
            // Channel 1-4 - Use fail-safe value
            PPM_COMPARE += failsafe_ppm[ ppm_out_channel ];

        }
        else 
        {
            // Channel 5-8 - Use last known value
            PPM_COMPARE += ppm[ ppm_out_channel ];
        }
       
    #if defined _THROTTLE_LOW_RECOVERY_POSSIBLE && defined _THROTTLE_LOW_FAILSAFE_INDICATION
        // Count the channel that we have lost
        disconnected_channels++;
    #elif defined _THROTTLE_LOW_FAILSAFE_INDICATION
        throttle_failsafe_force = true; 
    #endif

    #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
        PORTC &= ~(1<<PINC7); /* turn on */
    #endif    
    }
    else
    {
        // Use latest ppm input value   
        PPM_COMPARE += ppm[ ppm_out_channel ];
        
        // Increment active channel timeout (reset to zero in input interrupt each time a valid signal is detected)
        if( servo_input_connected[ ppm_out_channel ] >= SERVO_INPUT_CONNECTED_VALUE )
        {
            ppm_timeout[ ppm_out_channel ]++;
        }
    }

    if( ++ppm_out_channel >= PPM_ARRAY_MAX ) 
    {
        ppm_out_channel = 0;

    #ifdef _THROTTLE_LOW_RECOVERY_POSSIBLE
        // Did we lose one or more active servo input channel? If so force throttle fail-safe (RTL)
        if( disconnected_channels > 0 )
        {
            throttle_failsafe_force = true;
            disconnected_channels = 0;
        }
        else
        {
            throttle_failsafe_force = false;
        }
    #endif

        #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
        // Blink TX LED when PPM generator has finished a pulse train
        PIND |= ( 1<< PD5 );
        #endif
    }
}
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
// PPM READ - INTERRUPT SAFE PPM SERVO CHANNEL READ
// ------------------------------------------------------------------------------
uint16_t ppm_read_channel( uint8_t channel )
{
    // Limit channel to valid value
    uint8_t _channel = channel;
    if( _channel == 0 ) _channel = 1;
    if( _channel > SERVO_CHANNELS ) _channel = SERVO_CHANNELS;

    // Calculate ppm[..] position
    uint8_t ppm_index = ( _channel << 1 ) + 1;
    
    // Read ppm[..] in a non blocking interrupt safe manner
    uint16_t ppm_tmp = ppm[ ppm_index ];
    while( ppm_tmp != ppm[ ppm_index ] ) ppm_tmp = ppm[ ppm_index ];

    // Return as normal servo value
    return ppm_tmp + PPM_PRE_PULSE;    
}
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
// PPM ENCODER INIT
// ------------------------------------------------------------------------------
void ppm_encoder_init( void )
{
    // ATmegaXXU2 only init code
    // ------------------------------------------------------------------------------    
    #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
        // ------------------------------------------------------------------------------    
        // Reset Source checkings
        // ------------------------------------------------------------------------------
        if (MCUSR & 1)    // Power-on Reset
        {
            MCUSR=0; // Clear MCU Status register
            // custom code here
        }
        else if (MCUSR & 2)    // External Reset
        {
           MCUSR=0; // Clear MCU Status register
           // custom code here
        }
        else if (MCUSR & 4)    // Brown-Out Reset
        {
           MCUSR=0; // Clear MCU Status register
           brownout_reset=true;
           // Make sure PPM generator will always start directly with throttle F/S value (900us) after a brown-out reset
           ppm[5] = failsafe_ppm[ 5 ]; 
        }
        else    // Watchdog Reset
        {
           MCUSR=0; // Clear MCU Status register
           // custom code here
        }

        // APM USB connection status UART MUX selector pin
        // ------------------------------------------------------------------------------
        USB_DDR |= (1 << USB_PIN); // Set USB pin to output
        
        DDRD |= (1<<PD4); // RX LED OUTPUT
        DDRD |= (1<<PD5); // TX LED OUTPUT
        
        PORTD |= (1<<PD4); // RX LED OFF
        PORTD |= (1<<PD5); // TX LED OFF
    #endif
     


    // SERVO/PPM INPUT PINS
    // ------------------------------------------------------------------------------
    // Set all servo input pins to inputs
#ifdef SBUS_DEBUG
    SERVO_DDR = 0x7E;
#else
    SERVO_DDR = 0;
#endif

    // Activate pullups on all input pins
    SERVO_PORT |= 0xFF;

#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
    // on 32U2 set PD0 to be an output, and clear the bit. This tells
    // the 2560 that USB is connected. The USB connection event fires
    // later to set the right value
    DDRD |= 1;
    if (usb_connected) {
        PORTD     &= ~1;
    } else {
        PORTD     |= 1;
    }
#endif


    TCCR0B = 1<<CS01;
    TIMSK0 = 1<<TOIE0;
        // Set TIMER1 8x prescaler
        TCCR1B = ( 1 << CS11 );


    // Set servo input interrupt pin mask to servo input channel 1
    SERVO_INT_MASK = 0x00;
    
    // Enable servo input interrupt
    PCICR |= (1 << SERVO_INT_ENABLE);

    // PPM OUTPUT
    // ------------------------------------------------------------------------------
    // PPM generator (PWM output timer/counter) is started either by pin change interrupt or by watchdog interrupt

    // Set PPM pin to output
    PPM_DDR |= (1 << PPM_OUTPUT_PIN);

    PPM_DDR |= 1<<PINC7; /* red led */
    PORTC |= 1<<PINC7; /* turn off */
    
    // ------------------------------------------------------------------------------
    // Enable watchdog interrupt mode
    // ------------------------------------------------------------------------------
    {
        // Disable watchdog
        wdt_disable();
         // Reset watchdog timer
        wdt_reset();
         // Start timed watchdog setup sequence
        WDTCSR |= (1<<WDCE) | (1<<WDE );
        // Set 250 ms watchdog timeout and enable interrupt
        WDTCSR = (1<<WDIE) | (1<<WDP2);
    }
}
// ------------------------------------------------------------------------------

#endif

