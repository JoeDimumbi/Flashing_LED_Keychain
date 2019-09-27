/*
 * File:   hardware.c
 * Author: Lumkani Team
 *
 * Created on 1 September 2018
 */

// <editor-fold defaultstate="collapsed" desc="Includes">

#include "hardware.h"
#include <xc.h>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Pragma">

// CONFIG1 register

#pragma config FEXTOSC  = OFF                                                   // External oscillator disabled.
#pragma config RSTOSC   = LFINT                                                 // Power-up default value for COSC (LFINT = 31kHz).
#pragma config CLKOUTEN = OFF                                                   // CLKOUT function disabled.
#pragma config CSWEN    = ON                                                   // Cannot change clock source in user software (Cannot be changed from RSTOSC).

// CONFIG2 register

#pragma config MCLRE    = ON                                                    // Pin is set to MCLR function.
#pragma config PWRTE    = OFF                                                   // Power-up timer disabled.
#pragma config WDTE     = ON                                                    // Watchdog enabled.
#pragma config BOREN    = OFF                                                   // Brown-out reset disabled.

// CONFIG3 register

#pragma config LVP      = OFF                                                   // Low voltage programming disabled.

// CONFIG4 register

#pragma config CP       = ON                                                    // Program memory code protection enabled.
#pragma config CPD      = ON                                                    // Data memory code protection enabled.

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Oscillator">

#define _CLOCK_SWITCH_TIMEOUT_COUNT 10

static clock_source_t get_clock_source (void);
static void wait_for_startup_clock_ready (void);
static boolean_state_t switch_clock (clock_mode_t* clock);
static void set_clock_to_pin (pin_t* pin);

oscillator_interface_t oscillator = {
    .get_clock_source = get_clock_source,
    .wait_for_startup_clock_ready = wait_for_startup_clock_ready,
    .switch_clock = switch_clock,
    .set_clock_to_pin = set_clock_to_pin
};

static clock_source_t get_clock_source (void)
{    
    return OSCCON2bits.COSC;
}

static void wait_for_startup_clock_ready (void)
{
    while (!OSCCON3bits.ORDY)
    {
        asm("NOP");
    }
}

static boolean_state_t switch_clock (clock_mode_t* clock)
{
    uint8_t timeout_count = 0;
    boolean_state_t b_result = FALSE;
    clock_source_t old_clock_source = oscillator.get_clock_source();
    
    OSCCON3bits.CSWHOLD = 1;                                                    // Keep using old clock until this bit is cleared.
    OSCCON1bits.NOSC = clock->clock_source;
    OSCCON1bits.NDIV = clock->clock_divider;
    
    while (!(OSCCON3bits.NOSCR) && (timeout_count < _CLOCK_SWITCH_TIMEOUT_COUNT))
    {
        mDELAY_US(1);
        timeout_count++;
    }
    
    OSCCON3bits.CSWHOLD = 0;                                                    // Allow clock switch.
    
    b_result = clock->clock_source != old_clock_source;
    
    return b_result;
}

static void set_clock_to_pin (pin_t* pin)
{
    pin_mode(pin, DIGITAL_OUTPUT_PUSH_PULL);
    output_pps(pin, OUTPUT_PPS_CLKR);
    
    PMD0bits.CLKRMD = 0;                                                        // Ensure reference clock output module is powered.
    CLKRCONbits.CLKRDC = 0b10;                                                  // Set duty cycle to 50%.
    CLKRCONbits.CLKRDIV = 0;                                                    // No clock divide.
    CLKRCONbits.CLKREN = 1;
    
    // Note: Ensure CLKOUTEN is set in Pragma.
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="GPIO">

#define _PORTxPORT_OFFSET                   0x000
#define _PORTxTRIS_OFFSET                   0x080                          
#define _PORTxLAT_OFFSET                    0x100
#define _PORTxANSEL_OFFSET                  0x180
#define _PORTxWPU_OFFSET                    0x200
#define _PORTxODCON_OFFSET                  0x280

gpio_interface_t gpio = {
    .mode = pin_mode,
    .write = write_pin,
    .set = set_pin,
    .reset = reset_pin,
    .toggle = toggle_pin,
    .output_pps = output_pps
};

void pin_mode (pin_t *pin, gpio_mode_t mode)
{
    volatile uint8_t* base_pointer = pin->port;
    uint8_t pin_bit_position = HIGH << pin->pin_num;

    if (mode == DIGITAL_INPUT_NO_PULL)
    {
        base_pointer[_PORTxANSEL_OFFSET] &= ~(pin_bit_position);
        base_pointer[_PORTxTRIS_OFFSET] |= (pin_bit_position);
        base_pointer[_PORTxWPU_OFFSET] &= ~(pin_bit_position);
        base_pointer[_PORTxODCON_OFFSET] &= ~(pin_bit_position);
    }
    else if (mode == DIGITAL_INPUT_PULL_UP)
    {
        base_pointer[_PORTxANSEL_OFFSET] &= ~(pin_bit_position);
        base_pointer[_PORTxTRIS_OFFSET] |= (pin_bit_position);
        base_pointer[_PORTxWPU_OFFSET] |= (pin_bit_position);
        base_pointer[_PORTxODCON_OFFSET] &= ~(pin_bit_position);
    }
    else if (mode == DIGITAL_INPUT_OPEN_DRAIN)
    {
        base_pointer[_PORTxANSEL_OFFSET] &= ~(pin_bit_position);
        base_pointer[_PORTxTRIS_OFFSET] |= (pin_bit_position);
        base_pointer[_PORTxWPU_OFFSET] &= ~(pin_bit_position);
        base_pointer[_PORTxODCON_OFFSET] |= (pin_bit_position);
    }
    else if (mode == ANALOG_INPUT)
    {
        base_pointer[_PORTxANSEL_OFFSET] |= (pin_bit_position);
        base_pointer[_PORTxTRIS_OFFSET] |= (pin_bit_position);
        base_pointer[_PORTxWPU_OFFSET] &= ~(pin_bit_position);
        base_pointer[_PORTxODCON_OFFSET] &= ~(pin_bit_position);
    }
    else if (mode == DIGITAL_OUTPUT_PUSH_PULL)
    {
        base_pointer[_PORTxANSEL_OFFSET] &= ~(pin_bit_position);
        base_pointer[_PORTxTRIS_OFFSET] &= ~(pin_bit_position);
        base_pointer[_PORTxWPU_OFFSET] &= ~(pin_bit_position);
        base_pointer[_PORTxODCON_OFFSET] &= ~(pin_bit_position);
    }
    else if (mode == DIGITAL_OUTPUT_OPEN_DRAIN)
    {
        base_pointer[_PORTxANSEL_OFFSET] &= ~(pin_bit_position);
        base_pointer[_PORTxTRIS_OFFSET] &= ~(pin_bit_position);
        base_pointer[_PORTxWPU_OFFSET] &= ~(pin_bit_position);
        base_pointer[_PORTxODCON_OFFSET] |= (pin_bit_position);
    }
}

void write_pin (pin_t *pin, boolean_state_t state)
{
    volatile uint8_t* base_pointer = pin->port;
    uint8_t pin_bit_position = HIGH << pin->pin_num;

    if (state == HIGH)
    {
        base_pointer[_PORTxLAT_OFFSET] |= (pin_bit_position);
    }
    else if (state == LOW)
    {
        base_pointer[_PORTxLAT_OFFSET] &= ~(pin_bit_position);
    }
}

void set_pin (pin_t *pin)
{
    volatile uint8_t* base_pointer = pin->port;
    uint8_t pin_bit_position = HIGH << pin->pin_num;

    base_pointer[_PORTxLAT_OFFSET] |= (pin_bit_position);
}

void reset_pin (pin_t *pin)
{
    volatile uint8_t* base_pointer = pin->port;
    uint8_t pin_bit_position = HIGH << pin->pin_num;

    base_pointer[_PORTxLAT_OFFSET] &= ~(pin_bit_position);
}

void toggle_pin (pin_t *pin)
{
    volatile uint8_t* base_pointer = pin->port;
    uint8_t pin_bit_position = HIGH << pin->pin_num;

    boolean_state_t current_value = read_pin(pin);

    if (current_value == HIGH)
    {
        base_pointer[_PORTxLAT_OFFSET] &= ~(pin_bit_position);
    }
    else
    {
        base_pointer[_PORTxLAT_OFFSET] |= (pin_bit_position);
    }

}

boolean_state_t read_pin (pin_t *pin)
{
    volatile uint8_t* base_pointer = pin->port;
    uint8_t pin_bit_position = HIGH << pin->pin_num;
    boolean_state_t return_data = LOW;

    return_data = ((base_pointer[_PORTxPORT_OFFSET] & (pin_bit_position)) == (pin_bit_position)) ? HIGH : LOW;

    return return_data;
}

void input_pps (pin_t *pin, uint16_t reg_address)
{
    *(volatile uint8_t*)reg_address = pin->input_pps;
}

void output_pps (pin_t *pin, uint8_t peripheral)
{
    *(pin->output_pps) = peripheral;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Power Management">

typedef const enum
{
    WDT_TIMEOUT_PERIOD_1_MS = 0b00000,
    WDT_TIMEOUT_PERIOD_2_MS = 0b00001,
    WDT_TIMEOUT_PERIOD_4_MS = 0b00010,
    WDT_TIMEOUT_PERIOD_8_MS = 0b00011,
    WDT_TIMEOUT_PERIOD_16_MS = 0b00100,
    WDT_TIMEOUT_PERIOD_32_MS = 0b00101,
    WDT_TIMEOUT_PERIOD_64_MS = 0b00110,
    WDT_TIMEOUT_PERIOD_128_MS = 0b00111,
    WDT_TIMEOUT_PERIOD_256_MS = 0b01000,
    WDT_TIMEOUT_PERIOD_512_MS = 0b01001,
    WDT_TIMEOUT_PERIOD_1_S = 0b01010,
    WDT_TIMEOUT_PERIOD_2_S = 0b01011,
    WDT_TIMEOUT_PERIOD_4_S = 0b01100,
    WDT_TIMEOUT_PERIOD_8_S = 0b01101,
    WDT_TIMEOUT_PERIOD_16_S = 0b01110,
    WDT_TIMEOUT_PERIOD_32_S = 0b01111,
    WDT_TIMEOUT_PERIOD_64_S = 0b10000,
    WDT_TIMEOUT_PERIOD_128_S = 0b10001,
    WDT_TIMEOUT_PERIOD_256_S = 0b10010
} watchdog_timeout_period_t;

static void watchdog_initialize (void);
static void watchdog_clear (void);
static boolean_state_t watchdog_did_overflow (void);

watchdog_interface_t watchdog = {
    .initialize = watchdog_initialize,
    .clear = watchdog_clear,
    .did_overflow = watchdog_did_overflow
};

static void watchdog_initialize (void)
{
    WDTCONbits.WDTPS = WDT_TIMEOUT_PERIOD_64_S;
    watchdog_clear();
}

static void watchdog_clear (void)
{
    asm("WDTCLR");
}

static boolean_state_t watchdog_did_overflow (void)
{
    boolean_state_t b_did_overflow = FALSE;

    if (STATUSbits.nTO == LOW)
    {
        b_did_overflow = TRUE;
    }

    return b_did_overflow;
}


// </editor-fold>


// <editor-fold defaultstate="collapsed" desc="ADC">

#define ADC_CONVERSION_TIMEOUT_MS        5                                      // Timeout delay for getting an ADC sample, in milliseconds.

typedef enum
{
    ADC_FVR_BUFFER_GAIN_OFF = 0b00, // 0.000V
    ADC_FVR_BUFFER_GAIN_1 = 0b01, // 1.024V
    ADC_FVR_BUFFER_GAIN_2 = 0b10, // 2.048V
    ADC_FVR_BUFFER_GAIN_4 = 0b11 // 4.096V
} adc_fvr_buffer_gain_t;

typedef enum
{
    ADC_AUTOCONVERSION_TRIGGER_TMR5 = 0b10001,
    ADC_AUTOCONVERSION_TRIGGER_TMR3 = 0b10000,
    ADC_AUTOCONVERSION_TRIGGER_OFF = 0b0000,
    ADC_AUTOCONVERSION_TRIGGER_TMR4 = 0b0001,
    ADC_AUTOCONVERSION_TRIGGER_TMR6 = 0b0010,
    ADC_AUTOCONVERSION_TRIGGER_TMR0 = 0b0011,
    ADC_AUTOCONVERSION_TRIGGER_TMR1 = 0b0100,
    ADC_AUTOCONVERSION_TRIGGER_TMR2 = 0b0101,
    ADC_AUTOCONVERSION_TRIGGER_COMPARITOR_1 = 0b0110,
    ADC_AUTOCONVERSION_TRIGGER_COMPARITOR_2 = 0b0111,
    ADC_AUTOCONVERSION_TRIGGER_CLC1 = 0b1000,
    ADC_AUTOCONVERSION_TRIGGER_CLC2 = 0b1001,
    ADC_AUTOCONVERSION_TRIGGER_CLC3 = 0b1010,
    ADC_AUTOCONVERSION_TRIGGER_CLC4 = 0b1011,
    ADC_AUTOCONVERSION_TRIGGER_CCP1 = 0b1100,
    ADC_AUTOCONVERSION_TRIGGER_CCP2 = 0b1101,
    ADC_AUTOCONVERSION_TRIGGER_CCP3 = 0b1110,
    ADC_AUTOCONVERSION_TRIGGER_CCP4 = 0b1111
} adc_autoconversion_trigger_t;

typedef enum
{
    ADC_POSITIVE_REFERENCE_VDD = 0b00,
    ADC_POSITIVE_REFERENCE_EXT_VREF = 0b10,
    ADC_POSITIVE_REFERENCE_FVR = 0b11
} adc_positive_reference_t;

typedef enum
{
    ADC_NEGATIVE_REFERENCE_AVSS = 0b0,
    ADC_NEGATIVE_REFERENCE_EXT_VREF = 0b1
} adc_negative_reference_t;

typedef enum
{
    ADC_RESULT_FORMAT_RIGHT_JUSTIFIED = 0b1,
    ADC_RESULT_FORMAT_LEFT_JUSTIFIED = 0b0
} adc_result_format_t;

typedef enum
{
    ADC_CONVERSION_CLOCK_FOSC_DIV_2 = 0b000,
    ADC_CONVERSION_CLOCK_FOSC_DIV_4 = 0b100,
    ADC_CONVERSION_CLOCK_FOSC_DIV_8 = 0b001,
    ADC_CONVERSION_CLOCK_FOSC_DIV_16 = 0b101,
    ADC_CONVERSION_CLOCK_FOSC_DIV_32 = 0b010,
    ADC_CONVERSION_CLOCK_FOSC_DIV_64 = 0b110,
    ADC_CONVERSION_CLOCK_ADCRC = 0b111,
} adc_conversion_clock_t;

static void adc_initialize (void);
static inline void adc_enable (void);
static inline void adc_disable (void);
static uint16_t adc_measure_pin (pin_t* pin);
static uint16_t adc_measure_channel (adc_channel_t channel);

static void _adc_pin_channel_select (pin_t* pin);
static void _adc_channel_select (adc_channel_t channel);
static uint16_t _adc_get_value (void);

adc_interface_t adc = {
    .initialize = adc_initialize,
    .enable = adc_enable,
    .disable = adc_disable,
    .measure_pin = adc_measure_pin,
    .measure_channel = adc_measure_channel
};

static void adc_initialize (void)
{
    FVRCONbits.FVREN = 1; // Enable the fixed voltage reference module
    FVRCONbits.ADFVR = ADC_FVR_BUFFER_GAIN_2; // Set ADC FVR buffer gain to 2x (Vref+ = 2.048V)

    ADACT = ADC_AUTOCONVERSION_TRIGGER_OFF; // Auto-conversion is disabled
    ADCON1bits.ADPREF = ADC_POSITIVE_REFERENCE_FVR; // FVR (2.048V)
    ADCON1bits.ADNREF = ADC_NEGATIVE_REFERENCE_AVSS; // AVSS (GND)
    ADCON1bits.ADFM = ADC_RESULT_FORMAT_RIGHT_JUSTIFIED; // Result is right justified
    ADCON1bits.ADCS = ADC_CONVERSION_CLOCK_FOSC_DIV_2; // Conversion clock is set to Fosc/2

    adc_disable();
}

static inline void adc_enable (void)
{
    ADCON0bits.ADON = 1; // Enable ADC peripheral.
}

static inline void adc_disable (void)
{
    ADCON0bits.ADON = 0; // Disable ADC peripheral.
}

static uint16_t adc_measure_pin (pin_t* pin)
{
    return adc_measure_channel(pin->adc_channel);
}

static uint16_t adc_measure_channel (adc_channel_t channel)
{
    uint16_t adc_result = 0;

    _adc_channel_select(channel);

    adc_enable();
    mDELAY_US(ADC_ACQUISITION_TIME_US); // Acquisition time
    adc_result = _adc_get_value();
    adc_disable();
    
    return adc_result;
}

static void _adc_channel_select (adc_channel_t channel)
{
    ADCON0bits.CHS = channel; // ADC Channel selection for internal sources.
}

static uint16_t _adc_get_value (void)
{
    ADCON0bits.GO_nDONE = 1; // Start ADC conversion.
    uint16_t timeout = 0;

    while ((ADCON0bits.GO_nDONE == 1) && (timeout < ADC_CONVERSION_TIMEOUT_MS)) // Wait for conversion to complete.
    {
        timeout++;
        mDELAY_MS(1);
    }

    return ADRES;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="TIMING">

typedef const enum
{
    TMR0_OPERATING_MODE_16_BIT = 0b1,
    TMR0_OPERATING_MODE_8_BIT = 0b0
} tmr0_operating_mode_t;

typedef const enum
{
    TMR0_POSTSCALER_1 = 0b0000,
    TMR0_POSTSCALER_2 = 0b0001,
    TMR0_POSTSCALER_3 = 0b0010,
    TMR0_POSTSCALER_4 = 0b0011,
    TMR0_POSTSCALER_5 = 0b0100,
    TMR0_POSTSCALER_6 = 0b0101,
    TMR0_POSTSCALER_7 = 0b0110,
    TMR0_POSTSCALER_8 = 0b0111,
    TMR0_POSTSCALER_9 = 0b1000,
    TMR0_POSTSCALER_10 = 0b1001,
    TMR0_POSTSCALER_11 = 0b1010,
    TMR0_POSTSCALER_12 = 0b1011,
    TMR0_POSTSCALER_13 = 0b1100,
    TMR0_POSTSCALER_14 = 0b1101,
    TMR0_POSTSCALER_15 = 0b1110,
    TMR0_POSTSCALER_16 = 0b1111
} tmr0_postscaler_t;

typedef const enum
{
    TMR0_CLOCKSOURCE_T0CLIPPS = 0b000,
    TMR0_CLOCKSOURCE_T0CLIPPS_INVERTED = 0b001,
    TMR0_CLOCKSOURCE_FOSC_DIV_4 = 0b010,
    TMR0_CLOCKSOURCE_HFINTOSC = 0b011,
    TMR0_CLOCKSOURCE_LFINTOSC = 0b100,
    TMR0_CLOCKSOURCE_SOSC = 0b110,
    TMR0_CLOCKSOURCE_CLC1 = 0b111
} tmr0_clocksource_t;

typedef const enum
{
    TMR0_SYNCRONIZATION_NONE = 0b1,
    TMR0_SYNCRONIZATION_FOSC_DIV_4 = 0b0,
} tmr0_syncronization_t;

typedef const enum
{
    TMR0_PRESCALER_1 = 0b0000,
    TMR0_PRESCALER_2 = 0b0001,
    TMR0_PRESCALER_4 = 0b0010,
    TMR0_PRESCALER_8 = 0b0011,
    TMR0_PRESCALER_16 = 0b0100,
    TMR0_PRESCALER_32 = 0b0101,
    TMR0_PRESCALER_64 = 0b0110,
    TMR0_PRESCALER_128 = 0b0111,
    TMR0_PRESCALER_256 = 0b1000,
    TMR0_PRESCALER_512 = 0b1001,
    TMR0_PRESCALER_1024 = 0b1010,
    TMR0_PRESCALER_2048 = 0b1011,
    TMR0_PRESCALER_4096 = 0b1100,
    TMR0_PRESCALER_8192 = 0b1101,
    TMR0_PRESCALER_16384 = 0b1110,
    TMR0_PRESCALER_32768 = 0b1111
} tmr0_prescaler_t;

typedef const enum
{
    TMR2_PRESCALER_1 = 0b00,
    TMR2_PRESCALER_4 = 0b01,
    TMR2_PRESCALER_16 = 0b10,
    TMR2_PRESCALER_64 = 0b11
} timer2_4_6_prescaler_t;

typedef const enum
{
    TMR2_POSTSCALER_1 = 0b0000,
    TMR2_POSTSCALER_2 = 0b0001,
    TMR2_POSTSCALER_3 = 0b0010,
    TMR2_POSTSCALER_4 = 0b0011,
    TMR2_POSTSCALER_5 = 0b0100,
    TMR2_POSTSCALER_6 = 0b0101,
    TMR2_POSTSCALER_7 = 0b0110,
    TMR2_POSTSCALER_8 = 0b0111,
    TMR2_POSTSCALER_9 = 0b1000,
    TMR2_POSTSCALER_10 = 0b1001,
    TMR2_POSTSCALER_11 = 0b1010,
    TMR2_POSTSCALER_12 = 0b1011,
    TMR2_POSTSCALER_13 = 0b1100,
    TMR2_POSTSCALER_14 = 0b1101,
    TMR2_POSTSCALER_15 = 0b1110,
    TMR2_POSTSCALER_16 = 0b1111
} timer2_4_6_postscaler_t;

#define _TMR0_PR_CALC_MS(interrupt_time_milliseconds, prescaler, postscaler, input_clock_speed)\
                                                                                ((((uint32_t)(interrupt_time_milliseconds)) * ((uint32_t)(input_clock_speed))) \
                                                                                / (((uint32_t)(prescaler)) * ((uint32_t)(postscaler)) * ((uint32_t)(1000L))))

#define _TMR0_PR_CALC_S(interrupt_time_seconds, prescaler, postscaler, input_clock_speed)\
                                                                                ((((uint32_t)(interrupt_time_seconds)) * ((uint32_t)(input_clock_speed))) \
                                                                                / (((uint32_t)(prescaler)) * ((uint32_t)(postscaler))))

#define _TMR2_PR_CALC(interrupt_time_milliseconds, prescaler, postscaler)        ((((uint32_t)(interrupt_time_milliseconds)) * ((uint32_t)(_XTAL_FREQ)))\
                                                                                / (((uint32_t)(4L)) * ((uint32_t)(prescaler)) * ((uint32_t)(postscaler)) * ((uint32_t)(1000L))))

#define _INTERRUPT_TIMER_PERIOD_S                   60                          // x seconds between the timer interrupts

static void initialize_and_enable_interrupt_timer (void);
static boolean_state_t wait_for_register_flag_us (volatile uint8_t *reg_ptr, const uint8_t bit_mask, const uint8_t condition, const uint32_t wait_time);
static boolean_state_t wait_for_register_flag_ms (volatile uint8_t *reg_ptr, const uint8_t bit_mask, const uint8_t condition, const uint32_t wait_time);

static void _enable_interrupt_timer (void);
static void _disable_interrupt_timer (void);
static void _initialize_and_enable_timer2 (void);
static void _enable_timer2 (void);
static void _disable_timer2 (void);

system_time_interface_t system_time = {
    .initialize_and_enable_interrupt_timer = initialize_and_enable_interrupt_timer,
    .wait_for_register_flag_us = wait_for_register_flag_us,
    .wait_for_register_flag_ms = wait_for_register_flag_ms
};

static void initialize_and_enable_interrupt_timer (void)
{
    _disable_interrupt_timer();

    T0CON0bits.T016BIT = TMR0_OPERATING_MODE_8_BIT; // Set to 8 bit mode.
    T0CON0bits.T0OUTPS = TMR0_POSTSCALER_16; // Postscaler set to 1:16.
    T0CON1bits.T0CS = TMR0_CLOCKSOURCE_LFINTOSC; // Clock source for timer 0 is LFINT (31000 kHz).
    T0CON1bits.T0ASYNC = TMR0_SYNCRONIZATION_NONE; // Timer 0 not synchronised to system clock.
    T0CON1bits.T0CKPS = TMR0_PRESCALER_512; // Prescaler set to 1:512.

    TMR0H = (uint8_t)_TMR0_PR_CALC_S(_INTERRUPT_TIMER_PERIOD_S, 512, 16, FREQ_LFINTOSC);

//    interrupts.initialize_and_enable_all();
    _enable_interrupt_timer();
}

static boolean_state_t wait_for_register_flag_us (volatile uint8_t *reg_ptr, const uint8_t bit_mask, const uint8_t condition, const uint32_t wait_time)
{
    uint32_t timeout = 0;
    boolean_state_t b_success = TRUE;

    while ((((*reg_ptr & bit_mask) == bit_mask) != condition) && (timeout < wait_time))
    {
        mDELAY_US(1);
        timeout++;
    }

    if (timeout >= wait_time)
    {
        b_success = 0;
    }

    return b_success;
}

static boolean_state_t wait_for_register_flag_ms (volatile uint8_t *reg_ptr, const uint8_t bit_mask, const uint8_t condition, const uint32_t wait_time)
{
    return wait_for_register_flag_us(reg_ptr, bit_mask, condition, wait_time * 1000);
}

static void _enable_interrupt_timer (void)
{
    T0CON0bits.T0EN = 1; // Enable timer 0.  
}

static void _disable_interrupt_timer (void)
{
    T0CON0bits.T0EN = 0; // Disable timer 0.
}

static void _initialize_and_enable_timer2 (void)
{
    _disable_timer2();

    T2CONbits.T2OUTPS = TMR2_POSTSCALER_1;
    T2CONbits.T2CKPS = TMR2_PRESCALER_64;
    TMR2 = 0; // Clear count value

    PR2 = _TMR2_PR_CALC(1000, 1, 64); // 1 second overflow time

//    interrupts.initialize_and_enable_all();
    _enable_timer2();
}

static void _enable_timer2 (void)
{
    T2CONbits.TMR2ON = 1;
}

static void _disable_timer2 (void)
{
    T2CONbits.TMR2ON = 0;
}

// </editor-fold>