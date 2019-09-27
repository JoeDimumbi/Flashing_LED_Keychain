/* 
 * File:   hardware.h
 * Author: Lumkani Team
 * Created on 1 September 2018
 */

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

// <editor-fold defaultstate="collapsed" desc="System Configuration">

#define FREQ_LFINTOSC               31000L
#define FREQ_HFINTOSC32             32000000L
#define FREQ_HFINTOSC1              1000000L
#define _XTAL_FREQ                  FREQ_LFINTOSC                               // Define clock frequency in order to use built in __delay_ms() function

#define ADC_MAX_VALUE               1023L
#define ADC_REFERENCE_VOLTAGE       2048L


// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Includes">

#include <stdint.h>
#include "base.h"
#include "pinmap.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Oscillator">

typedef enum
{
    CLK_SOURCE_HFINTOSC_2PLL = 0b000,
    CLK_SOURCE_EXTOSC_4PLL = 0b001,
    CLK_SOURCE_SOSC = 0b011,
    CLK_SOURCE_LFINTOSC = 0b100,
    CLK_SOURCE_HFINTOSC = 0b110,
    CLK_SOURCE_EXTOSC = 0b111,
} clock_source_t;

typedef const enum
{
    CLK_DIV_1 = 0b0000,
    CLK_DIV_2 = 0b0001,
    CLK_DIV_4 = 0b0010,
    CLK_DIV_8 = 0b0011,
    CLK_DIV_16 = 0b0100,
    CLK_DIV_32 = 0b0101,
    CLK_DIV_64 = 0b0110,
    CLK_DIV_128 = 0b0111,
    CLK_DIV_256 = 0b1000,
    CLK_DIV_512 = 0b1001,
} clock_divider_t;

typedef struct
{
    uint32_t clock_speed;
    clock_source_t clock_source;
    clock_divider_t clock_divider;
} clock_mode_t;

clock_mode_t high_speed_uart_clock = { // TODO: Find a better place to have these modes
    .clock_speed = FREQ_HFINTOSC32, // Ensure this corresponds to clock_source/clock_divider
    .clock_source = CLK_SOURCE_HFINTOSC_2PLL,
    .clock_divider = CLK_DIV_1
};

clock_mode_t low_speed_clock = {
    .clock_speed = FREQ_LFINTOSC, // Ensure this corresponds to clock_source/clock_divider
    .clock_source = CLK_SOURCE_LFINTOSC,
    .clock_divider = CLK_DIV_1
};

typedef const struct
{
    clock_source_t (*get_clock_source) (void);
    void (*wait_for_startup_clock_ready) (void);
    boolean_state_t (*switch_clock) (clock_mode_t* clock);
    void (*set_clock_to_pin) (pin_t* pin);
} oscillator_interface_t;

extern oscillator_interface_t oscillator;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="GPIO">

typedef enum
{
    DIGITAL_OUTPUT_PUSH_PULL,
    DIGITAL_OUTPUT_OPEN_DRAIN,
    DIGITAL_INPUT_NO_PULL,
    DIGITAL_INPUT_PULL_UP,
    DIGITAL_INPUT_OPEN_DRAIN,
    ANALOG_INPUT
} gpio_mode_t;

typedef const struct
{
    void (*mode) (pin_t *pin, gpio_mode_t mode);
    void (*write) (pin_t *pin, boolean_state_t state);
    void (*set) (pin_t *pin);
    void (*reset) (pin_t *pin);
    void (*toggle) (pin_t *pin);
    uint8_t (*read) (pin_t *pin);
    void (*input_pps) (pin_t *pin, volatile uint32_t* reg_address);
    void (*output_pps) (pin_t *pin, uint8_t peripheral);
} gpio_interface_t;

extern gpio_interface_t gpio;

void pin_mode (pin_t *pin, gpio_mode_t mode);
void write_pin (pin_t *pin, boolean_state_t state);
void set_pin (pin_t *pin);
void reset_pin (pin_t *pin);
void toggle_pin (pin_t *pin);
boolean_state_t read_pin (pin_t *pin);
void input_pps (pin_t *pin, uint16_t reg_address);
void output_pps (pin_t *pin, uint8_t peripheral);

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Power Management">

typedef const struct
{
    void (*initialize) (void);
    void (*clear) (void);
    boolean_state_t (*did_overflow) (void);
} watchdog_interface_t;

extern watchdog_interface_t watchdog;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="UART">


// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ADC">

#define ADC_ACQUISITION_TIME_US     10                                          // Acquisition time is ~4.4uS @ 50deg

typedef const struct
{
    void (*initialize) (void);
    void (*enable) (void);
    void (*disable) (void);
    uint16_t (*measure_pin) (pin_t* pin);
    uint16_t (*measure_channel) (adc_channel_t channel);
} adc_interface_t;

extern adc_interface_t adc;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="TIMING">

#define mDELAY_MS(delay) __delay_ms(delay)
#define mDELAY_US(delay) __delay_us(delay)

typedef const struct
{
    void (*initialize_and_enable_interrupt_timer) (void);
    boolean_state_t (*wait_for_register_flag_us) (volatile uint8_t *reg_ptr, const uint8_t bit_mask, const uint8_t condition, const uint32_t wait_time);
    boolean_state_t (*wait_for_register_flag_ms) (volatile uint8_t *reg_ptr, const uint8_t bit_mask, const uint8_t condition, const uint32_t wait_time);
}system_time_interface_t;

//extern system_time_interface_t system_time;
// </editor-fold>

#endif
