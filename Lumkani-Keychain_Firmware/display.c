/*
 * File:   display.c
 * Author: Lumkani Team
 *
 * Created on 15 October 2018
 */

// <editor-fold defaultstate="collapsed" desc="Includes">

#include "display.h"
#include "hardware.h"
#include<stdio.h>
#include<stdlib.h>


// </editor-fold>

static boolean_state_t _gb_display_initialized = FALSE;

static void initialize (void);
static void pattern_scroll_out (uint8_t count);
static void pattern_scroll_in (uint8_t count);
static void led_on (led_t led);
static void led_off (led_t led);
static void led_flash (led_t led, uint16_t period);

static void _switch_led (led_t led, boolean_state_t state);

display_interface_t display = {
    .initialize = initialize,
    .pattern_scroll_out = pattern_scroll_out,
    .pattern_scroll_in = pattern_scroll_in,
    .led_on = led_on,
    .led_off = led_off,
    .led_flash = led_flash
};

static void initialize (void)
{
    if (!_gb_display_initialized)
    {

        gpio.mode(&led1, DIGITAL_OUTPUT_PUSH_PULL);
        gpio.mode(&led3, DIGITAL_OUTPUT_PUSH_PULL);
        gpio.mode(&led5, DIGITAL_OUTPUT_PUSH_PULL);
        gpio.mode(&led7, DIGITAL_OUTPUT_PUSH_PULL);
        
        gpio.mode(&led8, DIGITAL_OUTPUT_PUSH_PULL);
        gpio.mode(&led11, DIGITAL_OUTPUT_PUSH_PULL);
        
        gpio.mode(&led13, DIGITAL_OUTPUT_PUSH_PULL);
        gpio.mode(&led15, DIGITAL_OUTPUT_PUSH_PULL);
        
        gpio.mode(&led18, DIGITAL_OUTPUT_PUSH_PULL);
        
        _switch_led(RING1, LOW);
        _switch_led(RING2, LOW);
        _switch_led(RING3, LOW);
        _switch_led(RING4, LOW);

        _gb_display_initialized = TRUE;
    }
}

static void led_on (led_t led)
{
    boolean_state_t state = LOW;

    _switch_led(led, state);
}

static void led_off (led_t led)
{
    boolean_state_t state = HIGH;

    _switch_led(led, state);
}

static void led_flash (led_t led, uint16_t period)
{
    display.led_on(LED_ALL);
    
    _switch_led(led, HIGH);
        __delay_ms(1000);
    _switch_led(led, LOW);
}

static void _switch_led (led_t led, boolean_state_t state)
{
   
    switch (led)
    {
        case RING4:
            gpio.write(&led18, state);
            break;
        case RING3:
            gpio.write(&led15, state);
            gpio.write(&led13, state);
            break;
        case RING2:
            gpio.write(&led11, state);
            gpio.write(&led8, state);
            break;
        case RING1:
            gpio.write(&led7, state);
            gpio.write(&led5, state);
            gpio.write(&led3, state);
            gpio.write(&led1, state);
            break;
        case LED_ALL:
            gpio.write(&led1, state);
            gpio.write(&led3, state);
            gpio.write(&led5, state);
            gpio.write(&led7, state);
            gpio.write(&led8, state);
            gpio.write(&led11, state);
            gpio.write(&led13, state);
            gpio.write(&led15, state);
            gpio.write(&led18, state);
            break;
        default:
            break;
    }
}

static void pattern_scroll_out (uint8_t count)
{
    uint8_t i;
    
    display.led_off(LED_ALL);
    
    for (i = 0; i <= count; i++)
    {  
        _switch_led(RING4, LOW);
            __delay_ms(50);
        _switch_led(RING4, HIGH);       
    }
    for (i = 0; i <= count; i++)
    {    
        _switch_led(RING3, LOW);
            __delay_ms(50);
        _switch_led(RING3, HIGH);       
    }
    for (i = 0; i <= count; i++)
    {      
        _switch_led(RING2, LOW);
            __delay_ms(50);
        _switch_led(RING2, HIGH);       
    }
    for (i = 0; i <= count; i++)
    {
        _switch_led(RING1, LOW);
            __delay_ms(50);
        _switch_led(RING1, HIGH);       
    }
}

static void pattern_scroll_in (uint8_t count)
{
    uint8_t i;
    
    display.led_off(LED_ALL);
    
    for (i = 0; i <= count; i++)
    {     
        _switch_led(RING1, LOW);
            __delay_ms(50);
        _switch_led(RING1, HIGH);       
    }
    for (i = 0; i <= count; i++)
    {   
        _switch_led(RING2, LOW);
            __delay_ms(50);
        _switch_led(RING2, HIGH);       
    }
    for (i = 0; i <= count; i++)
    {   
        _switch_led(RING3, LOW);
            __delay_ms(50);
        _switch_led(RING3, HIGH);       
    }
    for (i = 0; i <= count; i++)
    {    
        _switch_led(RING4, LOW);
            __delay_ms(50);
        _switch_led(RING4, HIGH);       
    }
}