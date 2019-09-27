/* 
 * File:   display.h
 * Author: Lumkani Team
 * Created on 15 October 2018
 */

#ifndef _DISPLAY_H_
#define _DISPLAY_H_

// <editor-fold defaultstate="collapsed" desc="Includes">

#include <stdint.h>
#include "config.h"

// </editor-fold>

typedef const enum
{
    RING1 = 1,
    RING2,
    RING3,
    RING4,
    LED_ALL
} led_t;

typedef const struct
{
    void (*initialize) (void);
    void (*pattern_scroll_out) (uint8_t count);
    void (*pattern_scroll_in) (uint8_t count);
    void (*led_on) (led_t led);
    void (*led_off) (led_t led);
    void (*led_flash) (led_t led, uint16_t period);
} display_interface_t;

extern display_interface_t display;

#endif
