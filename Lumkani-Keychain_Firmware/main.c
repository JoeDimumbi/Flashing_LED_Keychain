// <editor-fold defaultstate="collapsed" desc="Description">
/** @file main.c
 *
 * @brief Firmware for the Lumkani Keychain
 *
 * @par
 * Author: Lumkani Team
 * Started on Apr 5, 2018
 */

#define _SUPPRESS_PLIB_WARNING
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Includes">

#include <stdint.h>
#include <stdio.h>
#include "config.h"
#include "pinmap.h"
#include "hardware.h"
#include "display.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Global Variables">

// </editor-fold>

void main (void)
{
    oscillator.wait_for_startup_clock_ready();
    watchdog.initialize();
    display.initialize();
    while(1)
    {
        display.pattern_scroll_out(1);
        __delay_ms(100);
        display.pattern_scroll_in(1);
    }
        watchdog.clear();
}