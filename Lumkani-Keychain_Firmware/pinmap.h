/* 
 * File:   pinmap.h
 * Author: Lumkani Team
 * Created on 1 September 2018
 */

#ifndef PINMAP_H
#define	PINMAP_H

// <editor-fold defaultstate="collapsed" desc="Includes">

#include <stdint.h>
#include <xc.h>

// </editor-fold>

/* Base Addresses for Peripherals */
#define _PORTA_BASE_ADDRESS                      0x00C
#define _PORTB_BASE_ADDRESS                      0x00D
#define _PORTC_BASE_ADDRESS                      0x00E

// <editor-fold defaultstate="collapsed" desc="PPS Definitions">

#define INPUT_PPS_INTPPS         (0xE10)
#define INPUT_PPS_T0CKIPPS       (0xE11)
#define INPUT_PPS_T1CKIPPS       (0xE12)
#define INPUT_PPS_T1GPPS         (0xE13)
#define INPUT_PPS_T3CKIPPS       (0xE2C)
#define INPUT_PPS_T3GPPS         (0xE2D)
#define INPUT_PPS_T5CKIPPS       (0xE2E)
#define INPUT_PPS_T5GPPS         (0xE2F)
#define INPUT_PPS_CCP1PPS        (0xE14)
#define INPUT_PPS_CCP2PPS        (0xE15)
#define INPUT_PPS_CCP3PPS        (0xE16)
#define INPUT_PPS_CCP4PPS        (0xE17)
#define INPUT_PPS_CWG1PPS        (0xE18)
#define INPUT_PPS_CWG2PPS        (0xE19)
#define INPUT_PPS_MDCIN1PPS      (0xE1A)
#define INPUT_PPS_MDCIN2PPS      (0xE1B)
#define INPUT_PPS_MDMINPPS       (0xE1C)
#define INPUT_PPS_SSP1CLKPPS     (0xE20)
#define INPUT_PPS_SSP1DATPPS     (0xE21)
#define INPUT_PPS_SSP1SSPPS      (0xE22)
#define INPUT_PPS_SSP2CLKPPS     (0xE1D)
#define INPUT_PPS_SSP2DATPPS     (0xE1E)
#define INPUT_PPS_SSP2SSPPS      (0xE1F)
#define INPUT_PPS_RXPPS          (0xE24)
#define INPUT_PPS_TXPPS          (0xE25)
#define INPUT_PPS_CLCIN0PPS      (0xE28)
#define INPUT_PPS_CLCIN1PPS      (0xE29)
#define INPUT_PPS_CLCIN2PPS      (0xE2A)
#define INPUT_PPS_CLCIN3PPS      (0xE2B)

#define INPUT_PPS_A0 			 0b00000
#define INPUT_PPS_A1 			 0b00001
#define INPUT_PPS_A2 			 0b00010
#define INPUT_PPS_A3 			 0b00011
#define INPUT_PPS_A4 			 0b00100
#define INPUT_PPS_A5             0b00101
#define INPUT_PPS_B4             0b01100
#define INPUT_PPS_B5             0b01101
#define INPUT_PPS_B6             0b01110
#define INPUT_PPS_B7             0b01111
#define INPUT_PPS_C0 			 0b10000
#define INPUT_PPS_C1 			 0b10001
#define INPUT_PPS_C2 			 0b10010
#define INPUT_PPS_C3 			 0b10011
#define INPUT_PPS_C4 			 0b10100
#define INPUT_PPS_C5 			 0b10101
#define INPUT_PPS_C6 			 0b10110
#define INPUT_PPS_C7 			 0b10111

#define OUTPUT_PPS_A0  			 (0xE90)
#define OUTPUT_PPS_A1  			 (0xE91)
#define OUTPUT_PPS_A2  			 (0xE92)
#define OUTPUT_PPS_A3  			 0                                              // MCLR pin not included in PPS.
#define OUTPUT_PPS_A4  			 (0xE94)
#define OUTPUT_PPS_A5            (0xE95)
#define OUTPUT_PPS_B4            (0xE9C)
#define OUTPUT_PPS_B5            (0xE9D)
#define OUTPUT_PPS_B6            (0xE9E)
#define OUTPUT_PPS_B7            (0xE9F)
#define OUTPUT_PPS_C0  			 (0xEA0)
#define OUTPUT_PPS_C1  			 (0xEA1)
#define OUTPUT_PPS_C2  			 (0xEA2)
#define OUTPUT_PPS_C3  			 (0xEA3)
#define OUTPUT_PPS_C4  			 (0xEA4)
#define OUTPUT_PPS_C5  			 (0xEA5)
#define OUTPUT_PPS_C6 			 (0xEA6)
#define OUTPUT_PPS_C7 			 (0xEA7)

#define OUTPUT_PPS_DSM           0b11111
#define OUTPUT_PPS_CLKR          0b11110
#define OUTPUT_PPS_NCO1          0b11101
#define OUTPUT_PPS_TMR0          0b11100
#define OUTPUT_PPS_SDO2_SDA2     0b11011
#define OUTPUT_PPS_SCK2_SCL2     0b11010
#define OUTPUT_PPS_SDO1_SDA1     0b11001
#define OUTPUT_PPS_SCK1_SCL1     0b11000
#define OUTPUT_PPS_CMP2          0b10111
#define OUTPUT_PPS_CMP1          0b10110
#define OUTPUT_PPS_DT            0b10101
#define OUTPUT_PPS_TX_CK         0b10100
#define OUTPUT_PPS_CWG2D         0b10011
#define OUTPUT_PPS_CWG2C         0b10010
#define OUTPUT_PPS_CWG2B         0b10001
#define OUTPUT_PPS_CWG2A         0b10000
#define OUTPUT_PPS_CCP4          0b01111
#define OUTPUT_PPS_CCP3          0b01110
#define OUTPUT_PPS_CCP2          0b01101
#define OUTPUT_PPS_CCP1          0b01100
#define OUTPUT_PPS_CWG1D         0b01011
#define OUTPUT_PPS_CWG1C         0b01010
#define OUTPUT_PPS_CWG1B         0b01001
#define OUTPUT_PPS_CWG1A         0b01000
#define OUTPUT_PPS_CLC4OUT       0b00111
#define OUTPUT_PPS_CLC3OUT       0b00110
#define OUTPUT_PPS_CLC2OUT       0b00101
#define OUTPUT_PPS_CLC1OUT       0b00100
#define OUTPUT_PPS_PWM6          0b00011
#define OUTPUT_PPS_PWM5          0b00010

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ADC Channels">

typedef const enum
{
    ADCON0_CHS_FVR  = 0b111111,
    ADCON0_CHS_DAC1 = 0b111110,
    ADCON0_CHS_TEMP = 0b111101,
    ADCON0_CHS_VSS  = 0b111100,
    ADCON0_CHS_ANC7 = 0b010111,
    ADCON0_CHS_ANC6 = 0b010110,
    ADCON0_CHS_ANC5 = 0b010101,
    ADCON0_CHS_ANC4 = 0b010100,
    ADCON0_CHS_ANC3 = 0b010011,
    ADCON0_CHS_ANC2 = 0b010010,
    ADCON0_CHS_ANC1 = 0b010001,
    ADCON0_CHS_ANC0 = 0b010000,
    ADCON0_CHS_ANB7 = 0b001111,
    ADCON0_CHS_ANB6 = 0b001110,
    ADCON0_CHS_ANB5 = 0b001101,
    ADCON0_CHS_ANB4 = 0b001100,
    ADCON0_CHS_ANA5 = 0b000101,
    ADCON0_CHS_ANA4 = 0b000100,
    ADCON0_CHS_ANA2 = 0b000010,
    ADCON0_CHS_ANA1 = 0b000001,
    ADCON0_CHS_ANA0 = 0b000000
}adc_channel_t;

// </editor-fold>
            
typedef const struct
{
    volatile uint8_t* port;
    uint8_t pin_num;
    uint8_t input_pps;
    volatile uint8_t* output_pps;
    adc_channel_t adc_channel;
}pin_t;

#define mPIN_DEFINITION(nme, prt, bit)    pin_t nme = {.port = (volatile uint8_t*)_PORT##prt##_BASE_ADDRESS, .pin_num = bit, .input_pps = INPUT_PPS_##prt##bit, .output_pps = (volatile uint8_t*) OUTPUT_PPS_##prt##bit, .adc_channel = ADCON0_CHS_AN##prt##bit}
/*
 * TEMPLATE
 * 
 * Pin definitions:
 * 
    mPIN_DEFINITION(name, port [A,B,C], pin [0-15]);
 * 
 */

mPIN_DEFINITION(led1, A, 2);
mPIN_DEFINITION(led2, A, 2);
mPIN_DEFINITION(led3, A, 4);
mPIN_DEFINITION(led4, A, 4);
mPIN_DEFINITION(led5, A, 5);
mPIN_DEFINITION(led6, A, 5);
mPIN_DEFINITION(led7, C, 0);
mPIN_DEFINITION(led8, C, 1);
mPIN_DEFINITION(led9, C, 1);
mPIN_DEFINITION(led10, C, 1);
mPIN_DEFINITION(led11, C, 2);
mPIN_DEFINITION(led12, C, 2);
mPIN_DEFINITION(led13, C, 3);
mPIN_DEFINITION(led14, C, 3);
mPIN_DEFINITION(led15, C, 4);
mPIN_DEFINITION(led16, C, 4);
mPIN_DEFINITION(led17, C, 4);
mPIN_DEFINITION(led18, C, 5);

#endif
