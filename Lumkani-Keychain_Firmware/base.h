/* 
 * File:   base.h
 * Author: Lumkani Team
 * Created on 1 September 2018
 */

#ifndef _BASE_H_
#define _BASE_H_

#include <stdint.h>

#define mSTRINGIZE(x)                      #x                                   // Macro to convert argument to string literal.
#define mSTRINGIZE_VALUE_OF(x)             mSTRINGIZE(x)

#define ARRAY_LENGTH(x)     (sizeof(x) / sizeof((x)[0]))                        // Calculate the length of an array 

#define UINT32_MSB(X)       (((X)&0xFF000000)>>24)
#define UINT32_MMSB(X)      (((X)&0x00FF0000)>>16)
#define UINT32_MLSB(X)      (((X)&0x0000FF00)>>8)
#define UINT32_LSB(X)       (((X)&0x000000FF)>>0)

#define UINT24_MSB(X)       (((X)&0xFF0000)>>16)
#define UINT24_MID(X)       (((X)&0x00FF00)>>8)
#define UINT24_LSB(X)       (((X)&0x0000FF)>>0)

#define UINT16_MSB(X)       (((X)&0xFF00)>>8)
#define UINT16_LSB(X)       (((X)&0x00FF)>>0)


#define UINT32_CONCAT(W,X,Y,Z)  ((uint32_t)((((uint32_t)(W)) << 24) | (((uint32_t)(X)) << 16) | (((uint32_t)(Y)) << 8) | (Z)))

#define UINT24_CONCAT(X,Y,Z)    ((uint32_t)((((uint32_t)(X)) << 16) | (((uint32_t)(Y)) << 8) | (Z)))

#define UINT16_CONCAT(X,Y)      ((uint16_t)((((uint32_t)(X)) << 8) | (Y)))

#define mCEILING(value, maximum)         (((value) > (maximum)) ? (maximum) : (value))
#define mFLOOR(value, minimum)           (((value) < (minimum)) ? (minimum) : (value))
#define mBOUND(value, minimum, maximum)  (mCEILING(mFLOOR(value, minimum), (maximum)))

#define mADC_VALUE_TO_MILLIVOLTS(adc_value)    ((((uint32_t)(adc_value)) * ((uint32_t)(ADC_REFERENCE_VOLTAGE))) / ((uint32_t)(ADC_MAX_VALUE)))

typedef enum
{  
    LOW = 0, 
    HIGH = 1,
    FALSE = 0,
    TRUE = 1
}boolean_state_t;

typedef enum                                                                    
{
    STATUS_TIMEOUT = -2,
    STATUS_FAILED = -1,                                                         // All negative Status codes signify a failure
    STATUS_IN_PROGRESS = 0,                                                     // Zero Status code signifies the operation is in progress
    STATUS_SUCCESS = 1,                                                         // All positive codes signify a success
    
}status_code_t;

#define mFAILED(X)          (((X) < 0) ? TRUE:FALSE)                            // These macros apply the generalised logic of the status codes
#define mSUCCEEDED(X)       (((X) > 0) ? TRUE:FALSE)
#define mIN_PROGRESS(X)     (((X) == 0) ? TRUE:FALSE)

typedef const struct
{
    uint8_t (*hex_character_to_int) (uint8_t character);
    void (*uint8_to_hex_string) (uint8_t integer, uint8_t* hex_string);
    uint16_t (*insert_string) (const uint8_t* input_buffer, uint16_t input_buffer_length, uint8_t* output_buffer, const uint16_t insert_start_index, uint16_t output_buffer_length);
} string_utilities_interface_t;

extern string_utilities_interface_t string_utilities;

#endif
