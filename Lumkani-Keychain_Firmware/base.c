/*
 * File:   base.c
 * Author: Lumkani Team
 *
 * Created on 31 May 2019
 */

// <editor-fold defaultstate="collapsed" desc="Includes">

#include "base.h"

// </editor-fold>

static uint8_t hex_character_to_int (const uint8_t character);
static void uint8_to_hex_string (uint8_t integer, uint8_t* hex_string);
static uint16_t insert_string (const uint8_t *input_buffer, uint16_t input_buffer_length, uint8_t *output_buffer, const uint16_t insert_start_index, uint16_t output_buffer_length);

string_utilities_interface_t string_utilities = {
    .hex_character_to_int = hex_character_to_int,
    .uint8_to_hex_string = uint8_to_hex_string,
    .insert_string = insert_string,
};

static uint8_t hex_character_to_int (uint8_t character)
{
    uint8_t value = 0;

    if ((character >= '0') && (character <= '9'))
    {
        value = character - '0';
    }
    else if ((character >= 'A') && (character <= 'F'))
    {
        value = character - 'A' + 10;
    }
    else if ((character >= 'a') && (character <= 'f'))
    {
        value = character - 'a' + 10;
    }

    return value;
}

static void uint8_to_hex_string (uint8_t integer, uint8_t* hex_string)
{
    const static char *hex = "0123456789ABCDEF";
    hex_string[0] = hex[((integer & 0xF0) >> 4)];
    hex_string[1] = hex[(integer & 0x0F)];
}

static uint16_t insert_string (const uint8_t* input_buffer, uint16_t input_buffer_length, uint8_t* output_buffer, const uint16_t insert_start_index, uint16_t output_buffer_length)
{
    uint16_t loop_index;

    for (loop_index = 0; ((loop_index < input_buffer_length) && ((insert_start_index + loop_index) < output_buffer_length)); loop_index++)
    {
        output_buffer[insert_start_index + loop_index] = input_buffer[loop_index];
    }

    return insert_start_index + loop_index;
}