// array16.h
//
// Written by Wes Hileman on 7 June 2016
// U. of Colorado, Colorado Springs
//
// Functions used to perform variuous operations
// on 16-bit arrays. Used in the LTC6804_2_Stack
// library.

#ifndef ARRAY16_H
#define ARRAY16_H

// For _t integer definitions.
#include "Arduino.h"
// For storing lookup tables in program memory. Note that special
// program-memory access functions such as pgm_read_word and pgm_read_byte
// must be used to access data stored in program memory (i.e. const globals
// with the PROGMEM modifier).
#include <avr/pgmspace.h>

void pgm_mask_array( uint16_t* const arr, const uint16_t* const mask, const uint8_t len );
void copy_array( const uint16_t* const src, uint16_t* const dest, const uint8_t len );
bool arrays_equal( const uint16_t* arr1, const uint16_t* arr2, const uint8_t len );

#endif