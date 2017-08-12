// CRC15.cpp
//
// Written by Wes Hileman on 4 June 2016
// U. of Colorado, Colorado Springs
//
// Functions used to compute 15-bit Packet Error Codes (PECs)
// as used by Linear Technology's LTC6804 battery-cell manager chips.

#include "CRC15.h"

// Computes a 15-bit cyclic redundancy check (CRC) code for the array
// of 16-bit values to which the supplied pointer points. The data in the
// array should be provided in Big Endian format (most-significant words first
// and most significant bits first within words).
uint16_t crc15( uint16_t const* data, const uint8_t len )
{
  uint16_t rem = 16, tblIndex;
  uint8_t dataIndex;

  for( dataIndex = 0; dataIndex < len; dataIndex++ )
  {
    // Most significant byte of data word
    tblIndex = ((rem >> 7) ^ (data[dataIndex] >> 8)) & 0xFF;
    rem = (rem << 8) ^ pgm_read_word(crc15Table + tblIndex);
    // Least-significant byte of data word
    tblIndex = ((rem >> 7) ^ (data[dataIndex] & 0xFF)) & 0xFF;
    rem = (rem << 8) ^ pgm_read_word(crc15Table + tblIndex);
  } // end for

  return rem << 1;
} // end crc15

// Computes a 15-bit CRC code for each group of 16-bit values in the array
// whose starting address is supplied. The elements of each group should be
// adjacent, and the size of each group uniform. The number of groups and the size 
// of the groups are supplied as arguments.
void crc15_group( uint16_t const* data, uint16_t* const pecs, const uint16_t grpSize, const uint8_t numGrps )
{
  for( uint8_t i = 0; i < numGrps; i++ )
  {
    pecs[i] = crc15( &data[i * grpSize], grpSize );
  } // end for
} // end crc15_group