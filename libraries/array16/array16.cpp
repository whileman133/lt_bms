// array16.cpp
//
// Written by Wes Hileman on 7 June 2016
// U. of Colorado, Colorado Springs
//
// Functions used to perform variuous operations
// on 16-bit arrays. Used in the LTC6804_2_Stack
// library.

#include "array16.h"

void pgm_mask_array( uint16_t* const arr, const uint16_t* const mask, const uint8_t len )
{
	for( uint8_t i = 0; i < len; i++ )
	{
		arr[i] &= pgm_read_word( &mask[i] );
	} // end for
} // end pgm_mask_array

void copy_array( const uint16_t* const src, uint16_t* const dest, const uint8_t len )
{
	for( uint8_t i = 0; i < len; i++ )
	{
		dest[i] = src[i];
	} // end for
} // end copy_array

// Compares corresponding elements in the arrays whose starting addresses are
// supplied for equality. If any pair of elements are not equal, false is returned.
// If all elements are equal, true is returned. Requires the length of the arrays be
// specified.
bool arrays_equal( const uint16_t* const arr1, const uint16_t* const arr2, const uint8_t len )
{
  for( uint8_t i = 0; i < len; i++ )
  {
    if( arr1[i] != arr2[i] )
    {
      return false;
    } // end if
  } // end for

  return true;
} // end arrays_equal