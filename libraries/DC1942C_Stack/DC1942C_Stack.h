// DC1942C_Stack.h
//
// Written by Wesley Hileman on 2 July 2016
//
// Class that handles communication with stacks
// of DC1942C boards, i.e. breakout boards for
// LTC6804-2 ICs. Extends the LTC6804_2_Stack
// class.

#ifndef DC1942C_STACK_H
#define DC1942C_STACK_H

#include "Arduino.h"
#include "avr/pgmspace.h"
#include "LTC6804_2_Stack.h"

#define KELVIN_CELSIUS_DIFFERENCE 273.15

// Analog input pins (values correspond
// to the binary state of the three GPIO select lines
// that route an analog input through the multiplexer
// on a DC1942C board)
#define DC1942C_AI0 0
#define DC1942C_AI1 1
#define DC1942C_AI2 2
#define DC1942C_AI3 3
#define DC1942C_AI4 4
#define DC1942C_AI5 5
#define DC1942C_AI6 6
#define DC1942C_AI7 7

#define DC1942C_ANALOG_IN_AUX_CHAN ADC_AUX_CHANNEL_ALL
#define DC1942C_ANALOG_IN_VOLTAGE GPIO1_VOLTAGE
#define DC1942C_ANALOG_IN_REF_VOLTAGE REF2_VOLTAGE
#define DC1942C_ANALOG_IN_SERIES_RESISTANCE_OHMS 10000
#define DC1942C_NUM_SELECT_LINES 3

class DC1942C_Stack: public LTC6804_2_Stack
{
	public:
		DC1942C_Stack( uint16_t chipSel );
		~DC1942C_Stack( void );
		bool thermistor_read_all(
			const size_t icCount, const uint8_t* const addresses,
			const size_t pinCount, const uint8_t* const pins,
			const uint8_t adcOpt, const uint8_t adcMode,
			const uint16_t bParam, const float tAmbKelvin, const float* const rAmbOhms, 
			float* temperatures );
		bool analog_read_all( 
			const size_t icCount, const uint8_t* const addresses, 
			const size_t pinCount, const uint8_t* const pins, 
			const uint8_t adcOpt, const uint8_t adcMode, 
			uint16_t* pinAdc, uint16_t* secRefAdc );
		bool analog_read( 
			const int8_t addr, 
			const size_t pinCount, const uint8_t* const pins, 
			const uint8_t adcOpt, const uint8_t adcMode, 
			uint16_t* const pinVoltage, uint16_t* const secRefVoltage );
		bool analog_select( const int8_t addr, const uint8_t pin );
};

#endif