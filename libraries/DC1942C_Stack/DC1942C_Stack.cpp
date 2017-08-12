// DC1942C_Stack.cpp
//
// Written by Wesley Hileman on 2 July 2016
//
// Class that handles communication with stacks
// of DC1942C boards, i.e. breakout boards for
// LTC6804-2 ICs. Extends the LTC6804_2_Stack
// class.

#include "DC1942C_Stack.h"

// GPIO lines connected to the analog multiplexer
// on the DC1942C '6804-2 demo board; the GPIOs are
// listed in order from least-significant select line
// to most significant select line
const uint8_t DC1942C_SELECT_LINES[] PROGMEM = {
	// LS Line ... MS Line
	GPIO2, GPIO3, GPIO4
};

// Constructor
DC1942C_Stack::DC1942C_Stack( uint16_t chipSel ) : LTC6804_2_Stack( chipSel )
{} // end constructor

// Destructor
DC1942C_Stack::~DC1942C_Stack( void )
{} // end destructor

bool DC1942C_Stack::thermistor_read_all(
	const size_t icCount, const uint8_t* const addresses,
	const size_t pinCount, const uint8_t* const pins,
	const uint8_t adcOpt, const uint8_t adcMode,
	const uint16_t bParam, const float tAmbKelvin, const float* const rAmbOhms,
	float* const temperatures )
{
	uint16_t thermAdc[icCount][pinCount];
	uint16_t ref2Adc[icCount][pinCount];
	size_t icIndex, pinIndex;
	float temperature;

	if( !analog_read_all( icCount, addresses, pinCount, pins, adcOpt, adcMode, thermAdc[0], ref2Adc[0] ) )
	{
		return false;
	} // end if

	for( icIndex = 0; icIndex < icCount; icIndex++ )
	{
		for( pinIndex = 0; pinIndex < pinCount; pinIndex++ )
		{
			temperature = (float)thermAdc[icIndex][pinIndex] / (float)(ref2Adc[icIndex][pinIndex] - thermAdc[icIndex][pinIndex]);
			temperature *= (float)DC1942C_ANALOG_IN_SERIES_RESISTANCE_OHMS / pgm_read_float( &rAmbOhms[icIndex * pinCount + pinIndex] );
			temperature = log( temperature ) / (float)bParam;
			temperature += 1.0 / tAmbKelvin;
			temperature = 1.0 / temperature;
			temperature -= KELVIN_CELSIUS_DIFFERENCE;

			temperatures[icIndex * pinCount + pinIndex] = temperature;
		} // end for
	} // end for

	return true;
} // end thermistor_read_all

bool DC1942C_Stack::analog_read_all( 
	const size_t icCount, const uint8_t* const addresses, 
	const size_t pinCount, const uint8_t* const pins, 
	const uint8_t adcOpt, const uint8_t adcMode, 
	uint16_t* pinAdc, uint16_t* secRefAdc )
{
	uint16_t auxData[LTC6804_NUM_AUX_WORDS];
	size_t icIndex, pinIndex;

	for( pinIndex = 0; pinIndex < pinCount; pinIndex++ )
	{
		// Perform analog-to-digital conversion of the voltage on the current
		// analog input pin for each '6804 IC
		for( icIndex = 0; icIndex < icCount; icIndex++ )
		{
			// Drive GPIOs to route the current analog-input pin through the current IC's
			// analog multiplexer to the GPIO1 port
			if( !analog_select( addresses[icIndex], pins[pinIndex] ) )
			{
				return false;
			} // end if

			// Start analog-to-digital conversion of both GPIO1 (the
			// port to which the analog input has been routed) and the
			// 2nd reference voltage, which drives devices connected to
			// the analog input pins
			start_auxillary_adc( addresses[icIndex], adcMode, DC1942C_ANALOG_IN_AUX_CHAN );
		} // end for

		// Wait for analog-to-digital conversions to finish on all target ICs
		adc_delay( AUXILLARY, adcOpt, adcMode, DC1942C_ANALOG_IN_AUX_CHAN );

		// Fetch and store the results of the analog-to-digital conversion 
		// for each IC
		for( icIndex = 0; icIndex < icCount; icIndex++ )
		{
			if( !get_raw_auxillary( addresses[icIndex], auxData ) )
			{
				// Failed to read auxillary data
				return false;
			} // end if

			pinAdc[icIndex * pinCount + pinIndex] = 		strip_data( auxData, DC1942C_ANALOG_IN_VOLTAGE );
			secRefAdc[icIndex * pinCount + pinIndex] = 	strip_data( auxData, DC1942C_ANALOG_IN_REF_VOLTAGE );
		} // end for
	} // end for

	return true;
} // end analog_read_all

// Performs analog-to-digital conversion of the voltages on the analog-input (AI) pins 
// specified in the array whose starting address is supplied and loads the results 
// (the digital values representing the voltage at the pin and the voltage of
// the 2nd reference at the time of the conversion) into the supplied arrays. An address
// argument specifies the DC1942C to target. This function does not support broadcast
// reads; use analog_read_all instead.
//
// NOTE: This function BLOCKS program execution by delaying for ADC completion.
// TODO create another, non-blocking function.
bool DC1942C_Stack::analog_read( const int8_t addr, const size_t pinCount, const uint8_t* const pins, const uint8_t adcOpt, const uint8_t adcMode, uint16_t* const pinVoltages, uint16_t* const secRefVoltages )
{
	uint16_t auxData[LTC6804_NUM_AUX_WORDS];
	size_t pinIndex;

	if( addr < 0 )
	{
		// Broadcast read not supported
		return false;
	} // end if

	for( pinIndex = 0; pinIndex < pinCount; pinIndex++ )
	{
		if( !analog_select( addr, pins[pinIndex] ) )
		{
			return false;
		} // end if

		start_auxillary_adc( addr, adcMode, DC1942C_ANALOG_IN_AUX_CHAN );
		adc_delay( AUXILLARY, adcOpt, adcMode, DC1942C_ANALOG_IN_AUX_CHAN );

		if( !get_raw_auxillary( addr, auxData ) )
		{
			// Failed to read auxillary data
			return false;
		} // end if

		pinVoltages[pinIndex] = 		strip_data( auxData, DC1942C_ANALOG_IN_VOLTAGE );
		secRefVoltages[pinIndex] = 	strip_data( auxData, DC1942C_ANALOG_IN_REF_VOLTAGE );
	} // end for

	return true;
} // end analog_read

bool DC1942C_Stack::analog_select( const int8_t addr, const uint8_t pin )
{
	uint8_t gpioStates = LTC6804_DEFAULT_GPIO_STATE;

	if( addr < 0 )
	{
		// Broadcast write not supported
		return false;
	} // end if

	for( size_t i = 0; i < DC1942C_NUM_SELECT_LINES; i++ )
	{
		bind_gpio_state( &gpioStates, pgm_read_byte( &DC1942C_SELECT_LINES[i] ), bitRead( pin, i ) ? HIGH : LOW );
	} // end for

	if( !digital_write( addr, gpioStates ) )
	{
		return false;
	} // end if

	return true;
} // end analog_select