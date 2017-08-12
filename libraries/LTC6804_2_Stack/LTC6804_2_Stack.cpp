// LTC6804_2_Stack.cpp
//
// Written by Wes Hileman on 4 June 2016
// U. of Colorado, Colorado Springs
//
// Class to write, read, and parse data from a stack of Linear Technology's 
// addressed LTC6804-2 ICs connected on a multidrop isoSPI bus.
//
// Most of this class's digital-architecture dependent functionality can
// be altered by changing program-memory lookup tables; note that many
// preprocessor-defined indices are zero-based for use in lookup tables.
//
// Not implemented by this class:
// - Cell-voltage, auxillary, and status self-tests
// - I2C/SPI over GPIOs
// - Polling ADC status (blind cycle implemented instead)

#include "LTC6804_2_Stack.h"

// Constants
#define MAX_US_DELAY 16300
#define NUM_US_IN_MS 1000
#define LTC6804_CC_MASK 0x7FF
#define LTC6804_2_ADDR_BIT 0x8000
#define LTC6804_ISO_SPI_WAKEUP_TIME_US 10
#define LTC6804_CORE_WAKEUP_TIME_MS 1
#define LTC6804_MAX_SPI_CLK_FREQ_HZ 1000000
#define LTC6804_MAX_READ_ATTEMPTS 5
#define LTC6804_MAX_WRITE_ATTEMPTS 5
// Clock divider select for Timer/Counter1 (16-bit).
// A value of 4 corresponds to a division of 256 which
// gives an overflow period of about 1 second when the
// clock frequency is 16MHz.
#define KEEP_ALIVE_TIMER_CLK_DIV_SEL 4
// Address placement in command word
#define LTC6804_2_CC_ADDR_LSB 11
// Argument placement in command words
const uint8_t LTC6804_2_CC_ARG_LSBS[] PROGMEM = {
  7, 4, 0, 6, 5, 0, 0
};
// Argument values
const uint8_t LTC6804_2_CC_ARG_VALUES[][LTC6804_MAX_NUM_CC_ARG_VALUES] PROGMEM = {
	{1, 2, 3}, 							// ADC mode
	{0, 1},									// Permit discharge
	{0, 1, 2, 3, 4, 5, 6},	// ADC cell channel
	{0, 1},									// Open-wire pullup
	{1, 2},									// Self-test mode
	{0, 1, 2, 3, 4, 5},			// ADC aux channel
	{0, 1, 2, 3, 4}					// ADC status channel
};
// Data placement in register groups
// The maximum data-segment size is 8-bits
const uint8_t LTC6804_REG_DATA_POS[][REG_DATA_NUM_GEN_DATA_ITEMS + REG_DATA_MAX_NUM_SEGMENTS * REG_DATA_NUM_SEGMENT_DATA_ITEMS] PROGMEM = {
  // Num data segments | Data-segment size (bits) | Word number (0) | Segment LSB (0) | Word number (1) | Segment LSB (1) | Word number (2) | Segment LSB (2)  ... 
  { 1, 1, 0, 10 }, // REFON bit (config reg group)
  { 1, 1, 0, 8 }, // ADCOPT bit (config reg group)
  { 1, 4, 2, 4 }, // cell discharge timeout (config reg group)
  { 1, 5, 0, 11 }, // GPIO control byte (config reg group)
  { 3, 4, 2, 8, 2, 12, 2, 0 }, // cell discharge control word (config reg group)
  { 3, 4, 0, 0, 0, 4, 1, 8 }, // undervoltage threshold (config reg group)
  { 3, 4, 1, 12, 1, 0, 1, 4 }, // overvoltage threshold (config reg group)
  { 2, 8, 0, 8, 0, 0 }, // cell 1 voltage
  { 2, 8, 1, 8, 1, 0 }, // cell 2 voltage
  { 2, 8, 2, 8, 2, 0 }, // cell 3 voltage
  { 2, 8, 3, 8, 3, 0 }, // cell 4 voltage
  { 2, 8, 4, 8, 4, 0 }, // cell 5 voltage
  { 2, 8, 5, 8, 5, 0 }, // cell 6 voltage
  { 2, 8, 6, 8, 6, 0 }, // cell 7 voltage
  { 2, 8, 7, 8, 7, 0 }, // cell 8 voltage
  { 2, 8, 8, 8, 8, 0 }, // cell 9 voltage
  { 2, 8, 9, 8, 9, 0 }, // cell 10 voltage
  { 2, 8, 10, 8, 10, 0 }, // cell 11 voltage
  { 2, 8, 11, 8,11, 0 }, // cell 12 voltage
  { 2, 8, 0, 8, 0, 0 }, // gpio 1 voltage
  { 2, 8, 1, 8, 1, 0 }, // gpio 2 voltage
  { 2, 8, 2, 8, 2, 0 }, // gpio 3 voltage
  { 2, 8, 3, 8, 3, 0 }, // gpio 4 voltage
  { 2, 8, 4, 8, 4, 0 }, // gpio 5 voltage
  { 2, 8, 5, 8, 5, 0 }, // 2nd reference voltage
  { 2, 8, 0, 8, 0, 0 }, // sum of cells voltage
  { 2, 8, 1, 8, 1, 0 }, // internal temperature
  { 2, 8, 2, 8, 2, 0 }, // analog supply voltage
  { 2, 8, 3, 8, 3, 0 }, // digital supply voltage
  { 12, 1, 4, 8, 4, 10, 4, 12, 4, 14, 4, 0, 4, 2, 4, 4, 4, 6, 5, 8, 5, 10, 5, 12, 5, 14 }, // undervoltage flags
  { 12, 1, 4, 9, 4, 11, 4, 13, 4, 15, 4, 1, 4, 3, 4, 5, 4, 7, 5, 9, 5, 11, 5, 13, 5, 15 }, // overvoltage flags
  { 1, 1, 5, 1 }, // MUXFAIL bit
  { 1, 1, 5, 0 }  // thermal shutdown bit
}; 
const uint8_t LTC6804_REG_DATA_SEGMENT_MASKS[] PROGMEM = {
  B00000001, B00000011, B00000111, B00001111, B00011111, B00111111, B01111111, B11111111
};
// Mask for the configuration register group; zeros mark bits
// that take on different meanings during write and read operations;
// these bits should be ingnored when reading-back previously-written
// configuration to test validity
const uint16_t CONFIG_GROUP_READBACK_MASK[] PROGMEM = {
	0x05FF, 0xFFFF, 0xFF0F
};
// Analog-to-digital conversion time lookup table
const uint32_t LTC6804_ADC_TIMES_US[][LTC6804_NUM_ADC_OPTS][LTC6804_NUM_ADC_MODES][LTC6804_MAX_NUM_ADC_CHANNELS] PROGMEM = {
	// Cell-voltage conversions
	{
		// ADC OPT 0
		{
			// ADC mode fast (27kHz)
			{
				// ALL, C1&7, C2&8, C3&9, C4&10, C5&11, C6&12
				1100, 201, 201, 201, 201, 201, 201
			},
			// ADC mode normal (7kHz)
			{
				// ALL, C1&7, C2&8, C3&9, C4&10, C5&11, C6&12
				2300, 405, 405, 405, 405, 405, 405
			},
			// ADC mode filtered (26Hz)
			{
				// ALL, C1&7, C2&8, C3&9, C4&10, C5&11, C6&12
				201000, 34000, 34000, 34000, 34000, 34000, 34000
			}
		},
		// ADC OPT 1
		{
			// ADC mode fast (14kHz)
			{
				// ALL, C1&7, C2&8, C3&9, C4&10, C5&11, C6&12
				1300, 230, 230, 230, 230, 230, 230
			},
			// ADC mode normal (3kHz)
			{
				// ALL, C1&7, C2&8, C3&9, C4&10, C5&11, C6&12
				3000, 501, 501, 501, 501, 501, 501
			},
			// ADC mode filtered (2kHz)
			{
				// ALL, C1&7, C2&8, C3&9, C4&10, C5&11, C6&12
				4400, 754, 754, 754, 754, 754, 754
			}
		}
	},

	// Auxillary conversions
	{
		// ADC OPT 0
		{
			// ADC mode fast (27kHz)
			{
				// ALL, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, 2ndRef
				1100, 201, 201, 201, 201, 201, 201
			},
			// ADC mode normal (7kHz)
			{
				// ALL, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, 2ndRef
				2300, 405, 405, 405, 405, 405, 405
			},
			// ADC mode filtered (26Hz)
			{
				// ALL, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, 2ndRef
				201000, 34000, 34000, 34000, 34000, 34000, 34000
			}
		},
		// ADC OPT 1
		{
			// ADC mode fast (14kHz)
			{
				// ALL, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, 2ndRef
				1300, 230, 230, 230, 230, 230, 230
			},
			// ADC mode normal (3kHz)
			{
				// ALL, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, 2ndRef
				3000, 501, 501, 501, 501, 501, 501
			},
			// ADC mode filtered (2kHz)
			{
				// ALL, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, 2ndRef
				4400, 754, 754, 754, 754, 754, 754
			}
		}
	},

	// Status conversions
	{
		// ADC OPT 0
		{
			// ADC mode fast (27kHz)
			{
				// ALL, SOC, ITMP, VA, VD
				748, 201, 201, 201, 201
			},
			// ADC mode normal (7kHz)
			{
				// ALL, SOC, ITMP, VA, VD
				1600, 405, 405, 405, 405
			},
			// ADC mode filtered (26Hz)
			{
				// ALL, SOC, ITMP, VA, VD
				134000, 34000, 34000, 34000, 34000
			}
		},
		// ADC OPT 1
		{
			// ADC mode fast (14kHz)
			{
				// ALL, SOC, ITMP, VA, VD
				865, 230, 230, 230, 230
			},
			// ADC mode normal (3kHz)
			{
				// ALL, SOC, ITMP, VA, VD
				2000, 501, 501, 501, 501
			},
			// ADC mode filtered (2kHz)
			{
				// ALL, SOC, ITMP, VA, VD
				3000, 754, 754, 754, 754
			}
		}
	}
};
// Register-data value lookup tables (for
// data values that are assigned indexes)
const uint16_t LTC6804_REG_DATA_VALUES_REF[] PROGMEM = {1, 0};
const uint16_t LTC6804_REG_DATA_VALUES_ADC_OPT[] PROGMEM = {0, 1};
const uint16_t LTC6804_REG_DATA_VALUES_DISCHARGE_TIMEOUT[] PROGMEM = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
const uint16_t* const LTC6804_REG_DATA_VALUES[] PROGMEM = {
	LTC6804_REG_DATA_VALUES_REF,
	LTC6804_REG_DATA_VALUES_ADC_OPT,
	LTC6804_REG_DATA_VALUES_DISCHARGE_TIMEOUT
};
// Position of GPIO-state bits in gpio-state byte; indexed
// by GPIOx preprocessor constants
const uint8_t LTC6804_GPIO_MAP_POS[] PROGMEM = {
	0, 1, 2, 3, 4
};

// Static class variables
uint8_t LTC6804_2_Stack::_numStacks = 0;
uint16_t* LTC6804_2_Stack::_chipSels;
bool* LTC6804_2_Stack::_timerMasks;

// Interrupt service routine for overflow of the hardware timer
// used to periodically reset the '6804 chips' watchdog timers
// (see "keep-alive timer" functions)
ISR (TIMER1_OVF_vect)
{
	LTC6804_2_Stack::timer_overflow_isr();
  // Interrupt flag is cleared by harware
} // end timer1 overflow ISR

// Constructor
LTC6804_2_Stack::LTC6804_2_Stack( uint16_t chipSel )
{
	_chipSel = chipSel;
	_stackId = _numStacks;

	++_numStacks;
	_chipSels = (uint16_t*) realloc( _chipSels, _numStacks * sizeof(uint16_t) );
	_timerMasks = (bool*) realloc( _timerMasks, _numStacks * sizeof(bool) );

	_chipSels[_stackId] = _chipSel;
	_timerMasks[_stackId] = false;
} // end constructor

// Destructor
LTC6804_2_Stack::~LTC6804_2_Stack( void )
{
	--_numStacks;
	_chipSels = (uint16_t*) realloc( _chipSels, _numStacks * sizeof(uint16_t) );
	_timerMasks = (bool*) realloc( _timerMasks, _numStacks * sizeof(bool) );
} // end destructor

// Non-static initializer
void LTC6804_2_Stack::setup( void )
{
	pinMode( _chipSel, OUTPUT );
	digitalWrite( _chipSel, HIGH );
} // end setup

// Static initializer
void LTC6804_2_Stack::start( void )
{
	SPI.begin();
	setup_keep_alive_timer();
	start_keep_alive_timer();
} // end start

// Static de-initializer
void LTC6804_2_Stack::stop( void )
{
	stop_keep_alive_timer();
	SPI.end();
} // end stop

// Writes configuration data to all of the '6804 ICs targeted by the
// addresses in the supplied array and verifies the correct data was
// written to each IC by reading-back the configuration data. Transmits
// configuration data to each chip in addressed, rather than broadcast,
// format for simplicity and flexibility (any subset of chips in a stack 
// can be configured by this function).
bool LTC6804_2_Stack::config_all( const uint8_t icCount, const uint8_t* const icAddresses, const uint16_t* const cfgGrp )
{
	for( uint8_t icIndex = 0; icIndex < icCount; icIndex++ )
  {
    if( !config( icAddresses[icIndex], cfgGrp ) )
    {
      return false;
    } // end if
  } // end for

  return true;
} // end config_all

// Writes configuration data to the '6804 IC targeted by the supplied
// address and verifies the correct data was written by reading-back
// the configuration data. Broadcast configuration is not supported by
// this method; use config_all instead.
bool LTC6804_2_Stack::config( const int8_t addr, const uint16_t* const cfgGrp )
{
	uint16_t	readBackPec, cmpPec;
	uint16_t	readBackCfgGrp[LTC6804_NUM_CONFIG_WORDS],
						cfgGrpCopy[LTC6804_NUM_CONFIG_WORDS];
	uint16_t	writeAtt = 0, readAtt = 0;

	if( addr < 0 )
	{
		// Broadcast configuration not supported
		return false;
	} // end if

	copy_array( cfgGrp, cfgGrpCopy, LTC6804_NUM_CONFIG_WORDS );
	pgm_mask_array( cfgGrpCopy, CONFIG_GROUP_READBACK_MASK, LTC6804_NUM_CONFIG_WORDS );

	do
	{
		// Limit write attempts to prevent infinite loop
		if( writeAtt++ > LTC6804_MAX_WRITE_ATTEMPTS )
		{
			// Write failed, unable to verify chip was successfuly configured
			return false;
		} // end if

		// Write configuration data to target IC
		write_config( addr, cfgGrp );

		// Attempt to read configuration data back to verify the chip was
		// configured properly
		do
		{
			// Limit read-back attempts to prevent infinite loop
			if( readAtt++ > LTC6804_MAX_READ_ATTEMPTS )
			{
				// Read-back failed, unable to verify chip was successfuly configured
				return false;
			} // end if

			read_config( addr, readBackCfgGrp, &readBackPec );
			cmpPec = crc15( readBackCfgGrp, LTC6804_NUM_CONFIG_WORDS );
		} while( cmpPec != readBackPec );

		// Isolate appropriate parts of read-back data for comparison
		pgm_mask_array( readBackCfgGrp, CONFIG_GROUP_READBACK_MASK, LTC6804_NUM_CONFIG_WORDS );

	} while( !arrays_equal( cfgGrpCopy, readBackCfgGrp, LTC6804_NUM_CONFIG_WORDS ) );

	return true;
} // end config

bool LTC6804_2_Stack::verify_all_config( const uint8_t icCount, const uint8_t* const icAddresses, const uint16_t* const cfgGrp, bool* const flags )
{
	bool configOk = true;

	for( uint8_t icIndex = 0; icIndex < icCount; icIndex++ )
	{
		if( !( flags[icIndex] = verify_config( icAddresses[icIndex], cfgGrp ) ) )
		{
			configOk = false;
		} // end if
	} // end for

	return configOk;
} // end verify_all_config

bool LTC6804_2_Stack::verify_config( const int8_t addr, const uint16_t* const cfgGrp )
{
	uint16_t	readBackPec, cmpPec;
	uint16_t	readBackCfgGrp[LTC6804_NUM_CONFIG_WORDS],
						cfgGrpCopy[LTC6804_NUM_CONFIG_WORDS];
	uint16_t	readAtt = 0;

	if( addr < 0 )
	{
		// Broadcast configuration verification not supported
		return false;
	} // end if

	copy_array( cfgGrp, cfgGrpCopy, LTC6804_NUM_CONFIG_WORDS );
	pgm_mask_array( cfgGrpCopy, CONFIG_GROUP_READBACK_MASK, LTC6804_NUM_CONFIG_WORDS );

	// Attempt to read configuration data back
	do
	{
		// Limit read-back attempts to prevent infinite loop
		if( readAtt++ > LTC6804_MAX_READ_ATTEMPTS )
		{
			// Read-back failed
			return false;
		} // end if

		read_config( addr, readBackCfgGrp, &readBackPec );
		cmpPec = crc15( readBackCfgGrp, LTC6804_NUM_CONFIG_WORDS );
	} while( cmpPec != readBackPec );

	// Isolate appropriate parts of read-back data for comparison
	pgm_mask_array( readBackCfgGrp, CONFIG_GROUP_READBACK_MASK, LTC6804_NUM_CONFIG_WORDS );

	return arrays_equal( cfgGrpCopy, readBackCfgGrp, LTC6804_NUM_CONFIG_WORDS );
} // end verify_config

// Reads raw configuration words from the '6804 IC targeted by the supplied address into the
// array whose starting address is given. Broadcast reads are not supported 
// by this function. Returns true if the read was completed successfully, false otherwise. 
// Implements a data integrity check by verifying recieved PECs and requesting retransmission if 
// data is corrupted.
bool LTC6804_2_Stack::get_raw_config( const int8_t addr, uint16_t* const configData )
{
  uint16_t	configPecs[LTC6804_NUM_CV_REGS], 
  					configCmpPecs[LTC6804_NUM_CV_REGS];
  uint16_t	readAtt = 0;

  if( addr < 0 )
  {
  	// Broadcast read not supported
  	return false;
  } // end if

  do
  {
  	// Limit read attempts to prevent infinite loop
		if( readAtt++ > LTC6804_MAX_READ_ATTEMPTS )
		{
			// Read failed, unable to fetch cell voltages
			return false;
		} // end if

  	read_config( addr, configData, configPecs );
  	crc15_group( configData, configCmpPecs, LTC6804_NUM_CONFIG_WORDS_PER_REG, LTC6804_NUM_CONFIG_REGS );
	} while( !arrays_equal( configPecs, configCmpPecs, LTC6804_NUM_CONFIG_REGS ) );

  return true;
} // end get_raw_config

// Reads raw cell-voltage-register data from the '6804 IC targeted by the supplied address into the
// array whose starting address is given. Broadcast reads are not supported by this function. Implements 
// a data integrity check by verifying recieved PECs and requesting retransmission if data is corrupted.
bool LTC6804_2_Stack::get_raw_cell_voltages( const int8_t addr, uint16_t* const cvData )
{
  uint16_t	cvPecs[LTC6804_NUM_CV_REGS], 
  					cvCmpPecs[LTC6804_NUM_CV_REGS];
  uint16_t	readAtt = 0;

  if( addr < 0 )
  {
  	// Broadcast read not supported
  	return false;
  } // end if

  do
  {
  	// Limit read attempts to prevent infinite loop
		if( readAtt++ > LTC6804_MAX_READ_ATTEMPTS )
		{
			// Read failed, unable to fetch cell voltages
			return false;
		} // end if

  	read_cell_voltages( addr, cvData, cvPecs );
  	crc15_group( cvData, cvCmpPecs, LTC6804_NUM_CV_WORDS_PER_REG, LTC6804_NUM_CV_REGS );
	} while( !arrays_equal( cvPecs, cvCmpPecs, LTC6804_NUM_CV_REGS ) );

  return true;
} // end get_raw_cell_voltages

// Reads raw auxillary-register data from the '6804 IC targeted by the supplied address into the
// array whose starting address is given. Broadcast reads are not supported by this function. Implements 
// a data integrity check by verifying recieved PECs and requesting retransmission if data is corrupted.
bool LTC6804_2_Stack::get_raw_auxillary( const int8_t addr, uint16_t* const auxData )
{
  uint16_t	auxPecs[LTC6804_NUM_AUX_REGS], 
  					auxCmpPecs[LTC6804_NUM_AUX_REGS];
  uint16_t	readAtt = 0;

  if( addr < 0 )
  {
  	// Broadcast read not supported
  	return false;
  } // end if

  do
  {
  	// Limit read attempts to prevent infinite loop
		if( readAtt++ > LTC6804_MAX_READ_ATTEMPTS )
		{
			// Read failed, unable to fetch auxillary voltages
			return false;
		} // end if

  	read_auxillary( addr, auxData, auxPecs );
  	crc15_group( auxData, auxCmpPecs, LTC6804_NUM_AUX_WORDS_PER_REG, LTC6804_NUM_AUX_REGS );
	} while( !arrays_equal( auxPecs, auxCmpPecs, LTC6804_NUM_AUX_REGS ) );

	// Read succedded
  return true;
} // end get_raw_auxillary

// Reads raw status-register data from the '6804 IC targeted by the supplied address into the
// array whose starting address is given. Broadcast reads are not supported by this function. Implements 
// a data integrity check by verifying recieved PECs and requesting retransmission if data is corrupted.
bool LTC6804_2_Stack::get_raw_status( const int8_t addr, uint16_t* const statData )
{
  uint16_t	statPecs[LTC6804_NUM_STAT_REGS], 
  					statCmpPecs[LTC6804_NUM_STAT_REGS];
  uint16_t	readAtt = 0;

  if( addr < 0 )
  {
  	// Broadcast read not supported
  	return false;
  } // end if

  do
  {
  	// Limit read attempts to prevent infinite loop
		if( readAtt++ > LTC6804_MAX_READ_ATTEMPTS )
		{
			// Read failed, unable to fetch auxillary voltages
			return false;
		} // end if

  	read_status( addr, statData, statPecs );
  	crc15_group( statData, statCmpPecs, LTC6804_NUM_STAT_WORDS_PER_REG, LTC6804_NUM_STAT_REGS );
	} while( !arrays_equal( statPecs, statCmpPecs, LTC6804_NUM_STAT_REGS ) );

  return true;
} // end get_raw_status

// Generates configuration words for a '6804 IC based on the supplied
// data and stores the words in the array whose starting address is
// specified. This function expects that the array's elements are initialized
// to zero.
void LTC6804_2_Stack::prepare_config( uint16_t* const cfgGrp, const uint8_t gpioPulldownCtl, const uint8_t ref, const uint8_t adcOpt, const uint16_t uvThreshold, const uint16_t ovThreshold, const uint16_t cellDischargeCtl, const uint8_t dischargeTimeout )
{
	bind_data( cfgGrp, CONFIG_GPIO_PULLDOWN_CTL, ~((const uint16_t)gpioPulldownCtl) );
  bind_data( cfgGrp, CONFIG_REF, (const uint16_t)ref );
  bind_data( cfgGrp, CONFIG_ADC_OPT, (const uint16_t)adcOpt );
  bind_data( cfgGrp, CONFIG_UV_THRESH, uvThreshold );
  bind_data( cfgGrp, CONFIG_OV_THRESH, ovThreshold );
  bind_data( cfgGrp, CONFIG_CELL_DISCHARGE_CTL, (const uint16_t)cellDischargeCtl );
  bind_data( cfgGrp, CONFIG_DISCHARGE_TIMEOUT, dischargeTimeout );
} // end prepare_config

// Extracts cell-voltages from an array of cell-voltage register-group data,
// whose starting address is specified, into the floating-point array whose
// starting address is also specified.
void LTC6804_2_Stack::extract_cell_voltages( const uint16_t* const regData, float* const cvData )
{
  cvData[0]   = CELL_VOLTAGE_CALC( strip_data( regData, CELL1_VOLTAGE ) );
  cvData[1]   = CELL_VOLTAGE_CALC( strip_data( regData, CELL2_VOLTAGE ) );
  cvData[2]   = CELL_VOLTAGE_CALC( strip_data( regData, CELL3_VOLTAGE ) );
  cvData[3]   = CELL_VOLTAGE_CALC( strip_data( regData, CELL4_VOLTAGE ) );
  cvData[4]   = CELL_VOLTAGE_CALC( strip_data( regData, CELL5_VOLTAGE ) );
  cvData[5]   = CELL_VOLTAGE_CALC( strip_data( regData, CELL6_VOLTAGE ) );
  cvData[6]   = CELL_VOLTAGE_CALC( strip_data( regData, CELL7_VOLTAGE ) );
  cvData[7]   = CELL_VOLTAGE_CALC( strip_data( regData, CELL8_VOLTAGE ) );
  cvData[8]   = CELL_VOLTAGE_CALC( strip_data( regData, CELL9_VOLTAGE ) );
  cvData[9]   = CELL_VOLTAGE_CALC( strip_data( regData, CELL10_VOLTAGE ) );
  cvData[10]  = CELL_VOLTAGE_CALC( strip_data( regData, CELL11_VOLTAGE ) );
  cvData[11]  = CELL_VOLTAGE_CALC( strip_data( regData, CELL12_VOLTAGE ) );
} // end extract_cell_voltages_float

// Extracts fixed-point cell-voltages from an array of cell-voltage register-group data,
// whose starting address is specified, into the integer array whose starting address is 
// also specified.
void LTC6804_2_Stack::extract_cell_voltages( const uint16_t* const regData, uint16_t* const cvData )
{
  cvData[0]   = strip_data( regData, CELL1_VOLTAGE );
  cvData[1]   = strip_data( regData, CELL2_VOLTAGE );
  cvData[2]   = strip_data( regData, CELL3_VOLTAGE );
  cvData[3]   = strip_data( regData, CELL4_VOLTAGE );
  cvData[4]   = strip_data( regData, CELL5_VOLTAGE );
  cvData[5]   = strip_data( regData, CELL6_VOLTAGE );
  cvData[6]   = strip_data( regData, CELL7_VOLTAGE );
  cvData[7]   = strip_data( regData, CELL8_VOLTAGE );
  cvData[8]   = strip_data( regData, CELL9_VOLTAGE );
  cvData[9]   = strip_data( regData, CELL10_VOLTAGE );
  cvData[10]  = strip_data( regData, CELL11_VOLTAGE );
  cvData[11]  = strip_data( regData, CELL12_VOLTAGE );
} // end extract_cell_voltages

// Extracts GPIO voltages and the 2nd reference voltage from an array of 
// auxillary register-group data, whose starting address is specified, into (1) 
// the floating-point array whose starting address is given (GPIO voltages) and
// (2) the float whose address is given (2nd reference voltage). 
void LTC6804_2_Stack::extract_auxillary( const uint16_t* const regData, float* const gpioVoltages, float* const ref2Voltage )
{
  gpioVoltages[0] = GPIO_VOLTAGE_CALC( strip_data( regData, GPIO1_VOLTAGE ) );
  gpioVoltages[1] = GPIO_VOLTAGE_CALC( strip_data( regData, GPIO2_VOLTAGE ) );
  gpioVoltages[2] = GPIO_VOLTAGE_CALC( strip_data( regData, GPIO3_VOLTAGE ) );
  gpioVoltages[3] = GPIO_VOLTAGE_CALC( strip_data( regData, GPIO4_VOLTAGE ) );
  gpioVoltages[4] = GPIO_VOLTAGE_CALC( strip_data( regData, GPIO5_VOLTAGE ) );
  *ref2Voltage    = REF2_VOLTAGE_CALC( strip_data( regData, REF2_VOLTAGE ) );
} // end extract_auxillary

// Extracts fixed-point GPIO voltages and the 2nd reference voltage from an array of 
// auxillary register-group data, whose starting address is specified, into (1) 
// the integer array whose starting address is given (GPIO voltages) and
// (2) the integer whose address is given (2nd reference voltage). 
void LTC6804_2_Stack::extract_auxillary( const uint16_t* const regData, uint16_t* const gpioVoltages, uint16_t* const ref2Voltage )
{
  gpioVoltages[0] = strip_data( regData, GPIO1_VOLTAGE );
  gpioVoltages[1] = strip_data( regData, GPIO2_VOLTAGE );
  gpioVoltages[2] = strip_data( regData, GPIO3_VOLTAGE );
  gpioVoltages[3] = strip_data( regData, GPIO4_VOLTAGE );
  gpioVoltages[4] = strip_data( regData, GPIO5_VOLTAGE );
  *ref2Voltage    = strip_data( regData, REF2_VOLTAGE );
} // end extract_auxillary

// Extracts status data from an array of status register-group data, whose
// starting address is specified, into the floating-point and unsigned integer
// varaibles whose memory addresses are specified.
void LTC6804_2_Stack::extract_status( const uint16_t* const regData, float* const socVoltage, float* const internalTemp, float* const analogSupplyVoltage, float* const digitalSupplyVoltage, uint16_t* const uvFlags, uint16_t* const ovFlags, uint8_t* const muxFail, uint8_t* const thermalShutdown )
{
  *socVoltage = SUM_OF_CELLS_VOLTAGE_CALC( strip_data( regData, SUM_OF_CELLS_VOLTAGE ) );
  *internalTemp = INTERNAL_TEMPERATURE_CALC( strip_data( regData, INTERNAL_TEMPERATURE ) );
  *analogSupplyVoltage = ANALOG_SUPPLY_VOLTAGE_CALC( strip_data( regData, ANALOG_SUPPLY_VOLTAGE ) );
  *digitalSupplyVoltage = DIGITAL_SUPPLY_VOLTAGE_CALC( strip_data( regData, DIGITAL_SUPPLY_VOLTAGE ) );
  *uvFlags = strip_data( regData, UNDERVOLTAGE_FLAGS );
  *ovFlags = strip_data( regData, OVERVOLTAGE_FLAGS );
  *muxFail = strip_data( regData, MUX_FAIL );
  *thermalShutdown = strip_data( regData, THERMAL_SHUTDOWN );
} // end extract_status

// Extracts fixed-point status data from an array of status register-group data, whose
// starting address is specified, into the floating-point and unsigned integer
// varaibles whose memory addresses are specified.
void LTC6804_2_Stack::extract_status( const uint16_t* const regData, uint16_t* const socVoltage, uint16_t* const internalTemp, uint16_t* const analogSupplyVoltage, uint16_t* const digitalSupplyVoltage, uint16_t* const uvFlags, uint16_t* const ovFlags, uint8_t* const muxFail, uint8_t* const thermalShutdown )
{
  *socVoltage = strip_data( regData, SUM_OF_CELLS_VOLTAGE );
  *internalTemp = strip_data( regData, INTERNAL_TEMPERATURE );
  *analogSupplyVoltage = strip_data( regData, ANALOG_SUPPLY_VOLTAGE );
  *digitalSupplyVoltage = strip_data( regData, DIGITAL_SUPPLY_VOLTAGE );
  *uvFlags = strip_data( regData, UNDERVOLTAGE_FLAGS );
  *ovFlags = strip_data( regData, OVERVOLTAGE_FLAGS );
  *muxFail = strip_data( regData, MUX_FAIL );
  *thermalShutdown = strip_data( regData, THERMAL_SHUTDOWN );
} // end extract_status

// Clears the status registers on the '6804 IC targeted by the
// specified address. Choose addr as BROADCAST or -1 to broadcast the
// clear command to all ICs.
void LTC6804_2_Stack::clear_status( const int8_t addr )
{
  uint16_t cmd = prepare_command( CLRSTAT );
  uint16_t cmdPec;

  if( addr >= 0 )
  {
    bind_address( &cmd, addr );
  } // end if

  cmdPec = crc15( &cmd, 1 );
  write_cmd( cmd, cmdPec );
} // end clear_status

// Sends a command starting cell-voltage analog-to-digital conversion (ADCV) on the
// '6804 IC whose address is supplied. Specify addr as BROADCAST or -1 to broadcast the 
// command to all ICs. Also accepts adc mode, discharge permit, and cell select arguments
// to bind to the ADCV command.
void LTC6804_2_Stack::start_cell_voltage_adc( const int8_t addr, const uint8_t mode, const uint8_t dischargePermit, const uint8_t cellChannel )
{
  static uint16_t cmd = prepare_command( ADCV );
  uint16_t cmdPec;

  if( addr >= 0 )
  {
    bind_address( &cmd, addr );
  } // end if

  bind_argument( &cmd, ADC_MODE, mode );
  bind_argument( &cmd, ADC_PERMIT_DISCHARGE, dischargePermit );
  bind_argument( &cmd, ADC_CELL_CHANNEL, cellChannel );
  cmdPec = crc15( &cmd, 1 );
  write_cmd( cmd, cmdPec );
} // end start_cell_voltage_adc

// Sends a command starting auxillary-voltage analog-to-digital conversion (ADAX) on the
// '6804 IC whose address is supplied. Specify addr as BROADCAST or -1 to broadcast the 
// command to all ICs. Also accepts adc mode and auxillary channel-select arguments to bind to 
// the ADAX command.
void LTC6804_2_Stack::start_auxillary_adc( const int8_t addr, const uint8_t mode, const uint8_t auxChannel )
{
  uint16_t cmd = prepare_command( ADAX );
  uint16_t cmdPec;

  if( addr >= 0 )
  {
    bind_address( &cmd, addr );
  } // end if

  bind_argument( &cmd, ADC_MODE, mode );
  bind_argument( &cmd, ADC_AUX_CHANNEL, auxChannel );
  cmdPec = crc15( &cmd, 1 );
  write_cmd( cmd, cmdPec );
} // end start_auxillary_adc

// Sends a command starting status-voltage analog-to-digital conversion (ADAX) on the
// '6804 IC whose address is supplied. Specify addr as BROADCAST or -1 to broadcast the 
// command to all ICs. Also accepts adc mode and self-test channel-select arguments to bind to 
// the ADAX command.
void LTC6804_2_Stack::start_status_adc( const int8_t addr, const uint8_t mode, const uint8_t statusChannel )
{
  uint16_t cmd = prepare_command( ADSTAT );
  uint16_t cmdPec;

  if( addr >= 0 )
  {
    bind_address( &cmd, addr );
  } // end if

  bind_argument( &cmd, ADC_MODE, mode );
  bind_argument( &cmd, ADC_STATUS_CHANNEL, statusChannel );
  cmdPec = crc15( &cmd, 1 );
  write_cmd( cmd, cmdPec );
} // end start_status_adc

// Sends a command starting multiplexer diagnostics (DIAGN) on the
// '6804 IC whose address is supplied. Specify addr as BROADCAST or -1 to broadcast the 
// command to all ICs.
void LTC6804_2_Stack::start_diagn( const int8_t addr )
{
  uint16_t cmd = prepare_command( DIAGN );
  uint16_t cmdPec;

  if( addr >= 0 )
  {
    bind_address( &cmd, addr );
  } // end if

  cmdPec = crc15( &cmd, 1 );
  write_cmd( cmd, cmdPec );
} // end start_status_adc

// Writes the specified data to the configuration registers of the IC whose 
// address is supplied. Specify addr as BROADCAST or -1 to broadcast the write 
// to all ICs.
void LTC6804_2_Stack::write_config( const int8_t addr, const uint16_t* const regDataPtr )
{
  uint16_t writeCmd = prepare_command( WRCFG );
  uint16_t cmdPec, dataPec;

  if( addr >= 0 )
  {
    bind_address( &writeCmd, addr );
  } // end if

  cmdPec = crc15( &writeCmd, 1 );
  dataPec = crc15( regDataPtr, LTC6804_NUM_CONFIG_WORDS );

  begin_transaction();
  // Wakeup isoSPI port; assume core kept awake by keep alive timer
  wakeup_iso_spi();
  digitalWrite( _chipSel, LOW );
  SPI.transfer16( writeCmd );
  SPI.transfer16( cmdPec );
  spi_write_buf( regDataPtr, LTC6804_NUM_CONFIG_WORDS );
  SPI.transfer16( dataPec );
  digitalWrite( _chipSel, HIGH );
  end_transaction();
} // end write_config

// Reads the contents of the configuration registers of the IC whose address
// is supplied into the array whose starting address is supplied. Also reads
// the PEC associated with the data into the 16-bit value to which the supplied
// pointer points.
void LTC6804_2_Stack::read_config( const uint8_t addr, uint16_t* const regDataPtr, uint16_t* const dataPecPtr )
{
  uint16_t readCmd = prepare_command( RDCFG );
  uint16_t cmdPec;

  bind_address( &readCmd, addr );
  cmdPec = crc15( &readCmd, 1 );
  
  read_regs( 
    regDataPtr, dataPecPtr,
    &readCmd, &cmdPec, 
    LTC6804_NUM_CONFIG_REGS, LTC6804_NUM_CONFIG_WORDS_PER_REG );
} // end read_config

// Reads the contents of a '6804 IC's cell-voltage register-groups into the
// array whose starting address is supplied. Stores the recieved PECs
// for each register group into another array. The target IC is specified
// by its isoSPI address.
void LTC6804_2_Stack::read_cell_voltages( const uint8_t addr, uint16_t* const regDataPtr, uint16_t* const dataPecsPtr )
{
  uint16_t readCmds[LTC6804_NUM_CV_REGS] = {
    prepare_command( RDCVA ),
    prepare_command( RDCVB ),
    prepare_command( RDCVC ),
    prepare_command( RDCVD )
  };
  uint16_t cmdPecs[LTC6804_NUM_CV_REGS];

  bind_address_array( readCmds, LTC6804_NUM_CV_REGS, addr );
  crc15_group( readCmds, cmdPecs, 1, LTC6804_NUM_CV_REGS );
  read_regs( 
    regDataPtr, dataPecsPtr,
    readCmds, cmdPecs, 
    LTC6804_NUM_CV_REGS, LTC6804_NUM_CV_WORDS_PER_REG );
} // end read_cell_voltages

// Reads the contents of a '6804 IC's auxillary register-groups into the
// array whose starting address is supplied. Stores the recieved PECs
// for each register group into another array. The target IC is specified
// by its isoSPI address.
void LTC6804_2_Stack::read_auxillary( const uint8_t addr, uint16_t* const regDataPtr, uint16_t* const dataPecsPtr )
{
  uint16_t readCmds[LTC6804_NUM_AUX_REGS] = {
    prepare_command( RDAUXA ),
    prepare_command( RDAUXB )
  };
  uint16_t cmdPecs[LTC6804_NUM_AUX_REGS];

  bind_address_array( readCmds, LTC6804_NUM_AUX_REGS, addr );
  crc15_group( readCmds, cmdPecs, 1, LTC6804_NUM_AUX_REGS );
  read_regs( 
    regDataPtr, dataPecsPtr, 
    readCmds, cmdPecs, 
    LTC6804_NUM_AUX_REGS, LTC6804_NUM_AUX_WORDS_PER_REG );
} // end read_auxillary

// Reads the contents of a '6804 IC's status register-groups into the
// array whose starting address is supplied. Stores the recieved PECs
// for each register group into another array. The target IC is specified
// by its isoSPI address.
void LTC6804_2_Stack::read_status( const uint8_t addr, uint16_t* const regDataPtr, uint16_t* const dataPecsPtr )
{
  uint16_t readCmds[LTC6804_NUM_STAT_REGS] = {
    prepare_command( RDSTATA ),
    prepare_command( RDSTATB )
  };
  uint16_t cmdPecs[LTC6804_NUM_STAT_REGS];

  bind_address_array( readCmds, LTC6804_NUM_STAT_REGS, addr );
  crc15_group( readCmds, cmdPecs, 1, LTC6804_NUM_STAT_REGS );
  read_regs( 
    regDataPtr, dataPecsPtr, 
    readCmds, cmdPecs, 
    LTC6804_NUM_STAT_REGS, LTC6804_NUM_STAT_WORDS_PER_REG );
} // end read_status

// Writes the specified command word with the specified PEC over
// the SPI bus.
void LTC6804_2_Stack::write_cmd( const uint16_t cmd, const uint16_t cmdPec )
{
  begin_transaction();
  // Wakeup isoSPI port; assume core kept awake by keep alive timer
  wakeup_iso_spi();
  digitalWrite( _chipSel, LOW );
  SPI.transfer16( cmd );
  SPI.transfer16( cmdPec );
  digitalWrite( _chipSel, HIGH );
  end_transaction();
} // end write_cmd

// Reads data from register groups on a '6804 IC into the
// array whose starting address is specified given the
// set of commands (and the corresponding PECs) that yield the
// data from the chip (supplied as pointers to arrays). Also
// stores the PECs for the recieved register-group data into
// the array whose starting address is supplied. Additional arguments
// are the number of register groups to be read and the number of 16-bit
// words in each register group.
void LTC6804_2_Stack::read_regs( uint16_t* const regDataPtr, uint16_t* const dataPecsPtr, const uint16_t* const readCmdsPtr, const uint16_t* const cmdPecsPtr, const uint8_t numRegs, const uint8_t numWordsPerReg )
{
  begin_transaction();
  // Wakeup isoSPI port; assume core kept awake by keep alive timer
  wakeup_iso_spi();

  for( uint8_t i = 0; i < numRegs; i++ )
  {
    digitalWrite( _chipSel, LOW );
    SPI.transfer16( readCmdsPtr[i] );
    SPI.transfer16( cmdPecsPtr[i] );
    spi_read_buf( &regDataPtr[i * numWordsPerReg], numWordsPerReg );
    dataPecsPtr[i] = SPI.transfer16(0);
    digitalWrite( _chipSel, HIGH );
  } // end for
  
  end_transaction();
} // end read_regs

// Starts an SPI communication transaction with LTC6804 ICs.
void LTC6804_2_Stack::begin_transaction(void)
{
  mask_keep_alive_timer_int();
  SPI.beginTransaction( SPISettings(LTC6804_MAX_SPI_CLK_FREQ_HZ, MSBFIRST, SPI_MODE0) );
} // end begin_transaction

// Terminates an SPI communication transaction with LTC6804 ICs.
void LTC6804_2_Stack::end_transaction(void)
{
  SPI.endTransaction();
  unmask_keep_alive_timer_int();
} // end end_transaction

void LTC6804_2_Stack::mask_keep_alive_timer_int( void )
{
	_timerMasks[_stackId] = true;
} // end mask_keep_alive_timer_int

void LTC6804_2_Stack::unmask_keep_alive_timer_int( void )
{
	_timerMasks[_stackId] = false;
} // end unmask_keep_alive_timer_int

// Sends the data in the specified array of uint16_t values over
// the SPI port.
void LTC6804_2_Stack::spi_write_buf( const uint16_t* buf, uint8_t len )
{
  for( uint8_t i = 0; i < len; i++ )
  {
    SPI.transfer16( buf[i] );
  } // end for
} // end spi_write_buf

// Reads the specified number of 16-bit values from the
// SPI port into the array whose starting address is supplied.
void LTC6804_2_Stack::spi_read_buf( uint16_t* const buf, uint8_t len )
{
  for( uint8_t i = 0; i < len; i++ )
  {
    buf[i] = SPI.transfer16(0);
  } // end for
} // end spi_read_buf

void LTC6804_2_Stack::wakeup_iso_spi( void )
{
  digitalWrite( _chipSel, LOW );
  delayMicroseconds( LTC6804_ISO_SPI_WAKEUP_TIME_US );
  digitalWrite( _chipSel, HIGH );
} // end wakeup_iso_spi

void LTC6804_2_Stack::wakeup_core( void )
{
  digitalWrite( _chipSel, LOW );
  delay( LTC6804_CORE_WAKEUP_TIME_MS );
  digitalWrite( _chipSel, HIGH );
} // end wakeup_core

// Prepares a 16-bit command word based on the
// supplied command identifier (an 11-bit command code).
uint16_t LTC6804_2_Stack::prepare_command( const uint16_t code )
{
  return code & LTC6804_CC_MASK;
} // end prepare_command

// Inserts the supplied 4-bit isoSPI address into the
// command word to which the supplied pointer points;
// makes the command word target the IC with the specified address.
void LTC6804_2_Stack::bind_address( uint16_t* const cmdPtr, const uint8_t addr )
{
  *cmdPtr |= LTC6804_2_ADDR_BIT | (addr << LTC6804_2_CC_ADDR_LSB);
} // end bind_address

// Inserts the supplied 4-bit isoSPI address into each element in the
// array of command words whose starting address is supplied; makes
// the command words target the IC with the specified address.
void LTC6804_2_Stack::bind_address_array( uint16_t* const cmdPtr, uint8_t len, const uint8_t addr )
{
  for( uint8_t i = 0; i < len; i++ )
  {
    bind_address( &cmdPtr[i], addr );
  } // end for
} // end bind_address

// Inserts the least-significant portion of the supplied 8-bit
// LTC6804 command argument into the command word to which the supplied
// pointer points based on the specified argument identifier (an 8-bit
// value).
void LTC6804_2_Stack::bind_argument( uint16_t* const cmdPtr, const uint8_t argId, const uint8_t valId )
{
  *cmdPtr |= pgm_read_byte( &(LTC6804_2_CC_ARG_VALUES[argId][valId]) ) << pgm_read_byte( &LTC6804_2_CC_ARG_LSBS[argId] );
} // end bind_argument

// Inserts the supplied 16-bit data item into the array of
// register-group data whose starting address is provided based
// on the specified register-data identifier (an 8-bit value).
// Overwrites prevuously-inserted data.
void LTC6804_2_Stack::bind_data( uint16_t* const regGrp, const uint8_t dataId, const uint16_t valId )
{
  // Lookup positioning information for the data item corresponding to the supplied id
  const uint8_t NUM_DATA_SEGMENTS = pgm_read_byte( &(LTC6804_REG_DATA_POS[dataId][REG_DATA_NUM_SEGMENTS_INDEX]) ),
                DATA_SEGMENT_SIZE = pgm_read_byte( &(LTC6804_REG_DATA_POS[dataId][REG_DATA_SEGMENT_SIZE_INDEX]) );    
  const uint8_t DATA_SEGMENT_MASK = pgm_read_byte( &(LTC6804_REG_DATA_SEGMENT_MASKS[DATA_SEGMENT_SIZE - 1]) );
  
  uint16_t data;
  uint8_t wordNum, segLsb;
  uint8_t seg;

  // Interpret data argument as a value index if the supplied data ID is less than
  // the lookup threshold
  if( dataId <= END_REG_DATA_VALUES_LOOKUP )
  {
  	data = pgm_read_word( LTC6804_REG_DATA_VALUES[dataId] + data );
  } // end if
  else
  {
  	data = valId;
  } // end else

  for( uint8_t segNum = 0; segNum < NUM_DATA_SEGMENTS; segNum++ )
  {
    // Extract a segment of data from the data word
    seg = (data >> (DATA_SEGMENT_SIZE * segNum)) & DATA_SEGMENT_MASK;
    // Lookup positioning information for the data segment
    wordNum = pgm_read_byte( &(LTC6804_REG_DATA_POS[dataId][REG_DATA_NUM_GEN_DATA_ITEMS + REG_DATA_NUM_SEGMENT_DATA_ITEMS * segNum + REG_DATA_WORD_NUM_INDEX]) );
    segLsb = pgm_read_byte( &(LTC6804_REG_DATA_POS[dataId][REG_DATA_NUM_GEN_DATA_ITEMS + REG_DATA_NUM_SEGMENT_DATA_ITEMS * segNum + REG_DATA_SEGMENT_LSB_INDEX]) );

    // Clear data segment in register group
    regGrp[wordNum] &= ~( (uint16_t)DATA_SEGMENT_MASK << segLsb );
    // Insert the data segment into the supplied register-group data array
    regGrp[wordNum] |= (uint16_t)seg << segLsb;
  } // end for
} // end bind_data

// Extracts the 16-bit (or smaller) data item from the array of
// register-group data whose starting address is supplied based
// on the specified register-data identifier (an 8-bit value).
uint16_t LTC6804_2_Stack::strip_data( const uint16_t* const regGrp, const uint8_t dataId )
{
  // Lookup positioning information for the data item corresponding to the supplied id
  const uint8_t NUM_DATA_SEGMENTS = pgm_read_byte( &(LTC6804_REG_DATA_POS[dataId][REG_DATA_NUM_SEGMENTS_INDEX]) ),
                DATA_SEGMENT_SIZE = pgm_read_byte( &(LTC6804_REG_DATA_POS[dataId][REG_DATA_SEGMENT_SIZE_INDEX]) );
  const uint8_t DATA_SEGMENT_MASK = pgm_read_byte( &(LTC6804_REG_DATA_SEGMENT_MASKS[DATA_SEGMENT_SIZE - 1]) );

  // Make certian to initialize data to zero (inconsistent behavior otherwise)
  uint16_t data = 0;
  uint8_t wordNum, segLsb;
  uint16_t seg;

  for( uint8_t segNum = 0; segNum < NUM_DATA_SEGMENTS; segNum++ )
  {
    // Lookup positioning information for the data segment
    wordNum = pgm_read_byte( &(LTC6804_REG_DATA_POS[dataId][REG_DATA_NUM_GEN_DATA_ITEMS + REG_DATA_NUM_SEGMENT_DATA_ITEMS * segNum + REG_DATA_WORD_NUM_INDEX]) );
    segLsb = pgm_read_byte( &(LTC6804_REG_DATA_POS[dataId][REG_DATA_NUM_GEN_DATA_ITEMS + REG_DATA_NUM_SEGMENT_DATA_ITEMS * segNum + REG_DATA_SEGMENT_LSB_INDEX]) );
    // Extract a segment of data from the register group data
    seg = regGrp[wordNum] >> segLsb;
    seg &= DATA_SEGMENT_MASK;
    seg <<= DATA_SEGMENT_SIZE * segNum;

    // Insert the data segment into 16-bit data word
    data |= seg;
  } // end for

  return data;
} // end strip_data

void LTC6804_2_Stack::setup_keep_alive_timer( void )
{
  stop_keep_alive_timer();
  clear_keep_alive_timer();

  // Set waveform generation mode to normal
  TCCR1A &= ~( bit( WGM11 ) | bit( WGM10 ) );
  TCCR1B &= ~( bit( WGM13 ) | bit( WGM12 ) );
  
  // Enable interrupt on overflow
  TIMSK1 |= bit( TOIE1 );
} // end setup_keep_alive_timer

void LTC6804_2_Stack::clear_keep_alive_timer( void )
{
  TCNT1 = 0;
} // end clear_keep_alive_timer

void LTC6804_2_Stack::start_keep_alive_timer( void )
{
  // Start clocking the timer
  TCCR1B |= KEEP_ALIVE_TIMER_CLK_DIV_SEL;
} // end start_keep_alive_timer

void LTC6804_2_Stack::stop_keep_alive_timer( void )
{
  // Stop clocking the timer
  TCCR1B &= ~( bit(CS12) | bit(CS11) | bit(CS10) );
} // end stop_keep_alive_timer

void LTC6804_2_Stack::timer_overflow_isr( void )
{
	for( uint8_t i = 0; i < _numStacks; i++ )
  {
  	if( _timerMasks[i] )
  	{
  		continue;
  	} // end if

  	digitalWrite( _chipSels[i], LOW );
  	delayMicroseconds( LTC6804_ISO_SPI_WAKEUP_TIME_US );
  	digitalWrite( _chipSels[i], HIGH );
  } // end for
} // end timer_overflow_isr

// Performs a blocking delay sufficent to guarantee ADC completion, given the 
// analog-to-digital conversion (ADC) type, analog-to-digital converter mode option, 
// analog-to-digital converter mode, and analog-to-digital converter channel.
void LTC6804_2_Stack::adc_delay( const uint8_t conversionType, const uint8_t adcOpt, const uint8_t adcMode, const uint8_t channel )
{
	uint32_t delayUs = get_adc_delay_us( conversionType, adcOpt, adcMode, channel );


	if( delayUs > MAX_US_DELAY )
	{
		// Millisecond delay (round up microseconds to nearest whole millisecond)
		delay( (delayUs + NUM_US_IN_MS - 1) / NUM_US_IN_MS );
	} // end if
	else
	{
		// Microsecond delay
		delayMicroseconds( delayUs );
	} // end else
} // end adc_delay

// Based on the supplied analog-to-digital conversion (ADC) type, analog-to-digital converter
// mode option, analog-to-digital converter mode, and analog-to-digital converter channel,
// returns the amount of time (in mircoseconds) required for ADC completion.
uint32_t LTC6804_2_Stack::get_adc_delay_us( const uint8_t conversionType, const uint8_t adcOpt, const uint8_t adcMode, const uint8_t channel )
{
	return pgm_read_dword( &(LTC6804_ADC_TIMES_US[conversionType][adcOpt][adcMode][channel]) );
} // end get_adc_delay_us

bool LTC6804_2_Stack::digital_write( const int8_t addr, uint8_t gpioStates )
{
	// Array to hold configuration data from target IC for
	// read-modify-write procedure
	uint16_t configData[LTC6804_NUM_CV_REGS];

	if( addr < 0 )
	{
		// Broadcast write not supported
		return false;
	} // end if

	if( !get_raw_config( addr, configData ) )
	{
		// Failed to retrieve configuration words from IC
		return false;
	} // end if

	bind_data( configData, CONFIG_GPIO_PULLDOWN_CTL, gpioStates );

	if( !config( addr, configData ) )
	{
		// Failed to write modified configuration data
		return false;
	} // end if

	return true;
} // end digital_write

void LTC6804_2_Stack::bind_gpio_state( uint8_t* gpioStates, uint8_t gpio, uint8_t state )
{
	if( state == HIGH )
	{
		*gpioStates |= 1 << pgm_read_byte( &LTC6804_GPIO_MAP_POS[gpio] );
	} // end if
	else
	{
		*gpioStates &= ~( (uint8_t)(1 << pgm_read_byte( &LTC6804_GPIO_MAP_POS[gpio] )) );
	} // end else
} // end bind_state