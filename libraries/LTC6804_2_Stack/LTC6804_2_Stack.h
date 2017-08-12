// LTC6804_2_Stack.h
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

#ifndef LTC6804_2_STACK_H
#define LTC6804_2_STACK_H

#include "Arduino.h"
// For communicating with '6804 chips via SPI.
#include "SPI.h"
// For storing lookup tables in program memory. Note that special
// program-memory access functions such as pgm_read_dword, pgm_read_word, and 
// pgm_read_byte must be used to access data stored in program memory (i.e. const 
// globals with the PROGMEM modifier).
#include "avr/pgmspace.h"
// For computing packet error codes (PECs).
#include "CRC15.h"
// For functions that operate on 16-bit arrays.
#include "array16.h"

// Broadcast specifier
#define BROADCAST -1
#define FIXED_POINT 0
#define FLOATING_POINT 1

// Ids for types of IC data
#define CELL_VOLTAGE 0
#define AUXILLARY 1
#define STATUS 2

// GPIO pin ids
#define GPIO1 0
#define GPIO2 1
#define GPIO3 2
#define GPIO4 3
#define GPIO5 4

// Command codes
#define WRCFG 0x001
#define RDCFG 0x002
#define RDCVA 0x004
#define RDCVB 0x006
#define RDCVC 0x008
#define RDCVD 0x00A
#define RDAUXA 0x00C
#define RDAUXB 0x00E
#define RDSTATA 0x010
#define RDSTATB 0x012
#define ADCV 0x260
#define ADOW 0x228
#define CVST 0x207
#define ADAX 0x460
#define AXST 0x407
#define ADSTAT 0x468
#define STATST 0x40F
#define ADCVAX 0x46F
#define CLRCELL 0x711
#define CLRAUX 0x712
#define CLRSTAT 0x713
#define PLADC 0x714
#define DIAGN 0x715
#define WRCOMM 0x721
#define RDCOMM 0x722
#define STCOMM 0x723

// Argument identifiers
#define ADC_MODE 0
#define ADC_PERMIT_DISCHARGE 1
#define ADC_CELL_CHANNEL 2
#define OPEN_WIRE_PULLUP 3
#define SELF_TEST_MODE 4
#define ADC_AUX_CHANNEL 5
#define ADC_STATUS_CHANNEL 6

// Maxumum number of values a command argument
// can take on (for initializing lookup table)
#define LTC6804_NUM_ADC_MODES 3
#define LTC6804_MAX_NUM_CC_ARG_VALUES 7
#define LTC6804_MAX_NUM_ADC_CHANNELS 7
#define LTC6804_NUM_ADC_CELL_CHANNELS 7
// Argument value identifiers
#define ADC_MODE_FAST 0
#define ADC_MODE_NORMAL 1
#define ADC_MODE_FILTERED 2
#define ADC_PERMIT_DISCHARGE_FALSE 0
#define ADC_PERMIT_DISCHARGE_TRUE 1
#define ADC_CELL_CHANNEL_ALL 0
#define ADC_CELL_CHANNEL_1_7 1
#define ADC_CELL_CHANNEL_2_8 2
#define ADC_CELL_CHANNEL_3_9 3
#define ADC_CELL_CHANNEL_4_10 4
#define ADC_CELL_CHANNEL_5_11 5
#define ADC_CELL_CHANNEL_6_12 6
#define OPEN_WIRE_PULL_DOWN_CURRENT 0
#define OPEN_WIRE_PULL_UP_CURRENT 1
#define SELF_TEST_MODE_1 0
#define SELF_TEST_MODE_2 1
#define ADC_AUX_CHANNEL_ALL 0
#define ADC_AUX_CHANNEL_GPIO1 1
#define ADC_AUX_CHANNEL_GPIO2 2
#define ADC_AUX_CHANNEL_GPIO3 3
#define ADC_AUX_CHANNEL_GPIO4 4
#define ADC_AUX_CHANNEL_GPIO5 5
#define ADC_AUX_CHANNEL_2ND_REF 6
#define ADC_STATUS_CHANNEL_ALL 0
#define ADC_STATUS_CHANNEL_SOC 1
#define ADC_STATUS_CHANNEL_ITMP 2
#define ADC_STATUS_CHANNEL_VA 3
#define ADC_STATUS_CHANNEL_VD 4

// Register-data identifiers
// Data items with a small number of discrete values
#define CONFIG_REF 0
#define CONFIG_ADC_OPT 1
#define CONFIG_DISCHARGE_TIMEOUT 2
#define END_REG_DATA_VALUES_LOOKUP 2
// Data items with a large number of discrete values
#define CONFIG_GPIO_PULLDOWN_CTL 3
#define CONFIG_CELL_DISCHARGE_CTL 4
#define CONFIG_UV_THRESH 5
#define CONFIG_OV_THRESH 6
// Read-only data items
#define CELL1_VOLTAGE 7
#define CELL2_VOLTAGE 8
#define CELL3_VOLTAGE 9
#define CELL4_VOLTAGE 10
#define CELL5_VOLTAGE 11
#define CELL6_VOLTAGE 12
#define CELL7_VOLTAGE 13
#define CELL8_VOLTAGE 14
#define CELL9_VOLTAGE 15
#define CELL10_VOLTAGE 16
#define CELL11_VOLTAGE 17
#define CELL12_VOLTAGE 18
#define GPIO1_VOLTAGE 19
#define GPIO2_VOLTAGE 20
#define GPIO3_VOLTAGE 21
#define GPIO4_VOLTAGE 22
#define GPIO5_VOLTAGE 23
#define REF2_VOLTAGE 24
#define SUM_OF_CELLS_VOLTAGE 25
#define INTERNAL_TEMPERATURE 26
#define ANALOG_SUPPLY_VOLTAGE 27
#define DIGITAL_SUPPLY_VOLTAGE 28
#define UNDERVOLTAGE_FLAGS 29
#define OVERVOLTAGE_FLAGS 30
#define MUX_FAIL 31
#define THERMAL_SHUTDOWN 32

// Register-data positioning support
// The maximum data-segment size is 8-bits
#define REG_DATA_NUM_GEN_DATA_ITEMS 2
#define REG_DATA_MAX_NUM_SEGMENTS 12
#define REG_DATA_NUM_SEGMENT_DATA_ITEMS 2
#define REG_DATA_NUM_SEGMENTS_INDEX 0
#define REG_DATA_SEGMENT_SIZE_INDEX 1
#define REG_DATA_WORD_NUM_INDEX 0
#define REG_DATA_SEGMENT_LSB_INDEX 1

#define LTC6804_NUM_ADC_OPTS 2

// Register-data value ids
// (for use in value lookup table)
#define REF_ON 0
#define REF_OFF 1
#define ADC_OPT_0 0
#define ADC_OPT_1 1
#define DISCHARGE_TIMEOUT_OFF 0
#define DISCHARGE_TIMEOUT_30_SEC 1
#define DISCHARGE_TIMEOUT_1_MIN 2
#define DISCHARGE_TIMEOUT_2_MIN 3
#define DISCHARGE_TIMEOUT_3_MIN 4
#define DISCHARGE_TIMEOUT_4_MIN 5
#define DISCHARGE_TIMEOUT_5_MIN 6
#define DISCHARGE_TIMEOUT_10_MIN 7
#define DISCHARGE_TIMEOUT_15_MIN 8
#define DISCHARGE_TIMEOUT_20_MIN 9
#define DISCHARGE_TIMEOUT_30_MIN 10
#define DISCHARGE_TIMEOUT_40_MIN 11
#define DISCHARGE_TIMEOUT_60_MIN 12
#define DISCHARGE_TIMEOUT_75_MIN 13
#define DISCHARGE_TIMEOUT_90_MIN 14
#define DISCHARGE_TIMEOUT_120_MIN 15

// Register-data value helpers
// PD => PullDown
#define PD_NONE 0
#define PD_GPIO1 bit(0)
#define PD_GPIO2 bit(1)
#define PD_GPIO3 bit(2)
#define PD_GPIO4 bit(3)
#define PD_GPIO5 bit(4)
// D => Discharge
#define D_NONE 0
#define D_CELL1 bit(0)
#define D_CELL2 bit(1)
#define D_CELL3 bit(2)
#define D_CELL4 bit(3)
#define D_CELL5 bit(4)
#define D_CELL6 bit(5)
#define D_CELL7 bit(6)
#define D_CELL8 bit(7)
#define D_CELL9 bit(8)
#define D_CELL10 bit(9)
#define D_CELL11 bit(10)
#define D_CELL12 bit(11)

#define UV_THRESHOLD_CODE(uv) (uint16_t)((float)uv * 625.0) - 1
#define OV_THRESHOLD_CODE(ov) (uint16_t)((float)ov * 625.0)
#define CELL_VOLTAGE_CALC(cv_code) (float)cv_code / 10000.0
#define GPIO_VOLTAGE_CALC(gv_code) (float)gv_code / 10000.0
#define REF2_VOLTAGE_CALC(rv_code) (float)rv_code / 10000.0
#define SUM_OF_CELLS_VOLTAGE_CALC(sv_code) (float)sv_code / 500.0
#define INTERNAL_TEMPERATURE_CALC(tmp_code) (float)tmp_code / 75.0 - 273.0
#define ANALOG_SUPPLY_VOLTAGE_CALC(asv_code) (float)asv_code / 10000.0
#define DIGITAL_SUPPLY_VOLTAGE_CALC(dsv_code) (float)dsv_code / 10000.0

#define LTC6804_NUM_CV_WORDS_PER_REG 3
#define LTC6804_NUM_CV_REGS 4
#define LTC6804_NUM_CV_WORDS LTC6804_NUM_CV_WORDS_PER_REG * LTC6804_NUM_CV_REGS
#define LTC6804_NUM_AUX_WORDS_PER_REG 3
#define LTC6804_NUM_AUX_REGS 2
#define LTC6804_NUM_AUX_WORDS LTC6804_NUM_AUX_WORDS_PER_REG * LTC6804_NUM_AUX_REGS
#define LTC6804_NUM_STAT_WORDS_PER_REG 3
#define LTC6804_NUM_STAT_REGS 2
#define LTC6804_NUM_STAT_WORDS LTC6804_NUM_STAT_WORDS_PER_REG * LTC6804_NUM_STAT_REGS
#define LTC6804_NUM_CONFIG_REGS 1
#define LTC6804_NUM_CONFIG_WORDS_PER_REG 3
#define LTC6804_NUM_CONFIG_WORDS LTC6804_NUM_CONFIG_REGS * LTC6804_NUM_CONFIG_WORDS_PER_REG

#define LTC6804_NUM_CELLS LTC6804_NUM_CV_WORDS
#define LTC6804_NUM_GPIOS 5
#define LTC6804_DEFAULT_GPIO_STATE B00011111

class LTC6804_2_Stack
{
	public:
		LTC6804_2_Stack( uint16_t chipSel );
		~LTC6804_2_Stack( void );
		void setup( void );
		static void start( void );
		static void stop( void );
		bool config_all( const uint8_t icCount, const uint8_t* const icAddresses, const uint16_t* const cfgGrp );
		bool config( const int8_t addr, const uint16_t* const cfgGrp );
		bool verify_all_config( const uint8_t icCount, const uint8_t* const icAddresses, const uint16_t* const cfgGrp, bool* const flags );
		bool verify_config( const int8_t addr, const uint16_t* const cfgGrp );
		bool get_raw_config( const int8_t addr, uint16_t* const configData );
		template <typename CV_T> bool get_all_cell_voltages( const uint8_t icCount, const uint8_t* const icAddresses, CV_T cellVoltages[][LTC6804_NUM_CELLS] );
		template <typename CV_T> bool get_cell_voltages( const int8_t addr, CV_T *const cellVoltages );
		bool get_raw_cell_voltages( const int8_t addr, uint16_t* cvData );
		template <typename AUX_T> bool get_all_auxillary( const uint8_t icCount, const uint8_t* const icAddresses, AUX_T gpioVoltages[][LTC6804_NUM_GPIOS], AUX_T* const ref2Voltages );
		template <typename AUX_T> bool get_auxillary( const int8_t addr, AUX_T* const gpioVoltages, AUX_T* const ref2VoltagePtr );
		bool get_raw_auxillary( const int8_t addr, uint16_t* const auxData );
		template <typename STAT_T> bool get_all_status( const uint8_t icCount, const uint8_t* const icAddresses, STAT_T socVoltages[], STAT_T internalTemps[], STAT_T analogSupplyVoltages[], STAT_T digitalSupplyVoltages[], uint16_t uvFlags[], uint16_t ovFlags[], uint8_t muxFails[], uint8_t thermalShutdowns[] );
		template <typename STAT_T> bool get_status( const int8_t addr, STAT_T* const socVoltagePtr, STAT_T* const internalTempPtr, STAT_T* const analogSupplyVoltagePtr, STAT_T* const digitalSupplyVoltagePtr, uint16_t* const uvFlagsPtr, uint16_t* const ovFlagsPtr, uint8_t* const muxFailPtr, uint8_t* const thermalShutdownPtr  );
		bool get_raw_status( const int8_t addr, uint16_t* const statData );
		static void prepare_config( uint16_t* const cfgGrp, const uint8_t gpioPulldownCtl, const uint8_t ref, const uint8_t adcOpt, const uint16_t uvThreshold, const uint16_t ovThreshold, const uint16_t cellDischargeCtl, const uint8_t dischargeTimeout );
		static void extract_cell_voltages( const uint16_t* const regData, float* const cvData );
		static void extract_cell_voltages( const uint16_t* const regData, uint16_t* const cvData );
		static void extract_auxillary( const uint16_t* const regData, float* const gpioVoltages, float* const ref2Voltage );
		static void extract_auxillary( const uint16_t* const regData, uint16_t* const gpioVoltages, uint16_t* const ref2Voltage );
		static void extract_status( const uint16_t* const regData, float* const socVoltage, float* const internalTemp, float* const analogSupplyVoltage, float* const digitalSupplyVoltage, uint16_t* const uvFlags, uint16_t* const ovFlags, uint8_t* const muxFail, uint8_t* const thermalShutdown );
		static void extract_status( const uint16_t* const regData, uint16_t* const socVoltage, uint16_t* const internalTemp, uint16_t* const analogSupplyVoltage, uint16_t* const digitalSupplyVoltage, uint16_t* const uvFlags, uint16_t* const ovFlags, uint8_t* const muxFail, uint8_t* const thermalShutdown );
		void clear_status( const int8_t addr );
		void start_cell_voltage_adc( const int8_t addr, const uint8_t mode, const uint8_t dischargePermit, const uint8_t cellChannel );
		void start_auxillary_adc( const int8_t addr, const uint8_t mode, const uint8_t auxChannel );
		void start_status_adc( const int8_t addr, const uint8_t mode, const uint8_t selfTestChannel );
		void start_diagn( const int8_t addr );
		void write_config( const int8_t addr, const uint16_t* const regDataPtr );
		void read_config( const uint8_t addr, uint16_t* const regDataPtr, uint16_t* const dataPecPtr );
		void read_cell_voltages( const uint8_t addr, uint16_t* const regDataPtr, uint16_t* const dataPecsPtr );
		void read_auxillary( const uint8_t addr, uint16_t* const regDataPtr, uint16_t* const dataPecsPtr );
		void read_status( const uint8_t addr, uint16_t* const regDataPtr, uint16_t* const dataPecsPtr );
		void write_cmd( const uint16_t cmd, const uint16_t cmdPec );
		void read_regs( uint16_t* const regDataPtr, uint16_t* const dataPecsPtr, const uint16_t* const readCmdsPtr, const uint16_t* const cmdPecsPtr, const uint8_t numRegs, const uint8_t numWordsPerReg );
		void begin_transaction( void );
		void end_transaction( void );
		void mask_keep_alive_timer_int( void );
		void unmask_keep_alive_timer_int( void );
		void spi_write_buf( const uint16_t* buf, uint8_t len );
		void spi_read_buf( uint16_t* const buf, uint8_t len );
		void wakeup_iso_spi( void );
		void wakeup_core( void );
		static uint16_t prepare_command( const uint16_t code );
		static void bind_address( uint16_t* const cmdPtr, const uint8_t addr );
		static void bind_address_array( uint16_t* const cmdPtr, uint8_t len, const uint8_t addr );
		static void bind_argument( uint16_t* const cmdPtr, const uint8_t argId, const uint8_t arg );
		static void bind_data( uint16_t* const regGrp, const uint8_t dataId, const uint16_t data );
		static uint16_t strip_data( const uint16_t* const regGrp, const uint8_t dataId );
		static void setup_keep_alive_timer( void );
		static void clear_keep_alive_timer( void );
		static void start_keep_alive_timer( void );
		static void stop_keep_alive_timer( void );
		static void timer_overflow_isr( void );
		static void adc_delay( const uint8_t conversionType, const uint8_t adcOpt, const uint8_t adcMode, const uint8_t channel );
		static uint32_t get_adc_delay_us( const uint8_t conversionType, const uint8_t adcOpt, const uint8_t adcMode, const uint8_t channel );

		// Digital-input (GPIO) support
		bool digital_write( int8_t addr, uint8_t gpioStates );
		void bind_gpio_state( uint8_t* gpioStates, uint8_t gpio, uint8_t state );

	private:
		static uint8_t _numStacks;
		static uint16_t* _chipSels;
		static bool* _timerMasks;
		uint16_t _chipSel, _stackId;
};

// Reads cell voltages from all of the '6804 ICs targeted by the
// addresses in the supplied array into the two-dimensional floating
// point array whose starting address is given. Cell voltages from
// each IC are read into the columns of the 2D array; the rows of the
// array correspond to each IC. Returns true if the read was completed 
// successfully, false otherwise.
template <typename CV_T>
bool LTC6804_2_Stack::get_all_cell_voltages( const uint8_t icCount, const uint8_t* const icAddresses, CV_T cellVoltages[][LTC6804_NUM_CELLS] )
{
	for( uint8_t icIndex = 0; icIndex < icCount; icIndex++ )
	{
		if( !get_cell_voltages<CV_T>( icAddresses[icIndex], cellVoltages[icIndex] ) )
		{
			return false;
		} // end if
	} // end for

	return true;
} // end get_all_cell_voltages

// Reads cell voltages from the '6804 IC targeted by the supplied address into the
// floating-point array whose starting address is given. Broadcast reads are not supported 
// by this function; use get_all_cell_voltages instead. Returns true if the read was
// completed successfully, false otherwise. Implements a data integrity check by verifying
// recieved PECs and requesting retransmission if data is correupted.
template <typename CV_T>
bool LTC6804_2_Stack::get_cell_voltages( const int8_t addr, CV_T *const cellVoltages )
{
  uint16_t	cvData[LTC6804_NUM_CV_WORDS];

  if( !get_raw_cell_voltages( addr, cvData ) )
  {
  	// Read failed, unable to fetch auxillary voltages
  	return false;
  } // end if

	extract_cell_voltages( cvData, cellVoltages );

  return true;
} // end get_cell_voltages

// Reads auxillary voltages from all of the '6804 ICs targeted by the
// addresses in the supplied array into the two-dimensional floating
// point array whose starting address is given. GPIO voltages from
// each IC are read into the columns of the 2D array; the rows of the
// array correspond to each IC. Returns true if the read was completed 
// successfully, false otherwise.
template <typename AUX_T>
bool LTC6804_2_Stack::get_all_auxillary( const uint8_t icCount, const uint8_t* const icAddresses, AUX_T gpioVoltages[][LTC6804_NUM_GPIOS], AUX_T* const ref2Voltages )
{
	for( uint8_t icIndex = 0; icIndex < icCount; icIndex++ )
	{
		if( !get_auxillary( icAddresses[icIndex], gpioVoltages[icIndex], &ref2Voltages[icIndex] ) )
		{
			return false;
		} // end if
	} // end for

	return true;
} // end get_all_auxillary

// Reads auxillary voltages from the '6804 IC targeted by the supplied address into the
// floating-point array whose starting address is given. Broadcast reads are not supported 
// by this function; use get_all_cell_voltages instead. Returns true if the read was
// completed successfully, false otherwise. Implements a data integrity check by verifying
// recieved PECs and requesting retransmission if data is corrupted.
template <typename AUX_T>
bool LTC6804_2_Stack::get_auxillary( const int8_t addr, AUX_T* const gpioVoltages, AUX_T* const ref2VoltagePtr )
{
	uint16_t auxData[LTC6804_NUM_AUX_WORDS];

  if( !get_raw_auxillary( addr, auxData ) )
  {
  	// Read failed, unable to fetch auxillary voltages
  	return false;
  } // end if

	// Read succedded, extract auxillary data
  extract_auxillary( auxData, gpioVoltages, ref2VoltagePtr );
  return true;
} // end get_auxillary

// Reads status data from all of the '6804 ICs targeted by the
// addresses in the supplied array into the arrsys whose starting 
// addresses are given. Items in the array correspond to ICs in the
// order of the supplied address array. Returns true if the read was 
// completed successfully, false otherwise.
template <typename STAT_T> 
bool LTC6804_2_Stack::get_all_status( const uint8_t icCount, const uint8_t* const icAddresses, STAT_T socVoltages[], STAT_T internalTemps[], STAT_T analogSupplyVoltages[], STAT_T digitalSupplyVoltages[], uint16_t uvFlags[], uint16_t ovFlags[], uint8_t muxFails[], uint8_t thermalShutdowns[] )
{
	for( uint8_t icIndex = 0; icIndex < icCount; icIndex++ )
	{
		if( !get_status( icAddresses[icIndex], &socVoltages[icIndex], &internalTemps[icIndex], &analogSupplyVoltages[icIndex], &digitalSupplyVoltages[icIndex], &uvFlags[icIndex], &ovFlags[icIndex], &muxFails[icIndex], &thermalShutdowns[icIndex] ) )
		{
			return false;
		} // end if
	} // end for

	return true;
} // end get_all_status

// Reads status data from the '6804 IC targeted by the supplied address into the
// variables whose addresses are given. Broadcast reads are not supported 
// by this function; use get_all_cell_status instead. Returns true if the read was
// completed successfully, false otherwise. Implements a data integrity check by verifying
// recieved PECs and requesting retransmission if data is corrupted.
template <typename STAT_T> 
bool LTC6804_2_Stack::get_status( const int8_t addr, STAT_T* const socVoltagePtr, STAT_T* const internalTempPtr, STAT_T* const analogSupplyVoltagePtr, STAT_T* const digitalSupplyVoltagePtr, uint16_t* const uvFlagsPtr, uint16_t* const ovFlagsPtr, uint8_t* const muxFailPtr, uint8_t* const thermalShutdownPtr )
{
	uint16_t	statData[LTC6804_NUM_STAT_WORDS];

  if( !get_raw_status( addr, statData ) )
  {
  	return false;
  } // end if

	// Read succedded, parse status data
  extract_status( statData, socVoltagePtr, internalTempPtr, analogSupplyVoltagePtr, digitalSupplyVoltagePtr, uvFlagsPtr, ovFlagsPtr, muxFailPtr, thermalShutdownPtr );

  return true;
} // end get_status

#endif