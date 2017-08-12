// LTC6804_2_demo.ino
//
// Written by Wes Hileman on 14 June 2016
// U. of Colorado, Colorado Springs
//
// Sketch to demonstrate the functionality of the
// LTC6804_2_Stack library on the Linduino One board.

#include <Linduino.h>         // Linduino defines
#include <LTC6804_2_Stack.h>  // Class to send and read data from '6804-2 stacks

// Rate of PC-Linduino serial communication
#define SERIAL_BAUD 9600
// Amount of time (in ms) to wait for Analog-to-Digital Conversion (ADC) to finish
#define ADC_TIME_MS 1000
// Number of '6804 ICs on the stack connected to the Linduino
#define NUM_ICS 1
// Adresses of the '6804 ICs
const uint8_t IC_ADDRESSES[NUM_ICS] = { 0 };

// Array to hold chip configuration words; make certian
// to initialize to zero. Declared as a global variable so
// that IC configuration can be verified periodically in loop(),
// but initialized in setup()
uint16_t configData[LTC6804_NUM_CONFIG_WORDS] = { 0 };

// Construct LTC6804_2_Stack object; the SPI chip select (CS)
// pin assocaited with the stack by is the only argument
LTC6804_2_Stack stack( QUIKEVAL_CS );

// Arrays to hold data read from ICs
// Defined as globals so that the Arduino IDE shows
// memory usage
float cellVoltages[NUM_ICS][LTC6804_NUM_CELLS];
float gpioVoltages[NUM_ICS][LTC6804_NUM_GPIOS];
float ref2Voltages[NUM_ICS];

// Variables to hold data used in verifying IC configuration, voltage, ad GPIO data
bool cvOk = false, auxOk = false, configOk = false;
bool configStatus[NUM_ICS];

void setup() 
{  
  // Initialize PC-Linduino serial communication
  Serial.begin( SERIAL_BAUD );
  
  // Enable QuikEval SPI (disables QuikEval I2C) - Linduino only
  pinMode( QUIKEVAL_MUX_MODE_PIN, OUTPUT );
  digitalWrite( QUIKEVAL_MUX_MODE_PIN, LOW );

  // Setup stack-specific hardware; this initializes the stack's
  // chip-select line
  stack.setup();
  // Call after all stack-object setup() functions have been called;
  // this initializes SPI and timer hardware shared between stacks
  LTC6804_2_Stack::start();

  // Load array with configuration words generated from the supplied 
  // parameters
  stack.prepare_config(
    configData,
    PD_NONE,                        // GPIO pulldown enable
    REF_ON,                         // REFON bit
    ADC_OPT_1,                      // ADCOPT bit
    // Use macros to generate codes corresponding to under- and over-voltage values
    UV_THRESHOLD_CODE(2.0),         // Undervoltage threshold
    OV_THRESHOLD_CODE(3.6),         // Overvoltage threshold
    0,                              // Cell discharge shunt enabe
    DISCHARGE_TIMEOUT_OFF           // Cell discharge timer
  );
  Serial.println( "\n\nPREPARED CONFIGURATION" );
  print_reg_group( configData, LTC6804_NUM_CONFIG_WORDS_PER_REG, 1 );
  Serial.print( "\n\n" );

  // Attempt to write configuration data to all '6804 ICs on the stack
  if( !stack.config_all( NUM_ICS, IC_ADDRESSES, configData ) )
  {
    // Could not verify all ICs configured properly; halt SPI communications
    // and output a message to the user
    Serial.println( "Failed to verify IC configuration! Verify connections and software constants (e.g. IC count and addresses)." );
    LTC6804_2_Stack::stop();
    while( true );
  } // end if
} // end setup

void loop()
{
  // Broadcast ADC start commands to all ICs on the stack
  stack.start_cell_voltage_adc( BROADCAST, ADC_MODE_NORMAL, ADC_PERMIT_DISCHARGE_FALSE, ADC_CELL_CHANNEL_ALL );
  //stack.start_auxillary_adc( BROADCAST, ADC_MODE_NORMAL, ADC_AUX_CHANNEL_ALL );

  // Wait for ADC to finish
  //delay( ADC_TIME_MS );
  stack.adc_delay( CELL_VOLTAGE, ADC_OPT_1, ADC_MODE_NORMAL, ADC_CELL_CHANNEL_ALL );

  // Fetch cell-voltage and auxillary data from ICs
  cvOk = stack.get_all_cell_voltages( NUM_ICS, IC_ADDRESSES, cellVoltages );
  //auxOk = stack.get_all_auxillary( NUM_ICS, IC_ADDRESSES, gpioVoltages, ref2Voltages );
  // Verify ICs have retianed proper configuration
  configOk = stack.verify_all_config( NUM_ICS, IC_ADDRESSES, configData, configStatus );

  // Output collected data

  if( cvOk )
  {
    Serial.println( "CELL VOLTAGES:" );
    Serial.println( "Rows <=> ICs / Columns <=> Cells" );
    print_float_matrix( cellVoltages[0], NUM_ICS, LTC6804_NUM_CELLS, 3, "V" );
  } // end if
  else
  {
    Serial.println( "Could not fetch cell-voltage data." );
  } // end else

  Serial.println();

  if( auxOk )
  {
    Serial.println( "GPIO VOLTAGES:" );
    Serial.println( "Rows <=> ICs / Columns <=> Cells" );
    print_float_matrix( gpioVoltages[0], NUM_ICS, LTC6804_NUM_GPIOS, 3, "V" );
    Serial.println();
    
    Serial.println( "SECOND REFERENCE VOLTAGES:" );
    Serial.println( "Columns <=> ICs" );
    print_float_array( ref2Voltages, NUM_ICS, 3, "V" );
  } // end if
  else
  {
    Serial.println( "Could not fetch auxillary data." );
  } // end else

  Serial.println();

  Serial.print( "CHIP CONFIGURATION STATUS: " );
  Serial.println( configOk ? "No errors detected." : "Error(s) detected!" );
  Serial.println( "Columns <=> ICs" );
  print_bool_array( configStatus, NUM_ICS, "OK", "ERR" );

  Serial.println();
} // end loop

//
// Helper functions
//

void print_float_array( float* arr, uint8_t len, uint8_t prec, char* unit )
{
  for( uint8_t i = 0; i < len; i++ )
  {
    Serial.print( arr[i], prec );
    Serial.print( unit );
    Serial.print( " " );
  } // end for

  Serial.println();
} // end print_float_array

void print_bool_array( bool* arr, uint8_t len, char* trueVal, char* falseVal )
{
  for( uint8_t i = 0; i < len; i++ )
  {
    Serial.print( arr[i] ? trueVal : falseVal );
    Serial.print( " " );
  } // end for

  Serial.println();
} // end print_float_array

void print_float_matrix( float* arr, uint8_t nRows, uint8_t nCols, uint8_t prec, char* unit )
{
  for( uint8_t row = 0; row < nRows; row++ )
  {
    for( uint8_t col = 0; col < nCols; col++ )
    {
      Serial.print( arr[row * nCols + col], prec );
      Serial.print( unit );
      Serial.print( " " );
    } // end for

    Serial.println();
  } // end for
} // end print_float_matrix

void print_reg_group( const uint16_t* const regDataPtr, const uint8_t grpSize, const uint8_t numGrps )
{
  for( uint8_t grp = 0; grp < numGrps; grp++ )
  {    
    for( uint8_t grpElement = grpSize * grp; grpElement < grpSize * (grp + 1); grpElement++ )
    {
      Serial.print( "0x" );
      Serial.print( regDataPtr[grpElement], HEX );
      Serial.print( " " );
    } // end for

    Serial.println();
  } // end for
} // end print_reg_group
