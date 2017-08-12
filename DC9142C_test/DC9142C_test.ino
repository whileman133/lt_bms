// DC9143C_test.ino
//
// Written by Wes Hileman on 5 July 2016
// U. of Colorado, Colorado Springs
//
// Sketch that tests the functionality of the
// DC1942C_Stack class --specifically analog input
// voltage monitoring.

#include <Linduino.h>         // Linduino defines
#include <DC1942C_Stack.h>

// Cell voltage limits 
#define UNDERVOLTAGE_THRESHOLD 2.0  // Volts
#define OVERVOLTAGE_THRESHOLD 3.6   // Volts

// Rate of PC-Linduino serial communication
#define SERIAL_BAUD 250000
// Number of '6804 ICs on the stack connected to the Linduino
#define NUM_ICS 6
// Number of analog-input pins utilized on the DC1942C boards
#define NUM_AIS 8
// Adresses of the '6804 ICs
const uint8_t IC_ADDRESSES[NUM_ICS] = { 0, 1, 3, 4, 6, 7 };
// Analog input pins utlized on the DC1942C boards
const uint8_t AI_PINS[NUM_AIS] = {
  DC1942C_AI0, DC1942C_AI1, DC1942C_AI2, DC1942C_AI3, DC1942C_AI4, DC1942C_AI5, DC1942C_AI6, DC1942C_AI7
};
// Thermistor constants
#define THERMISTOR_B_PARAMETER 3950
#define THERMISTOR_AMBIENT_TEMPERATURE_KELVIN 297.6
const float THERMISTOR_AMBIENT_RESISTANCES_OHMS[NUM_ICS][NUM_AIS] PROGMEM = {
  {10730, 10730, 10730, 10730, 10730, 10730, 10730, 10730},
  {10730, 10730, 10730, 10730, 10730, 10730, 10730, 10730},
  {10730, 10730, 10730, 10730, 10730, 10730, 10730, 10730},
  {10730, 10730, 10730, 10730, 10730, 10730, 10730, 10730},
  {10730, 10730, 10730, 10730, 10730, 10730, 10730, 10730},
  {10730, 10730, 10730, 10730, 10730, 10730, 10730, 10730}
};

// Array to hold chip configuration words; make certian
// to initialize to zero. Declared as a global variable so
// that IC configuration can be verified periodically in loop(),
// but initialized in setup()
uint16_t configData[LTC6804_NUM_CONFIG_WORDS] = { 0 };

// Construct DC1942C_Stack object; the SPI chip select (CS)
// pin assocaited with the stack by is the only argument
DC1942C_Stack stack( QUIKEVAL_CS );

// Arrays to hold data read from ICs; defined as globals so 
// that the Arduino IDE shows memory usage
float cellVoltages[NUM_ICS][LTC6804_NUM_CELLS];
// DC1942C analog input data
float temperatures[NUM_ICS][NUM_AIS];

// Variables to hold data used in verifying IC configuration, voltage, ad GPIO data
bool cvOk, auxOk, configOk, analogOk;
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
  DC1942C_Stack::start();

  // Load array with configuration words generated from the supplied 
  // parameters
  stack.prepare_config(
    configData,
    PD_NONE,                        // GPIO pulldown enable
    REF_ON,                         // REFON bit
    ADC_OPT_1,                      // ADCOPT bit
    // Use macros to generate codes corresponding to under- and over-voltage values
    UV_THRESHOLD_CODE(UNDERVOLTAGE_THRESHOLD),  // Undervoltage threshold
    OV_THRESHOLD_CODE(OVERVOLTAGE_THRESHOLD),   // Overvoltage threshold
    D_NONE,                         // Cell discharge shunt enabe
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
    DC1942C_Stack::stop();
    while( true );
  } // end if
} // end setup

void loop()
{
  // Broadcast ADC start commands to all ICs on the stack
  // NOTE: ADC start commands cannot be issued one-after-the-other; must wait
  // for the previous conversion to finish before another can be started
  stack.start_cell_voltage_adc( BROADCAST, ADC_MODE_NORMAL, ADC_PERMIT_DISCHARGE_FALSE, ADC_CELL_CHANNEL_ALL );
  stack.adc_delay( CELL_VOLTAGE, ADC_OPT_1, ADC_MODE_NORMAL, ADC_CELL_CHANNEL_ALL );

  // Fetch cell-voltage and auxillary data from ICs
  cvOk = stack.get_all_cell_voltages( NUM_ICS, IC_ADDRESSES, cellVoltages );
  // Verify ICs have retianed proper configuration
  configOk = stack.verify_all_config( NUM_ICS, IC_ADDRESSES, configData, configStatus );
  // Fetch analog-input voltages from ICs
  analogOk = stack.thermistor_read_all( 
    NUM_ICS, IC_ADDRESSES, 
    NUM_AIS, AI_PINS, 
    ADC_OPT_1, ADC_MODE_NORMAL, 
    THERMISTOR_B_PARAMETER, THERMISTOR_AMBIENT_TEMPERATURE_KELVIN, THERMISTOR_AMBIENT_RESISTANCES_OHMS[0], 
    temperatures[0] );
  
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

  Serial.print( "CHIP CONFIGURATION STATUS: " );
  Serial.println( configOk ? "No errors detected." : "Error(s) detected!" );
  Serial.println( "Columns <=> ICs" );
  print_bool_array( configStatus, NUM_ICS, "OK", "ERR" );

  Serial.println();

  if( analogOk )
  {
    Serial.println( "TEMPERATURES" );
    Serial.println( "Rows <=> ICs / Columns <=> Thermistors" );
    print_float_matrix( temperatures[0], NUM_ICS, NUM_AIS, 3, "*C" );
  } // end if
  else
  {
    Serial.println( "Could not fetch analog input data." );
  } // end else

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

void print_binary16_array( uint16_t* arr, uint8_t len )
{
  for( uint8_t i = 0; i < len; i++ )
  {
    Serial.print( arr[i], BIN );
    Serial.print( " " );
  } // end for

  Serial.println();
} // end print_binary_array

void print_binary8_array( uint8_t* arr, uint8_t len )
{
  for( uint8_t i = 0; i < len; i++ )
  {
    Serial.print( arr[i], BIN );
    Serial.print( " " );
  } // end for

  Serial.println();
} // end print_binary_array

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

void print_uint16_matrix( uint16_t* arr, uint8_t nRows, uint8_t nCols, char* unit )
{
  for( uint8_t row = 0; row < nRows; row++ )
  {
    for( uint8_t col = 0; col < nCols; col++ )
    {
      Serial.print( arr[row * nCols + col] );
      Serial.print( unit );
      Serial.print( " " );
    } // end for

    Serial.println();
  } // end for
} // end print_uint16_matrix

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
