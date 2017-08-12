// LTC6804_2_test.ino
//
// Written by Wes Hileman on 7 June 2016
// U. of Colorado, Colorado Springs
//
// Sketch to test the functionality of the
// LTC6804_2_Stack library on the Linduino One
// board.

#include <Linduino.h>
#include <LTC6804_2_Stack.h>

#define SERIAL_BAUD 9600
#define NUM_ICS 2
const uint8_t IC_ADDRESSES[NUM_ICS] = { 0, 1 };

LTC6804_2_Stack stack( QUIKEVAL_CS );

void setup() 
{
  const char* regNames[] = {
    "CFG REG"
  };
  
  // Array to hold chip configuration words; make certian
  // to initialize to zero
  uint16_t configData[LTC6804_NUM_CONFIG_WORDS] = { 0 };

  // Initialize serial communication with PC
  Serial.begin( SERIAL_BAUD );
  
  // Enable QuikEval SPI (disables QuikEval I2C) - Linduino only
  pinMode( QUIKEVAL_MUX_MODE_PIN, OUTPUT );
  digitalWrite( QUIKEVAL_MUX_MODE_PIN, LOW );

  // Setup stack-specific hardware; this initializes the stack's
  // chip-select line
  stack.setup();
  // Call after all stack object setup() functions have been called;
  // this initializes SPI and timer hardware shared between stacks
  LTC6804_2_Stack::start();

  // Load array with configuration words generated from the supplied 
  // parameters
  stack.prepare_config(
    configData,
    PD_GPIO1 | PD_GPIO2,            // GPIO pulldown enable
    REF_ON,                         // REFON bit
    ADC_OPT_1,                      // ADCOPT bit
    // Use macros to generate codes corresponding to under- and over-voltage values
    UV_THRESHOLD_CODE(3.5),         // Undervoltage threshold
    OV_THRESHOLD_CODE(4.2),         // Overvoltage threshold
    D_CELL1 | D_CELL2,              // Cell discharge shunt enabe
    DISCHARGE_TIMEOUT_OFF           // Cell discharge timer
  );
  Serial.println( "\n\nPREPARED CONFIGURATION" );
  print_reg_group( configData, &regNames[0], LTC6804_NUM_CONFIG_WORDS_PER_REG, 1 );
  Serial.print( "\n\n" );

  // Attempt to write configuration data to all '6804 ICs on the stack
  if( !stack.config_all( NUM_ICS, IC_ADDRESSES, configData ) )
  {
    // Could not verify all ICs configured properly; halt SPI communications
    // and output a message to the user
    Serial.println( "Failed to verify IC configuration! Verify connections and software constants." );
    LTC6804_2_Stack::stop();
    while( true );
  } // end if
} // end setup

void loop()
{
  const char* regNames[] = {
    "CFG REG", 
    "CV REG A", "CV REG B", "CV REG C", "CV REG D", 
    "AUX REG A", "AUX REG B", 
    "STAT REG A", "STAT REG B" 
  };
  const char* pecNames[] = { 
    "CFG PEC", 
    "CV PEC A", "CV PEC B", "CV PEC C", "CV PEC D",
    "AUX PEC A", "AUX PEC B", 
    "STAT PEC A", "STAT PEC B" 
  };
  const char* cmpPecNames[] = { 
    "CMP CFG PEC", 
    "CMP CV PEC A", "CMP CV PEC B", "CMP CV PEC C", "CMP CV PEC D",
    "CMP AUX PEC A", "CMP AUX PEC B",
    "CMP STAT PEC A", "CMP STAT PEC B" 
  };
  
  uint16_t cfgDat[LTC6804_NUM_CONFIG_WORDS];
  uint16_t cfgPec, cfgCmpPec;

  uint16_t cvDat[LTC6804_NUM_CV_WORDS];
  uint16_t cvPec[LTC6804_NUM_CV_REGS], cvCmpPec[LTC6804_NUM_CV_REGS];
  float cv[LTC6804_NUM_CELLS];

  uint16_t auxDat[LTC6804_NUM_AUX_WORDS];
  uint16_t auxPec[LTC6804_NUM_AUX_REGS], auxCmpPec[LTC6804_NUM_AUX_REGS];
  float gpioVolt[LTC6804_NUM_GPIOS];
  float ref2Volt;

  uint16_t statDat[LTC6804_NUM_AUX_WORDS];
  uint16_t statPec[LTC6804_NUM_AUX_REGS], statCmpPec[LTC6804_NUM_AUX_REGS];
  float socVolt, intTemp, analogSuppVolt, digitalSuppVolt;
  uint16_t uvFlags, ovFlags;
  uint8_t muxFail, thermalShutdown;

  stack.start_cell_voltage_adc( BROADCAST, ADC_MODE_NORMAL, ADC_PERMIT_DISCHARGE_FALSE, ADC_CELL_CHANNEL_ALL );
  stack.start_auxillary_adc( BROADCAST, ADC_MODE_NORMAL, ADC_AUX_CHANNEL_ALL );
  stack.start_status_adc( BROADCAST, ADC_MODE_FAST, ADC_SELF_TEST_CHANNEL_ALL );

  delay( 3000 );

  for( uint8_t icIndex = 0; icIndex < NUM_ICS; icIndex++ )
  {
    Serial.println();
    Serial.println();
    Serial.print( "IC " );
    Serial.print( icIndex );
    Serial.print( " (ADDRESS " );
    Serial.print( IC_ADDRESSES[icIndex] );
    Serial.println( ")" );
    Serial.println( "-------------------------" );
    
    stack.read_config( IC_ADDRESSES[icIndex], cfgDat, &cfgPec );
    cfgCmpPec = crc15( cfgDat, LTC6804_NUM_CONFIG_WORDS_PER_REG );
    
    stack.read_cell_voltages( IC_ADDRESSES[icIndex], cvDat, cvPec );
    crc15_group( cvDat, cvCmpPec, LTC6804_NUM_CV_WORDS_PER_REG, LTC6804_NUM_CV_REGS );
    stack.extract_cell_voltages( cvDat, cv );
    
    stack.read_auxillary( IC_ADDRESSES[icIndex], auxDat, auxPec );
    crc15_group( auxDat, auxCmpPec, LTC6804_NUM_AUX_WORDS_PER_REG, LTC6804_NUM_AUX_REGS );
    stack.extract_auxillary( auxDat, gpioVolt, &ref2Volt );
    
    stack.read_status( IC_ADDRESSES[icIndex], statDat, statPec );
    crc15_group( statDat, statCmpPec, LTC6804_NUM_STAT_WORDS_PER_REG, LTC6804_NUM_STAT_REGS );
    stack.extract_status( statDat, &socVolt, &intTemp, &analogSuppVolt, &digitalSuppVolt, &uvFlags, &ovFlags, &muxFail, &thermalShutdown );
    
    if( !arrays_equal( &cfgPec, &cfgCmpPec, 1 ) )
    {
      Serial.println( "PEC Verification Failed!" );
    } // end if
    print_reg_group( cfgDat, &regNames[0], LTC6804_NUM_CONFIG_WORDS_PER_REG, 1 );
    print_reg_group( &cfgPec, &pecNames[0], 1, 1 );
    print_reg_group( &cfgCmpPec, &cmpPecNames[0], 1, 1 );
    Serial.println();
    
    if( !arrays_equal( cvPec, cvCmpPec, LTC6804_NUM_CV_REGS ) )
    {
      Serial.println( "PEC Verification Failed!" );
    } // end if
    print_reg_group( cvDat, &regNames[1], LTC6804_NUM_CV_WORDS_PER_REG, LTC6804_NUM_CV_REGS );
    print_reg_group( cvPec, &pecNames[1], 1, LTC6804_NUM_CV_REGS );
    print_reg_group( cvCmpPec, &cmpPecNames[1], 1, LTC6804_NUM_CV_REGS );
    Serial.println();
    
    Serial.println( "Cell Voltages:" );
    for( uint8_t i = 0; i < LTC6804_NUM_CELLS; i++ )
    {
      Serial.print( "Cell " );
      Serial.print( i + 1 );
      Serial.print( ": " );
      Serial.print( cv[i], 3 );
      Serial.println( " V" );
    } // end for
    Serial.println();
    
    if( !arrays_equal( auxPec, auxCmpPec, LTC6804_NUM_AUX_REGS ) )
    {
      Serial.println( "PEC Verification Failed!" );
    } // end if
    print_reg_group( auxDat, &regNames[5], LTC6804_NUM_AUX_WORDS_PER_REG, LTC6804_NUM_AUX_REGS );
    print_reg_group( auxPec, &pecNames[5], 1, LTC6804_NUM_AUX_REGS );
    print_reg_group( auxCmpPec, &cmpPecNames[5], 1, LTC6804_NUM_AUX_REGS );  
    Serial.println();
    
    Serial.println( "Auxillary Data:" );
    for( uint8_t i = 0; i < LTC6804_NUM_GPIOS; i++ )
    {
      Serial.print( "GPIO " );
      Serial.print( i + 1 );
      Serial.print( ": " );
      Serial.print( gpioVolt[i], 3 );
      Serial.println( " V" );
    } // end for
    Serial.print( "REF2: " );
    Serial.print( ref2Volt, 3 );
    Serial.println( " V" );
    Serial.println();
    
    if( !arrays_equal( statPec, statCmpPec, LTC6804_NUM_STAT_REGS ) )
    {
      Serial.println( "PEC Verification Failed!" );
    } // end if
    print_reg_group( statDat, &regNames[7], LTC6804_NUM_STAT_WORDS_PER_REG, LTC6804_NUM_STAT_REGS );
    print_reg_group( statPec, &pecNames[7], 1, LTC6804_NUM_STAT_REGS );
    print_reg_group( statCmpPec, &cmpPecNames[7], 1, LTC6804_NUM_STAT_REGS );  
    Serial.println();
    
    Serial.println( "Status Data:" );
    Serial.print( "SOC: " );
    Serial.print( socVolt, 3 );
    Serial.println( " V" );
    Serial.print( "Internal Temperature: " );
    Serial.print( intTemp, 3 );
    Serial.println( " Celsius" );
    Serial.print( "Analog Supply Voltage: " );
    Serial.print( analogSuppVolt, 3 );
    Serial.println( " V" );
    Serial.print( "Digital Supply Voltage: " );
    Serial.print( digitalSuppVolt, 3 );
    Serial.println( " V" );
    Serial.print( "Undervoltage Flags: " );
    Serial.println( uvFlags, BIN );
    Serial.print( "Overvoltage Flags: " );
    Serial.println( ovFlags, BIN );
    Serial.print( "MUXFAIL: " );
    Serial.println( muxFail, BIN );
    Serial.print( "Thermal Shutdown: " );
    Serial.println( thermalShutdown, BIN );
  } // end for
} // end loop

void print_reg_group( const uint16_t* const regDataPtr, const char* const regNames[], const uint8_t grpSize, const uint8_t numGrps )
{
  for( uint8_t grp = 0; grp < numGrps; grp++ )
  {
    Serial.print( regNames[grp] );
    Serial.print( ": " );
    
    for( uint8_t grpElement = grpSize * grp; grpElement < grpSize * (grp + 1); grpElement++ )
    {
      Serial.print( "0x" );
      Serial.print( regDataPtr[grpElement], HEX );
      Serial.print( " " );
    } // end for

    Serial.println();
  } // end for
} // end print_reg_group
