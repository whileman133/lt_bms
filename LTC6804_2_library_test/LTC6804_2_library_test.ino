// LTC6804_2_library_test.ino
//
// Written by Wes Hileman on 6 June 2016
// U. of Colorado, Colorado Springs
//
// Sketch to test the functionality of the
// LTC6804_2_Stack library.

#include <Linduino.h>
#include <LTC6804_2_Stack.h>

LTC6804_2_Stack stack( QUIKEVAL_CS );

void setup() 
{
  Serial.begin( 9600 );
  
  // Enable QuikEval SPI
  pinMode( QUIKEVAL_MUX_MODE_PIN, OUTPUT );
  digitalWrite( QUIKEVAL_MUX_MODE_PIN, LOW );
  
  stack.setup();
  // Call after all stack object setup() functions have been called.
  LTC6804_2_Stack::start();  
} // end setup

void loop() 
{
  static const char* regNames[] = {
    "CFG REG", 
    "CV REG A", "CV REG B", "CV REG C", "CV REG D", 
    "AUX REG A", "AUX REG B", 
    "STAT REG A", "STAT REG B" 
  };
  static const char* pecNames[] = { 
    "CFG PEC", 
    "CV PEC A", "CV PEC B", "CV PEC C", "CV PEC D",
    "AUX PEC A", "AUX PEC B", 
    "STAT PEC A", "STAT PEC B" 
  };
  static const char* cmpPecNames[] = { 
    "CMP CFG PEC", 
    "CMP CV PEC A", "CMP CV PEC B", "CMP CV PEC C", "CMP CV PEC D",
    "CMP AUX PEC A", "CMP AUX PEC B",
    "CMP STAT PEC A", "CMP STAT PEC B" 
  };
  
  static const uint16_t cfg[LTC6804_NUM_CONFIG_WORDS] = { 0x0D, 0x00, 0x00 };
  
  uint16_t cfgDat[LTC6804_NUM_CONFIG_WORDS];
  uint16_t cfgPec, cfgCmpPec;

  uint16_t cvDat[LTC6804_NUM_CV_WORDS];
  float cv[LTC6804_NUM_CV_WORDS];
  uint16_t cvPec[LTC6804_NUM_CV_REGS], cvCmpPec[LTC6804_NUM_CV_REGS];

  uint16_t auxDat[LTC6804_NUM_AUX_WORDS];
  uint16_t auxPec[LTC6804_NUM_AUX_REGS], auxCmpPec[LTC6804_NUM_AUX_REGS];

  uint16_t statDat[LTC6804_NUM_AUX_WORDS];
  uint16_t statPec[LTC6804_NUM_AUX_REGS], statCmpPec[LTC6804_NUM_AUX_REGS];

  stack.start_cell_voltage_adc( BROADCAST, ADC_MODE_NORMAL, ADC_PERMIT_DISCHARGE_FALSE, ADC_CELL_CHANNEL_ALL );
  stack.start_auxillary_adc( BROADCAST, ADC_MODE_NORMAL, ADC_AUX_CHANNEL_ALL );
  stack.start_status_adc( BROADCAST, ADC_MODE_FAST, ADC_SELF_TEST_CHANNEL_ALL );

  delay( 3000 );

  stack.write_config( BROADCAST, cfg );
  stack.read_config( 0, cfgDat, &cfgPec );
  cfgCmpPec = crc15( cfgDat, LTC6804_NUM_CONFIG_WORDS_PER_REG );

  if( !arrays_equal( &cfgPec, &cfgCmpPec, 1 ) )
  {
    Serial.println( "PEC Verification Failed!" );
  } // end if
  print_reg_group( cfgDat, &regNames[0], LTC6804_NUM_CONFIG_WORDS_PER_REG, 1 );
  print_reg_group( &cfgPec, &pecNames[0], 1, 1 );
  print_reg_group( &cfgCmpPec, &cmpPecNames[0], 1, 1 );
  Serial.println();

  stack.read_cell_voltages( 0, cvDat, cvPec );
  crc15_group( cvDat, cvCmpPec, LTC6804_NUM_CV_WORDS_PER_REG, LTC6804_NUM_CV_REGS );
  stack.extract_cell_voltages( cvDat, cv );

  if( !arrays_equal( cvPec, cvCmpPec, LTC6804_NUM_CV_REGS ) )
  {
    Serial.println( "PEC Verification Failed!" );
  } // end if
  print_reg_group( cvDat, &regNames[1], LTC6804_NUM_CV_WORDS_PER_REG, LTC6804_NUM_CV_REGS );
  print_reg_group( cvPec, &pecNames[1], 1, LTC6804_NUM_CV_REGS );
  print_reg_group( cvCmpPec, &cmpPecNames[1], 1, LTC6804_NUM_CV_REGS );
  Serial.println();

  Serial.println( "Cell Voltages:" );
  for( uint8_t i = 0; i < LTC6804_NUM_CV_WORDS; i++ )
  {
    Serial.print( "Cell " );
    Serial.print( i + 1 );
    Serial.print( ": " );
    Serial.print( cv[i], 3 );
    Serial.println( " V" );
  } // end for
  Serial.println();

  stack.read_auxillary( 0, auxDat, auxPec );
  crc15_group( auxDat, auxCmpPec, LTC6804_NUM_AUX_WORDS_PER_REG, LTC6804_NUM_AUX_REGS );

  if( !arrays_equal( auxPec, auxCmpPec, LTC6804_NUM_AUX_REGS ) )
  {
    Serial.println( "PEC Verification Failed!" );
  } // end if
  print_reg_group( auxDat, &regNames[5], LTC6804_NUM_AUX_WORDS_PER_REG, LTC6804_NUM_AUX_REGS );
  print_reg_group( auxPec, &pecNames[5], 1, LTC6804_NUM_AUX_REGS );
  print_reg_group( auxCmpPec, &cmpPecNames[5], 1, LTC6804_NUM_AUX_REGS );  
  Serial.println();

  stack.read_status( 0, statDat, statPec );
  crc15_group( statDat, statCmpPec, LTC6804_NUM_STAT_WORDS_PER_REG, LTC6804_NUM_STAT_REGS );

  if( !arrays_equal( statPec, statCmpPec, LTC6804_NUM_STAT_REGS ) )
  {
    Serial.println( "PEC Verification Failed!" );
  } // end if
  print_reg_group( statDat, &regNames[7], LTC6804_NUM_STAT_WORDS_PER_REG, LTC6804_NUM_STAT_REGS );
  print_reg_group( statPec, &pecNames[7], 1, LTC6804_NUM_STAT_REGS );
  print_reg_group( statCmpPec, &cmpPecNames[7], 1, LTC6804_NUM_STAT_REGS );  
  Serial.println();
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
