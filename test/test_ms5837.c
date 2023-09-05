#include "unity.h"
#include <string.h>

// MODULE UNDER TEST
#include "ms5837.h"

// DEFINITIONS 
 
// PRIVATE TYPES
 
// PRIVATE DATA
ms5837_t sensor = { 0 };

// PRIVATE FUNCTIONS
void mock_write_function( uint8_t address, uint8_t command, uint8_t *data, uint8_t num_bytes );
void mock_read_function( uint8_t address, uint8_t command, uint8_t *data, uint8_t num_bytes );
 
 
void mock_write_function( uint8_t address, uint8_t command, uint8_t *data, uint8_t num_bytes )
{
    printf("Write %d bytes to %d\n", num_bytes, address);
}

void mock_read_function( uint8_t address, uint8_t command, uint8_t *data, uint8_t num_bytes )
{
    printf("Read %d bytes from: %d\n", num_bytes, address);
}

// SETUP, TEARDOWN
void setUp(void)
{
    memset( &sensor, 0, sizeof(sensor) );
    sensor.user_write_fn = &mock_write_function;
    sensor.user_read_fn = &mock_read_function;
    sensor.i2c_address = 0x76;  // only supported address

}
 
void tearDown(void)
{

}

// TESTS

void test_conversions_02BA( void )
{   
    // Force load example data from the MS5837-02BA datasheet tables
    sensor.calibration_data[C0_VERSION] = 0;

    sensor.calibration_data[C1_PRESSURE_SENSITIVITY]            = 46372;
    sensor.calibration_data[C2_PRESSURE_OFFSET]                 = 43981;
    sensor.calibration_data[C3_TEMP_PRESSURE_SENSITIVITY_COEFF] = 29059;
    sensor.calibration_data[C4_TEMP_PRESSURE_OFFSET_COEFF]      = 27842;
    sensor.calibration_data[C5_TEMP_REFERENCE]                  = 31553;
    sensor.calibration_data[C6_TEMP_COEFF]                      = 28165;

    sensor.variant = 0; // TODO: this should be set more carefully once multi-versions are supported
    sensor.calibration_loaded = true;

    sensor.samples[SENSOR_PRESSURE]     = 6465444;
    sensor.samples[SENSOR_TEMPERATURE]  = 8077636;

    // Run the compensation calculations
    ms5837_calculate( &sensor );
    
    // 20C and 1100.02 mbar are datasheet values after first order compensation

    // Library performs second order compensation, but low-temperature compensation
    // shouldn't affect the value
    TEST_ASSERT_EQUAL_INT( 2000, sensor->measurements[SENSOR_TEMPERATURE] );
    TEST_ASSERT_EQUAL_INT( 110002, sensor->measurements[SENSOR_PRESSURE] );
}

// -------------------------------------------------------------------

void test_output_celcius( void )
{   
    // Force load a compensated/converted value
    sensor->measurements[SENSOR_TEMPERATURE] = 2000;
    TEST_ASSERT_EQUAL_FLOAT( 20.0f, ms5837_temperature_celcius(&sensor) );
}

void test_output_fahrenheit( void )
{   
    sensor->measurements[SENSOR_TEMPERATURE] = 2000;
    TEST_ASSERT_EQUAL_FLOAT( 68.0f, ms5837_temperature_fahrenheit(&sensor) );
}

void test_output_pascal( void )
{   
    // Force load the datasheet example value
    sensor->measurements[SENSOR_PRESSURE] = 110002;
    TEST_ASSERT_EQUAL_FLOAT( 110002.0f, ms5837_pressure_pascal(&sensor) );
}

void test_output_bar( void )
{
    sensor->measurements[SENSOR_PRESSURE] = 110002;
    TEST_ASSERT_EQUAL_FLOAT( 1.10002f, ms5837_pressure_bar(&sensor) );
}

void test_output_mbar( void )
{
    sensor->measurements[SENSOR_PRESSURE] = 110002;
    TEST_ASSERT_EQUAL_FLOAT( 1100.02f, ms5837_pressure_mbar(&sensor) );
}

void test_output_atm( void )
{
    sensor->measurements[SENSOR_PRESSURE] = 110002;
    TEST_ASSERT_EQUAL_FLOAT( 1.08563533f, ms5837_pressure_atm(&sensor) );
}

