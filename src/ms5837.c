/* Copyright (c) 2023 Scott Rapson
 * MIT Licenced - see LICENCE for details.
 */

#include "ms5837.h"


// Stored in PROM word 0
#define MS5837_ID_02BA01 (0x00)
#define MS5837_ID_02BA21 (0x15)
#define MS5837_ID_30BA26 (0x1A)
// TODO cleanup/enumify this

#define MS5837_ADDR (0x76)

typedef enum {
    CMD_RESET = 0x1E,
    CMD_READ = 0x00,
    CMD_READ_PROM_START = 0xA0,
    CMD_READ_PROM_END = 0xAE,

    CMD_PRESSURE_OSR_BASE = 0x40,    // OSR256
    // other OSR requests
    CMD_TEMPERATURE_OSR_BASE = 0x50, // OSR256
    // other OSR requests
} MS5837_COMMANDS;

// Structure for cleanly handling OSR and conversion duration values
typedef struct {
    uint8_t offset;            // Address offset from base for this OSR level
    uint16_t duration_us;    // Maximum duration for ADC conversions
} adc_osr_properties_t;

// The offsets need to be added to the Pressure OSR base or Temp OSR base addresses
// used as a reference
adc_osr_properties_t adc_osr_settings[] = {
    { .offset = 0x00, .duration_us = 560 },
    { .offset = 0x02, .duration_us = 1100 },
    { .offset = 0x04, .duration_us = 2170 },
    { .offset = 0x06, .duration_us = 4320 },
    { .offset = 0x08, .duration_us = 8610 },
    { .offset = 0x0A, .duration_us = 17200 },
};

// ---------------------------------------------------------------------

uint8_t crc4( uint16_t n_prom[], uint8_t len );

void ms5837_i2c_read( ms5837_t *sensor, uint8_t command, uint8_t *data, uint8_t num_bytes );

void ms5837_i2c_write( ms5837_t *sensor, uint8_t command, uint8_t data );

// ---------------------------------------------------------------------

void ms5837_i2c_set_read_fn( ms5837_t *sensor, user_i2c_cb_t callback )
{
    if( sensor && callback )
    {
        sensor->user_read_fn = callback;
    }
}

void ms5837_i2c_set_write_fn( ms5837_t *sensor, user_i2c_cb_t callback )
{
    if( sensor && callback )
    {
        sensor->user_write_fn = callback;
    }
}

void ms5837_i2c_read( ms5837_t *sensor, uint8_t command, uint8_t *data, uint8_t num_bytes )
{
    if( sensor->user_read_fn && sensor->user_write_fn )
    {
        sensor->user_read_fn( MS5837_ADDR, command, data, num_bytes );
    }
}

void ms5837_i2c_write( ms5837_t *sensor, uint8_t command, uint8_t data )
{
    if( sensor->user_write_fn )
    {
        sensor->user_write_fn( MS5837_ADDR, command, data, 0 );
    }
}

// ---------------------------------------------------------------------

// Perform a software reset
void ms5837_reset( ms5837_t *sensor )
{
    ms5837_i2c_write( sensor, CMD_RESET, 0 );
}

// Requests PROM data from sensor and stores it in the structure
// Returns true if successful
bool ms5837_read_calibration_data( ms5837_t *sensor )
{
    if( !sensor )
    {
        return false;
    }

    // Read the 7 16-bit values from PROM
    for( uint8_t i = 0; i < NUM_CALIBRATION_VARIABLES; i++ )
    {
        uint8_t buffer[2] = { 0 };
        ms5837_i2c_read( sensor, CMD_READ_PROM_START+(i*2), &buffer, 2 );

        sensor->calibration_data[i] = (buffer[0] << 8);    // upper byte
        sensor->calibration_data[i] |= buffer[1];          // lower byte
    }

    // Validate CRC
    uint8_t crc_rx = sensor->calibration_data[C0_VERSION] >> 12;
    uint8_t crc_calc = crc4( &sensor->calibration_data, sizeof(sensor->calibration_data) );

    sensor->calibration_loaded = ( crc_rx == crc_calc );

    // Check the sensor version
    uint8_t version = (sensor->calibration_data[C0_VERSION] >> 5) & 0x7F;
    sensor->variant = version;  // TODO map to an enum here

    return sensor->calibration_loaded;
}

// Starts an ADC conversion, returns the number of microseconds until data is ready
// If invalid/error, 0 will be returned
uint16_t ms5837_start_conversion( ms5837_t *sensor, MS5837_SELECT_SENSOR type, MS5837_ADC_OSR osr )
{
    // TODO ptr valid and range for input args

    switch( type )
    {
        case SENSOR_PRESSURE:
            ms5837_i2c_write( sensor, CMD_PRESSURE_OSR_BASE + adc_osr_settings[osr].offset, 0 );
            sensor->last_conversion = SENSOR_PRESSURE;
        break;

        case SENSOR_TEMPERATURE:
            ms5837_i2c_write( sensor, CMD_TEMPERATURE_OSR_BASE + adc_osr_settings[osr].offset, 0 );
            sensor->last_conversion = SENSOR_TEMPERATURE;
        break; 

        default:
            sensor->last_conversion = 999; // TODO add an invalid entry
            return 0;
    }

    return adc_osr_settings[osr].duration_us;
}

// Read sensor data, 24-bit unsigned value
// Then store it in the object
// Returns 0 for errors, the conversion value if OK
uint32_t ms5837_read_conversion( ms5837_t *sensor )
{
    if( !sensor || sensor->last_conversion >= NUM_SENSOR_FIELDS )
    {
        return 0;
    }

    uint8_t value[3] = { 0 };
    ms5837_i2c_read( sensor, CMD_READ, &value, 3 );

    uint32_t conversion = 0;
    conversion = value[0];
    conversion = (conversion << 8) | value[1];
    conversion = (conversion << 8) | value[2];

    sensor->samples[sensor->last_conversion] = conversion;
    sensor->last_conversion = NUM_SENSOR_FIELDS; // invalidate

    return conversion;
}

bool ms5837_calculate( ms5837_t *sensor )
{
    if( !sensor || !sensor->calibration_loaded )
    {
        return false;
    }

    if( !sensor->samples[0] || !sensor->samples[1] )
    {
        return false;
    }

    uint32_t sample_pressure = sensor->samples[SENSOR_PRESSURE];
    uint32_t sample_temperature = sensor->samples[SENSOR_TEMPERATURE];

    // First Order conversion 

    // deltaTemp, 25-bit signed
    // dT = D2 - TREF 
    //    = D2 - C5 * 2^8 
    int32_t delta_temp = sample_temperature - (uint32_t)sensor->calibration_data[C5_TEMP_REFERENCE] * (1 << 8);

    // Actual Temperature - 41-bit signed
    // TEMP = 20°C + dT * TEMPSENS 
    //      = 2000 + dT * C6 / 2^23
    int32_t temperature = 2000UL + delta_temp * (int64_t)sensor->calibration_data[C6_TEMP_COEFF] / ((uint32_t)1 << 23);

    // Pressure Offset - 41-bit signed
    // OFF = OFF_T1 + TCO * dT 
    //     = C2 * 2^17 + (C4 * dT ) / 2^6
    int64_t pressure_offset = ((int64_t)sensor->calibration_data[C2_PRESSURE_OFFSET] * ((uint32_t)1 << 17)) 
                              + ((int64_t)sensor->calibration_data[C4_TEMP_PRESSURE_OFFSET_COEFF] * delta_temp)/(1 << 6);

    // Pressure Sensitivity at actual temp - 41-bit signed
    // SENS = SENS T1 + TCS * dT 
    //      = C1 * 2^16 + (C3 * dT ) / 2^7
    int64_t pressure_sensitivity = (int64_t)sensor->calibration_data[C1_PRESSURE_SENSITIVITY] * ((uint32_t)1 << 16) 
                                   + ( (int64_t)sensor->calibration_data[C3_TEMP_PRESSURE_SENSITIVITY_COEFF] * delta_temp)/((uint32_t)1 << 7); 

    // Pressure (temperature compensated) - 58-bit signed
    // P = D1 * SENS - OFF 
    //   = (D1 * SENS / 2^21 - OFF) / 2^15
    int32_t pressure = ( sample_pressure * (pressure_sensitivity / ((uint32_t)1 << 21)) - pressure_offset ) / ((uint32_t)1 << 15);

    // Calculations for Second Order Compensation

    // Low temp compensation (<20C)
    int32_t temp_i = 0;
    int32_t offset_i = 0;
    int32_t sensitivity_i = 0;

    if( (temperature / 100U) < 20 )
    {
        // Ti = 11 * dT^2 / 2^35
         temp_i = ( 11 * (int64_t)delta_temp*(int64_t)delta_temp ) / ((uint64_t)1 << 35);
        
        // OFFi = 31 * (TEMP - 2000)^2 / 2^3
         offset_i = ( 31 * (temperature-2000)*(temperature-2000) ) / (1 << 3);

        // SENSi = 63 * (TEMP - 2000)^2 / 2^5
         sensitivity_i = ( 63 * (temperature-2000)*(temperature-2000) ) / (1 << 5);
    }

    // Calculate 2nd order terms
    // OFF2 = OFF - OFFi
    int64_t offset_2 = pressure_offset - offset_i;

    // SENS2 = SENS - SENSi
    int64_t sensitivity_2 = pressure_sensitivity - sensitivity_i;

    // TEMP2 = (TEMP - Ti)/100      degC
    int32_t temperature_2 = ( temperature - temp_i );

    // P2 = ( (D1*SENS2 / 2^21 - OFF2 ) / 2^15 ) / 100    milibar
    int32_t pressure_2 = ( ( (sample_pressure * sensitivity_2) / ((uint32_t)1 << 21) - offset_2 ) / ((uint32_t)1 << 15) );

    // Store the results in the sensor structure
    sensor->measurements[SENSOR_PRESSURE] = pressure_2;
    sensor->measurements[SENSOR_TEMPERATURE] = temperature_2;

    // Wipe conversion value fields
    sensor->samples[SENSOR_PRESSURE] = 0;
    sensor->samples[SENSOR_TEMPERATURE] = 0;

    return true;
}

// ---------------------------------------------------------------------

float ms5837_temperature_celcius( ms5837_t *sensor )
{
    return sensor->measurements[SENSOR_TEMPERATURE]/100.0f;
}

float ms5837_pressure_mbar( ms5837_t *sensor )
{
    return sensor->measurements[SENSOR_PRESSURE]/100.0f;
}

// ---------------------------------------------------------------------

// RROM is 7 unsigned int16 values for 112-bits
uint8_t crc4( uint16_t n_prom[], uint8_t len ) 
{
    uint16_t crc_rem = 0; // CRC remainder

    n_prom[0] = ((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
    n_prom[7] = 0;                      // Subsidiary value, set to 0
    
    for( uint8_t byte = 0; byte < 16; byte++ )
    {
        // choose LSB or MSB
        if( byte % 2 == 1 ) 
        {
            crc_rem ^= (unsigned short)(n_prom[byte >> 1] & 0x00FF);
        }
        else 
        {
            crc_rem ^= (unsigned short)(n_prom[byte >>1 ] >> 8);
        }

        for( uint8_t n_bit = 8; n_bit > 0; n_bit-- )
        {
            if (crc_rem & (0x8000))
            {
                crc_rem = (crc_rem << 1) ^ 0x3000;
            }
            else
            {
                crc_rem = (crc_rem << 1);
            }
        }
    }

    crc_rem = ((crc_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code

    return (crc_rem ^ 0x00);
}