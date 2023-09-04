/* Copyright (c) 2023 Scott Rapson
 * MIT Licenced - see LICENCE for details.
 */

#ifndef MS5837_H
#define MS5837_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------

typedef enum {
    SENSOR_PRESSURE = 0,
    SENSOR_TEMPERATURE,
    NUM_SENSOR_FIELDS   // Should be last in enum
} MS5837_SELECT_SENSOR;

// End-user enum for selecting an oversample resolution
typedef enum {
    OSR_256 = 0,
    OSR_512,
    OSR_1024,
    OSR_2048,
    OSR_4096,
    OSR_8192,
} MS5837_ADC_OSR;

// Internal calibration values stored in PROM
// Read from CMD_READ_PROM_START to CMD_READ_PROM_END
typedef enum {
    C0_VERSION = 0,
    C1_PRESSURE_SENSITIVITY,
    C2_PRESSURE_OFFSET,
    C3_TEMP_PRESSURE_SENSITIVITY_COEFF,
    C4_TEMP_PRESSURE_OFFSET_COEFF,
    C5_TEMP_REFERENCE,
    C6_TEMP_COEFF,
    NUM_CALIBRATION_VARIABLES    // must be last enum value
} MS5837_CALIBRATION_VARIABLES;


// End-user I2C callback function signature
// address, command, buffer, number of bytes
typedef void (*user_i2c_cb_t)(uint8_t, uint8_t, uint8_t*, uint8_t);


typedef struct {
    user_i2c_cb_t user_write_fn;
    user_i2c_cb_t user_read_fn;
    uint8_t i2c_address;

    uint8_t variant;
    bool calibration_loaded;
    uint16_t calibration_data[NUM_CALIBRATION_VARIABLES];

    MS5837_SELECT_SENSOR last_conversion;
    uint32_t samples[2];
    int32_t measurements[2];
} ms5837_t;

// ---------------------------------------------------------------------

void ms5837_i2c_set_read_fn( ms5837_t *sensor, user_i2c_cb_t callback );

void ms5837_i2c_set_write_fn( ms5837_t *sensor, user_i2c_cb_t callback );

void ms5837_reset( ms5837_t *sensor );

bool ms5837_read_calibration_data( ms5837_t *sensor );

uint16_t ms5837_start_conversion( ms5837_t *sensor, MS5837_SELECT_SENSOR type, MS5837_ADC_OSR osr );

uint32_t ms5837_read_conversion( ms5837_t *sensor );

bool ms5837_calculate( ms5837_t *sensor );

float ms5837_temperature_celcius( ms5837_t *sensor );

float ms5837_temperature_fahrenheit( ms5837_t *sensor );

float ms5837_pressure_bar( ms5837_t *sensor );

float ms5837_pressure_mbar( ms5837_t *sensor );

float ms5837_pressure_atm( ms5837_t *sensor );

float ms5837_pressure_pascal( ms5837_t *sensor );

// ---------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif //end MS5837_H