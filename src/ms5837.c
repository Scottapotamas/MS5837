/* Copyright (c) 2023 Scott Rapson
 * MIT Licenced - see LICENCE for details.
 */

#include "ms5837.h"


// Stored in PROM word 0
#define MS5837_ID_02BA01 (0x00)
#define MS5837_ID_02BA21 (0x15)
#define MS5837_ID_30BA26 (0x1A)

typedef enum {
	CMD_RESET = 0x1E,
	CMD_READ = 0x00,
	CMD_READ_PROM_START = 0xA0,
	CMD_READ_PROM_END = 0xAE,

	CMD_PRESSURE_OSR_BASE = 0x40,	// OSR256
	// other OSR requests

	CMD_TEMPERATURE_OSR_BASE = 0x50, // OSR256
	// other OSR requests
} MS5837_COMMANDS;

// End-user enum for selecting an oversample resolution
typedef enum {
	OSR_256 = 0,
	OSR_512,
	OSR_1024,
	OSR_2048,
	OSR_4096,
	OSR_8192,
} MS5837_ADC_OSR;

// Structure for cleanly handling OSR and conversion duration values
typedef struct {
	uint8_t offset;			// Address offset from base for this OSR level
	uint16_t duration_us;	// Maximum duration for ADC conversions
} adc_osr_properties_t;

adc_osr_properties_t adc_osr_settings[] = {
	{ .offset = 0x00, .duration_us = 560 },
	{ .offset = 0x02, .duration_us = 1100 },
	{ .offset = 0x04, .duration_us = 2170 },
	{ .offset = 0x06, .duration_us = 4320 },
	{ .offset = 0x08, .duration_us = 8610 },
	{ .offset = 0x0A, .duration_us = 17200 },
};

// These need to be added to the Pressure OSR base or Temp OSR base addresses
// used as a reference



// Internal calibration values stored in PROM
// Read from CMD_READ_PROM_START to CMD_READ_PROM_END
typedef enum {
	C0_VERSION,
	C1_PRESSURE_SENSITIVITY,
	C2_PRESSURE_OFFSET,
	C3_TEMP_PRESSURE_SENSITIVITY_COEFF,
	C4_TEMP_PRESSURE_OFFSET_COEFF,
	C5_TEMP_REFERENCE,
	C6_TEMP_COEFF,
	NUM_CALIBRATION_VARIABLES	// must be last enum value
} MS5837_CALIBRATION_VARIABLES;


typedef struct {
	// I2C read/write callbacks?

	uint8_t i2c_address;

	bool calibration_loaded;
	uint16_t calibration_data[NUM_CALIBRATION_VARIABLES];

} ms5837_t;




void ms5837_i2c_read( ms5837_t *sensor, uint8_t address, uint8_t bytes )
{

}

void ms5837_i2c_write( ms5837_t *sensor, uint8_t address, uint8_t data )
{
	
}

void ms5837_reset( ms5837_t *sensor )
{

}

void ms5837_read_calibration_data( ms5837_t *sensor )
{
	// Read the 7 16-bit values from PROM
	for( uint8_t i = 0; i < NUM_CALIBRATION_VARIABLES; i++ )
	{
		ms5837_i2c_read( sensor, CMD_READ_PROM_START+(i*2), 2 );

		sensor->calibration_data[i] = (_i2cPort->read() << 8);	// upper byte
		sensor->calibration_data[i] |= _i2cPort->read()			// lower byte
	}

	// Validate CRC
    uint8_t crc_rx = sensor->calibration_data[C0_VERSION] >> 12;
    uint8_t crc_calc = crc4( &calibration_data, sizeof(calibration_data) );

    sensor->calibration_loaded = ( crc_rx == crc_calc );

    // Check the sensor version
    uint8_t version = (sensor->calibration_data[C0_VERSION] >> 5) & 0x7F;
}


// Starts an ADC conversion, returns the number of microseconds until data is ready
// If invalid/error, 0 will be returned
uint16_t ms5837_start_conversion( ms5837_t *sensor,  )
{
    ms5837_i2c_write( sensor, );

    return 0;
}

void ms5837_read_conversion( ms5837_t *sensor )
{

}



// Read sensor data
// Pressure (D1) is 24-bit unsigned
// Temperature (D2) is 24-bit unsigned


// Calculations for First Order 

// deltaTemp, 25-bit signed
// dT = D2 - T_REF = D2 - C5 * 2^8

// Actual Temperature - 41-bit signed
// TEMP = 20°C + dT * TEMPSENS = 2000 + dT * C6 / 2^23

// Pressure Offset - 41-bit signed
// OFF = OFF_T1 + TCO * dT = C2 * 2^17 + (C4 * dT ) / 2^6

// Pressure Sensitivity at actual temp - 41-bit signed
// SENS = SENS T1 + TCS * dT = C1 * 2^16 + (C3 * dT ) / 2^7

// Pressure (temperature compensated) - 58-bit signed
// P = D1 * SENS - OFF = (D1 * SENS / 2 21 - OFF) / 2^15

// Calculations for Second Order Compensation
// Assumes first order values usable

// If temperature is under 20C, first
// Ti = 11 * dT^2 / 2^35
// OFFi = 31 * (TEMP - 2000)^2 / 2^3
// SENSi = 63 * (TEMP - 2000)^2 / 2^5

// Then calc 2nd order terms
// OFF2 = OFF - OFFi
// SENS2 = SENS - SENSi

// TEMP2 = (TEMP - Ti)/100		degC
// P2 = ( (D1*SENS2 / 2^21 - OFF2 ) / 2^15 ) / 100    milibar


// n_prom defined as 8x unsigned int (n_prom[8])
unsigned char crc4( unsigned int n_prom[] ) 
{
	int cnt; // simple counter
	unsigned int n_rem=0; // crc remainder
	unsigned char n_bit;

	n_prom[0] = ((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
	n_prom[7] = 0; 						// Subsidiary value, set to 0
	
	for(cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
	{ 
		// choose LSB or MSB
		if (cnt%2==1) n_rem ^= (unsigned short)
		{
			((n_prom[cnt>>1]) & 0x00FF);
		}
		else 
		{
			n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--)
		{
			if (n_rem & (0x8000))
			{
				n_rem = (n_rem << 1) ^ 0x3000;
			}
			else
			{
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code

	return (n_rem ^ 0x00);
}