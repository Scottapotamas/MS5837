#include <Wire.h>
#include <ms5837.h>

ms5837_t sensor = { 0 };
bool new_data_ready = false;

void i2c_read( uint8_t address, uint8_t command, uint8_t *data, uint8_t num_bytes );
void i2c_write( uint8_t address, uint8_t command, uint8_t *data, uint8_t num_bytes );

// -------------------------------------------------------------------------

// The different SM states to handle sampling the sensor
typedef enum
{
  STATE_STARTUP = 0,
  STATE_WAIT_RESET,
  STATE_ERROR,
  STATE_IDLE,
  STATE_REQUEST_PRESSURE,
  STATE_READ_PRESSURE,
  STATE_REQUEST_TEMPERATURE,
  STATE_READ_TEMPERATURE,
  STATE_CONVERSION
} SENSOR_STATES;

SENSOR_STATES handler_state = STATE_STARTUP;
uint32_t handler_timestamp  = 0;
uint32_t sensor_wait_us     = 0;

// SM Configuration settings
bool sensor_enabled               = false;    // allows sampling state-machine to pause
MS5837_ADC_OSR sensor_p_osr       = OSR_8192; // Maximum OSR - OSR_256, OSR_512, etc supported
MS5837_ADC_OSR sensor_t_osr       = OSR_2048; // Reduce sampling time with lower temperature OSR 
const uint32_t startup_wait_ms    = 10;       // Wait after reset command before reading constants
const uint32_t sample_interval_ms = 250;      // period between samples, set to 0 for free-running

void sensor_handler( void );

// -------------------------------------------------------------------------

void setup( void )
{
  Serial.begin(115200);

  Wire.begin();
  ms5837_i2c_set_read_fn( &sensor,  i2c_read );
  ms5837_i2c_set_write_fn( &sensor, i2c_write );

  Serial.println("MS5837 Non-Blocking State Machine Demo");
  sensor_enabled = true;
}

void loop( void )
{
  sensor_handler();

  if( new_data_ready )
  {
    // Query converted values from the library
    float pressure    = ms5837_pressure_mbar( &sensor );
    float temperature = ms5837_temperature_celcius( &sensor );

    // Report values via UART
    Serial.print("Pressure (mbar) = ");
    Serial.println(pressure);

    Serial.print("Temperature C = ");
    Serial.println(temperature);

    Serial.println("");

    new_data_ready = false;
  }

  // Perform normal application tasks
  // A simple non-blocking blink for demo purposes

}

// User configurable I2C calls to allow for wide portability.
void i2c_read(uint8_t address, uint8_t command, uint8_t *data, uint8_t num_bytes)
{
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.endTransmission();

  Wire.requestFrom(address, num_bytes);
  for( uint8_t i = 0; i < num_bytes; i++ )
  {
    data[i] = Wire.read();  
  }
}

void i2c_write(uint8_t address, uint8_t command, uint8_t *data, uint8_t num_bytes)
{
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.endTransmission();
}

// -------------------------------------------------------------------------

// Implements a simple state-machine for non-blocking MS5837 sampling
void sensor_handler( void )
{
  switch( handler_state )
  {
    case STATE_STARTUP:
      ms5837_reset( &sensor );

      handler_timestamp = millis();
      handler_state = STATE_WAIT_RESET;
    break;

    case STATE_WAIT_RESET:
      if( millis() - handler_timestamp >= startup_wait_ms )
      {
        //Retrieve calibration constants for conversion math.
        ms5837_read_calibration_data( &sensor );

        if( sensor.calibration_loaded )
        {
          handler_state = STATE_IDLE;
        }
        else
        {
          handler_state = STATE_ERROR;
        }

        handler_timestamp = millis();
      }
    break;

    case STATE_ERROR:
      // TODO: consider more involved failure recovery processes on platforms
      //       with sophisticated peripherals/designs (i.e. power cycle etc)
      handler_state = STATE_STARTUP;
    break;

    case STATE_IDLE:
      if( sensor_enabled )
      {
        // Is it time to sample the sensor?
        if( millis() - handler_timestamp >= sample_interval_ms )
        {
          handler_state = STATE_REQUEST_PRESSURE;
        }
      }
    break;

    case STATE_REQUEST_PRESSURE:
      sensor_wait_us = ms5837_start_conversion( &sensor, SENSOR_PRESSURE, sensor_p_osr );

      if( sensor_wait_us == 0)
      {
        handler_state = STATE_ERROR;
      }
      else
      {
        handler_state = STATE_READ_PRESSURE;
      }

      handler_timestamp = millis();
    break;

    case STATE_READ_PRESSURE:
      if( millis() - handler_timestamp >= (sensor_wait_us/1000)+1 )
      {
        if( ms5837_read_conversion( &sensor ) )
        {
          handler_state = STATE_REQUEST_TEMPERATURE;
        }
        else
        {
          handler_state = STATE_ERROR;
        }
      }
    break;

    case STATE_REQUEST_TEMPERATURE:
      sensor_wait_us = ms5837_start_conversion( &sensor, SENSOR_TEMPERATURE, sensor_t_osr );

      if( sensor_wait_us == 0)
      {
        handler_state = STATE_ERROR;
      }
      else
      {
        handler_state = STATE_READ_TEMPERATURE;
      }

      handler_timestamp = millis();
    break;

    case STATE_READ_TEMPERATURE:
      if( millis() - handler_timestamp >= (sensor_wait_us/1000)+1 )
      {
        if( ms5837_read_conversion( &sensor ) )
        {
          handler_state = STATE_CONVERSION;
        }
        else
        {
          handler_state = STATE_ERROR;
        }
      }
    break;

    case STATE_CONVERSION:
      if( ms5837_calculate( &sensor ) )
      {
        // Sensor measurements were converted successfully
        // This is a good time to flag that new measurements are ready for use, or similar?

        // This example sets a 'end-user' bool which is polled in the main loop
        new_data_ready = true;

        // Go back to the start for another aquisition cycle
        handler_state = STATE_IDLE;
      }
      else
      {
        handler_state = STATE_ERROR;
      }

      handler_timestamp = millis();
    break;

    default:
      // Not supposed to get here...
      handler_state = STATE_STARTUP;
    break;
  }
}
