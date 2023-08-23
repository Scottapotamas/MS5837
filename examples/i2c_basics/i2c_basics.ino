#include <Wire.h>
#include <ms5837.h>

ms5837_t sensor = { 0 };

void i2c_read( uint8_t address, uint8_t command, uint8_t *data, uint8_t num_bytes );
void i2c_write( uint8_t address, uint8_t command, uint8_t *data, uint8_t num_bytes );

void setup()
{
  Serial.begin(115200);

  Wire.begin();

  ms5837_i2c_set_read_fn( &sensor, i2c_read );
  ms5837_i2c_set_write_fn( &sensor, i2c_write );

  //Retrieve calibration constants for conversion math.
  ms5837_reset( &sensor );
  delay(10);  // TODO: work out how long this should be
  ms5837_read_calibration_data( &sensor );
}

void loop()
{
  uint16_t wait_us = 0;

  // Controllable oversampling levels for higher/lower resolution
  // OSR_256 
  // OSR_512 
  // OSR_1024
  // OSR_2048
  // OSR_4096
  // OSR_8192

  // When requesting a measurement, the function returns a microsecond duration
  // when we can safely attempt to read the value.

  // Request a pressure reading
  wait_us = ms5837_start_conversion( &sensor, SENSOR_PRESSURE, OSR_8192 );
  delayMicroseconds(wait_us);
  ms5837_read_conversion( &sensor );

  wait_us = ms5837_start_conversion( &sensor, SENSOR_TEMPERATURE, OSR_8192 );
  delayMicroseconds(wait_us);
  ms5837_read_conversion( &sensor );

  // Once pressure and temp are sampled, run the conversion function
  ms5837_calculate( &sensor );
    
  // The values are now ready to use.
  // TODO: helper functions for different units

  // Report values via UART
  Serial.print("Temperature C = ");
  Serial.println(sensor.measurements[1]);
    
  Serial.print("Pressure (mbar) = ");
  Serial.println(sensor.measurements[0]);

  Serial.println("");
  delay(500);
}

// User configurable I2C calls to allow for wide portability.
void i2c_read(uint8_t address, uint8_t command, uint8_t *data, uint8_t num_bytes)
{
  Wire.requestFrom(address, num_bytes);
  for(int i=0; i < num_bytes; i++ )
  {
    data[i] = Wire.read();  
  }
}

void i2c_write(uint8_t address, uint8_t command, uint8_t *data, uint8_t num_bytes)
{
  Wire.beginTransmission(address);
  Wire.write(data, num_bytes);
  Wire.endTransmission();
}