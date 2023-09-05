# MS5837 Library

This C library is intended for use on microcontrollers, and written alongside BlueRobotic's Bar02 depth sensor using the [`MS5837-02BA` datasheet](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5837-02BA01%7FA8%7Fpdf%7FEnglish%7FENG_DS_MS5837-02BA01_A8.pdf%7FCAT-BLPS0059) as sole reference.

In contrast to [BlueRobotic's C++ Arduino library](https://github.com/bluerobotics/BlueRobotics_MS5837_Library), this C library takes a hardware/HAL agnostic approach with user-configurable I2C callback functions, and aims to provide a greater level of control in exchange for a slight increase in integration effort.

> Happy to discuss issues/feature requests & merge reasonable PRs.

## Why?

At time of writing the BlueRobotics library doesn't allow configuration of sensor sampling resolution, and reading the sensor includes a forced 40ms blocking delay.

*This* library allows for the host micro to do other work while we wait for the sampling to complete, including use the I2C bus to work with other sensors.

> Right now it doesn't support the different conversion functions used by other sensor variants.

Oh, and I wrote unit tests for it...

## Basic usage

The Arduino compatible example shows a basic working implementation.

Bring your own I2C function callbacks for read and write matching the following signature:

```c
// End-user I2C callback function signature
// address, command, buffer, number of bytes
typedef void (*user_i2c_cb_t)(uint8_t, uint8_t, uint8_t*, uint8_t);
```

The Arduino compatible implementation looks like this:

```c
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
```

To use the sensor, create the 'object' structure we'll pass around as context.

```c 
ms5837_t sensor = { 0 };
```

Setup can be done during declaration, or at runtime. Before attempting to sample, the library needs to read the calibration constants stored on the sensor.

```c
ms5837_i2c_set_read_fn( &sensor, i2c_read );
ms5837_i2c_set_write_fn( &sensor, i2c_write );

ms5837_reset( &sensor );
// A short delay might be needed after reset.
ms5837_read_calibration_data( &sensor );
```

To use the sensor, pressure and temperature data is needed before the final values can be calculated. This is because the pressure sensor is highly sensitive to temperature fluctuation, and a 2nd order compensation is applied.

After requesting a sample, the sensor needs time before it can be read out. Instead of blocking internally, the request function returns the expected sampling duration in microseconds (us), which allows you to use your RTOS yielding delay or build a simple state-machine around handling the sensor to keep running other system tasks while you wait.  

The library internally tracks the prior request, `ms5837_read_conversion` will store the result in the appropriate slot inside the `ms5837_t` structure for later use.

``` c
// When requesting a measurement, the function returns a microsecond duration
// when we can safely attempt to read the value.

// Request a pressure reading
uint32_t wait_us = ms5837_start_conversion( &sensor, SENSOR_PRESSURE, OSR_8192 );
delayMicroseconds(wait_us);
ms5837_read_conversion( &sensor );

// Request temperature
wait_us = ms5837_start_conversion( &sensor, SENSOR_TEMPERATURE, OSR_8192 );
delayMicroseconds(wait_us);
ms5837_read_conversion( &sensor );

// Once pressure and temp are sampled, run the conversion function
ms5837_calculate( &sensor );

// The values are now ready to use.
float pressure = ms5837_pressure_mbar( &sensor );
float temperature = ms5837_temperature_celcius( &sensor );
```

Once `ms5837_calculate()` runs, it clears the internal raw conversion values and prepares for new samples. 

> The converted readings stored in the structure are accessible without specific lifespans.
>
> If polling against the converted values, it's wise to keep track of the age since sampling/calculation last occurred to prevent accidental use of stale data. *The library does not help you here*!

The conversion request function allows you to choose the oversampling level with an enum value ranging from `256` to `8192`. 

| enum       | Resolution (mbar) | Delay (milliseconds) | Sample Frequency (Hz) |
| ---------- | ----------------- | -------------------- | --------------------- |
| `OSR_256`  | 0.012             | 0.56                 | 1750                  |
| `OSR_512`  | 0.009             | 1.10                 | 900                   |
| `OSR_1024` | 0.006             | 2.71                 | 360                   |
| `OSR_2048` | 0.004             | 4.32                 | 230                   |
| `OSR_4096` | 0.003             | 8.61                 | 112                   |
| `OSR_8192` | 0.002             | 17.2                 | 57                    |

> Sample frequency is the highest rate I could achieve using the datasheet's maximum duration as the delay time.
>
> I2C transaction duration also contributes to the minor reduction in effective rate from literature values.

Whilst not discussed in the datasheet at all, I found no measurable difference in accuracy when using a lower OSR value for temperature readings than pressure, which improves measurement latency somewhat.

# Running tests

[![Ceedling](https://github.com/Scottapotamas/ms5837/actions/workflows/ceedling.yml/badge.svg)](https://github.com/Scottapotamas/ms5837/actions/workflows/ceedling.yml)

Testing uses the [Ceedling](http://www.throwtheswitch.org/ceedling/) (Ruby/rake) based testing framework with `Unity` and `CMock`.

1. *If* you don't have Ceedling installed:

   - Either install it with your OS's package manager,
   - Manual install `ceedling` with `gem install --user ceedling`.

2. Once setup, run `ceedling` or `ceedling test:all`.

GitHub Actions compile the Arduino examples and run unit-tests when pull-requests are made.

# Licence

`ms5837` is [MIT licensed](LICENSE.md).