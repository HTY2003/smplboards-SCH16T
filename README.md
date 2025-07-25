# smplboards SCH16T

Arduino library for Murata SCH16T-K01 and SCH16T-K10 6 DoF IMU sensors, adapted from Murata source code

## Installation

1. Download this repository from GitHub as a .zip file.

2. In Arduino IDE, navigate to `Sketch` > `Include Library` > `Add .ZIP Library...`, then select the downloaded .zip file to install the library.

3. When installed, you can include the library header by selecting it in `Sketch` > `Include Library`, and view example sketches in `File` > `Examples`.

In the future, I will add this library to the Arduino Library Manager for easier installation.

## Wiring

| Pin | Description |
|---|---|
| SPI RX | SPI Receive Pin of sensor. Connect to SPI TX pin on microcontroller |
| CS | SPI Chip Select Pin of sensor. Connect to any digital pin on microcontroller |
| SCK | SPI Clock Pin of sensor. Connect to SCK pin on microcontroller |
| SPI TX | SPI Transmit Pin of sensor. Connect to SPI RX pin on microcontroller |
| EXTRESN | Reset pin, pulled HIGH with a 10K resistor to enable the sensor by default. Connect to button/microcontroller pin if needed. |
| 3V3 (+) | Power pin. Connect to 3V3 power source |
| GND (-) | Ground pin. Connect to GND pin of microcontroller |
| DRY_SYNC | Data Ready output/Sync input Pin of sensor. Can be used with interrupt-capable digital pin on microcontroller to read data once it is ready, resulting in minimized timing jitter. |

This board is designed for 3.3V power only, and all IO pins are not 5V-tolerant.

On the back of the board, there are solder pads to alter the values of TA9 and TA8 (0 by default). Solder the middle pad to the `1` pad to change its value to 1.

Changing these values allows for multiple of the sensor on the same SPI bus without requiring additional Chip Select pins - as long as they all have different TA values.

On the top of the board, there are markings for the X Y and Z axes of the sensor. Additionally, there is a JST-SH 8 pin connector (1mm pitch) for more secure and compact wiring.

## API Reference

### Error Codes
These codes are defined as pre-processor macros for users to check function outputs and verify whether library operations succeed:

* `SCH16T_OK`
* `SCH16T_ERR_NULL_POINTER`
* `SCH16T_ERR_INVALID_PARAM`
* `SCH16T_ERR_SENSOR_INIT`
* `SCH16T_ERR_OTHER`

### Structs
Additionally, the library uses structs to organize groups of data to be passed into or modified by functions using pointers, each listed below:

* `SCH16T_raw_data`: Stores raw data readings transmitted from the sensor (not converted to real world units yet)
    * `int32_t Rate1_raw[3]`
    * `int32_t Rate2_raw[3]`
    * `int32_t Acc1_raw[3]`
    * `int32_t Acc2_raw[3]`
    * `int32_t Acc3_raw[3]`
    * `int32_t Temp_raw`
    * `bool frame_error`

* `SCH16T_result`: Stores converted readings stored in real world units
    * `float Rate1[3]`
    * `float Rate2[3]`
    * `float Acc1[3]`
    * `float Acc2[3]`
    * `float Acc3[3]`
    * `float Temp`

* `SCH16T_filter`: Stores filter settings for gyro and accelerometer channels
    * `uint16_t Rate12`
    * `uint16_t Acc12`
    * `uint16_t Acc3`
    * Valid values for `Rate` (Hz): 0, 13, 30, 68, 235, 280, 370
    * Valid values for `Acc` (Hz): 0, 13, 30, 68, 210, 240, 290

* `SCH16T_sensitivity`: Stores sensitivity settings for gyro and accelerometer channels
    * `uint16_t Rate1`
    * `uint16_t Rate2`
    * `uint16_t Acc1`
    * `uint16_t Acc2`
    * `uint16_t Acc3`
    * Valid values for `Rate` (K01 only, LSB/dps): 100, 200, 400
    * Valid values for `Rate` (K10 only, LSB/dps): 1600, 3200, 6400
    * Valid values for `Accel` (LSB/(m/s^2)): 3200, 6400, 12800, 25600

* `SCH16T_decimation`: Stores decimation settings for decimated-output gyro and accelerometer channels
    * `uint16_t Rate2`
    * `uint16_t Acc2`
    * Valid values: 2, 4, 8, 16, 32

* `SCH16T_status`: Stores status register values
    * `uint16_t Summary`
    * `uint16_t Summary_Sat`
    * `uint16_t Common`
    * `uint16_t Rate_Common`
    * `uint16_t Rate_X`
    * `uint16_t Rate_Y`
    * `uint16_t Rate_Z`
    * `uint16_t Acc_X`
    * `uint16_t Acc_Y`
    * `uint16_t Acc_Z`

### Enums

* `SCH16T_axis`: Can be used to retrieve correct array index to read a specific axis
    * `AXIS_X`
    * `AXIS_Y`
    * `AXIS_Z`

### Constructors

There is a constructor for each of the SCH16T models (K01 and K10). The input parameters for the constructors are identical, but  use the correct constructor for your sensor:

#### `SCH16T_K01(SPIClass& spi, int cs_pin, int reset_pin = -1, int ta9_8 = 0)`
#### `SCH16T_K10(SPIClass& spi, int cs_pin, int reset_pin = -1, int ta9_8 = 0)`

* `spi`: SPI object used for communications (e.g. `SPI` or `SPI1`)
* `cs_pin`: GPIO pin number for CS pin
* `reset_pin`: GPIO pin number for EXTRESN if it is connected (optional, defaults to -1)
* `ta9_8`: Bits 9 and 8 of device target address (optional, defaults to 0, can modify with TA9 and TA8 solder jumpers to a value from 0-3)



### Functions

The `SCH16T_K10` and `SCH16T_K01` classes are mostly identical, except that the gyroscope sensitivity values accepted are different:



#### `int begin(SCH16T_filter sFilter, SCH16T_sensitivity sSensitivity, SCH16T_decimation sDecimation, bool enableDRY = false)`

Initializes sensor with given settings. Call this function before all others.

* `sFilter`: Filter settings
* `sSensitivity`: Sensitivity settings
* `sDecimation`: Decimation settings
* `enableDRY`: When enabled, the sensor will output HIGH on the DRY pin when a new reading is ready. Defaults to `false`
* Returns: Error code representing success of operation

Note: When DRY output is enabled, the interrupt is only cleared when the decimated output channels are read. This can be done with `getDataDecimated()`.

Note2: Gyroscope sensitivity values accepted for K01 and K10 sensors are different. Look at the description for the `SCH16T_sensitivity` struct or `setRateSensDec()` for the accepted values.



#### `void getData(SCH16T_raw_data *data)`

Reads interpolated raw data readings (Rate1_raw and Acc1_raw) and writes them into a SCH16T_raw_data object.
Also reads temperature raw data reading (Temp_raw).

* `data`: Pointer to SCH16T_raw_data object to be written to




#### `void getDataDecimated(SCH16T_raw_data *data)`

Reads decimated raw data readings (Rate2_raw and Acc2_raw) and writes them into a SCH16T_raw_data object.
Also reads temperature raw data reading (Temp_raw).

* `data`: Pointer to SCH16T_raw_data object to be written to



#### `void getDataAux(SCH16T_raw_data *data)`

Reads auxilliary accel raw data readings (Acc3_raw) and writes it into a SCH16T_raw_data object.
Also reads temperature raw data reading (Temp_raw).

* `data`: Pointer to SCH16T_raw_data object to be written to



#### `void convertData(SCH16T_raw_data *data_in, SCH16T_result *data_out)`

Converts interpolated raw data readings (Rate1_raw and Acc1_raw) from SCH16T_raw_data object to real-world units (Rate1 and Acc1) in SCH16T_result object.
Also converts temperature reading (Temp_raw) to real-world units (Temp).

* `* data_in`: Pointer to SCH16T_raw_data object to be read from
* `* data_out`: Pointer to SCH16T_result object to be written to



#### `void convertDataDecimated(SCH16T_raw_data *data_in, SCH16T_result *data_out)`

Converts decimated raw data readings (Rate2_raw and Acc2_raw) from SCH16T_raw_data object to real-world units (Rate2 and Acc2) in SCH16T_result object.
Also converts temperature reading (Temp_raw) to real-world units (Temp).

* `* data_in`: Pointer to SCH16T_raw_data object to be read from
* `* data_out`: Pointer to SCH16T_result object to be written to



#### `void convertDataAux(SCH16T_raw_data *data_in, SCH16T_result *data_out)`

Converts auxilliary accel raw data readings (Acc3_raw) from SCH16T_raw_data object to real-world units (Acc3) in SCH16T_result object.
Also converts temperature reading (Temp_raw) to real-world units (Temp).

* `* data_in`: Pointer to SCH16T_raw_data object to be read from
* `* data_out`: Pointer to SCH16T_result object to be written to



#### `int setFilters(uint32_t Freq_Rate12, uint32_t Freq_Acc12, uint32_t Freq_Acc3)`

Sets cutoff frequencies for low pass filters on the data output channels.
Valid values for `Rate` (Hz): 0, 13, 30, 68, 235, 280, 370
Valid values for `Acc` (Hz): 0, 13, 30, 68, 210, 240, 290

* `Freq_Rate12`: Cutoff frequencies for Gyro Output Channels 1 and 2
* `Freq_Acc12`: Cutoff frequencies for Accel Output Channels 1 and 2
* `Freq_Acc3`: Cutoff frequencies for Accel Output Channel 3
* Returns: Error code representing success of operation



#### `int setRateSensDec(uint16_t Sens_Rate1, uint16_t Sens_Rate2, uint16_t Dec_Rate2)`

Sets sensitivity and decimation settings for Accel channels.
Valid sensitivity values for `Rate` (K01 only, LSB/dps): 100, 200, 400
Valid sensitivity values for `Rate` (K10 only, LSB/dps): 1600, 3200, 6400
Valid decimation ratio values: 2, 4, 8, 16, 32

* `Sens_Rate1`: Sensitivity for Gyro Channel 1
* `Sens_Rate2`: Sensitivity for Gyro Channel 2
* `Dec_Rate2`: Decimation for Gyro Channel 2
* Returns: Error code representing success of operation



#### `int getRateSensDec(uint16_t *Sens_Rate1, uint16_t *Sens_Rate2, uint16_t *Dec_Rate2)`

Gets sensitivity and decimation settings for Accel channels and write to variables.

* `* Sens_Rate1`: Variable to write sensitivity of Gyro Channel 1 to
* `* Sens_Rate2`: Variable to write sensitivity of Gyro Channel 2 to
* `* Dec_Rate2`: Variable to write decimation of Gyro Channel 2 to
* Returns: Error code representing success of operation



#### `int setAccSensDec(uint16_t Sens_Acc1, uint16_t Sens_Acc2, uint16_t Sens_Acc3, uint16_t Dec_Acc2)`

Sets sensitivity and decimation settings for Accel channels.
Valid sensitivity values for `Accel` (LSB/(m/s^2)): 3200, 6400, 12800, 25600
Valid decimation ratio values: 2, 4, 8, 16, 32

* `Sens_Acc1`: Sensitivity for Accel Channel 1
* `Sens_Acc2`: Sensitivity for Accel Channel 2
* `Dec_Acc2`: Sensitivity for Accel Channel 2
* Returns: Error code representing success of operation



#### `int getAccSensDec(uint16_t *Sens_Acc1, uint16_t *Sens_Acc2, uint16_t *Sens_Acc3, uint16_t *Dec_Acc2)`

Gets sensitivity and decimation settings for Accel channels and write to variables.

* `* Sens_Acc1`: Variable to write sensitivity of Accel Channel 1 to
* `* Sens_Acc2`: Variable to write sensitivity of Accel Channel 2 to
* `* Dec_Acc2`: Variable to write decimation of Accel Channel 2 to
* Returns: Error code representing success of operation



#### `int setDRY(int8_t polarity, bool enable)`

Enables/disable DRY pin output and sets output polarity.

* `polarity`: Sets DRY output polarity. 0 = high active (default), 1 = low active. -1 = don't care
* `enable`: Sets whether DRY output should be enabled
* Returns: Error code representing success of operation



#### `int enableMeas(bool enableSensor, bool setEOI)`

Activates/deactivates measurement mode and sets the EOI (End Of Initialization) bit if needed.

* `enableSensor`: Sets whether measurement mode should be enabled
* `setEOI`: Sets EOI-bit. Locks all R/W registers, except soft reset. Can only be set when no errors in common status.
* Returns: Error code representing success of operation



#### `void reset()`

Performs hard reset of sensor by pulling EXTRESN pin LOW, then HIGH (only if the pin is connected and pin number is passed into constructor).



#### `void sendSPIreset()`

Sends soft reset command over SPI.



#### `int getStatus(SCH16T_status *Status)`

Reads all status register values and writes them into SCH16T_status object.

* `* Status`: Pointer to SCH16T_status object to be written to
* Returns: Error code representing success of operation



#### `bool verifyStatus(SCH16T_status *Status)`

Checks all status register values in SCH16T_status object and returns `true` if they are all valid, `false` otherwise.

* `* Status`: Pointer to SCH16T_status object to be read from
* Returns: `true` if status register values are all valid, `false` otherwise.



#### `char *getSnbr()`

Returns pointer to C string containing serial number. 

You can copy this into a C string variable using `strcpy()` as shown here: `strcpy(serial_num, imu.getSnbr());`.
