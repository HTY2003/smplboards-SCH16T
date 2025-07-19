# smplboards SCH16T

Arduino library for Murata SCH16T-K01 and SCH16T-K10 6 DoF IMU sensors, adapted from Murata source code

## Installation

To be done...soon

## Wiring

To be done...soon

## API Reference

`SCH16T_OK`
`SCH16T_ERR_NULL_POINTER`
`SCH16T_ERR_INVALID_PARAM`
`SCH16T_ERR_SENSOR_INIT`
`SCH16T_ERR_OTHER`

`SCH16T_raw_data`
`SCH16T_result`
`SCH16T_filter`
`SCH16T_sensitivity`
`SCH16T_decimation`
`SCH16T_status`
`SCH16T_axis`

The `SCH16T_K10` and `SCH16T_K01` classes are mostly identical, except for the gyroscope sensitivity values (more explained below):

#### `int begin(SCH16T_filter sFilter, SCH16T_sensitivity sSensitivity, SCH16T_decimation sDecimation, bool enableDRY);`
#### `void getData(SCH16T_raw_data *data);`
#### `void getDataDecimated(SCH16T_raw_data *data);`
#### `void convertData(SCH16T_raw_data *data_in, SCH16T_result *data_out);`
#### `void convertDataDecimated(SCH16T_raw_data *data_in, SCH16T_result *data_out);`
#### `int setFilters(uint32_t Freq_Rate12, uint32_t Freq_Acc12, uint32_t Freq_Acc3);`
#### `int setRateSensDec(uint16_t Sens_Rate1, uint16_t Sens_Rate2, uint16_t Dec_Rate2);`
#### `int getRateSensDec(uint16_t *Sens_Rate1, uint16_t *Sens_Rate2, uint16_t *Dec_Rate2);`
#### `int setAccSensDec(uint16_t Sens_Acc1, uint16_t Sens_Acc2, uint16_t Sens_Acc3, uint16_t Dec_Acc2);`
#### `int getAccSensDec(uint16_t *Sens_Acc1, uint16_t *Sens_Acc2, uint16_t *Sens_Acc3, uint16_t *Dec_Acc2);`
#### `int setDRY(int8_t polarity, bool enable);`
#### `int enableMeas(bool enableSensor, bool setEOI);`
#### `void reset(void);`
#### `void sendSPIreset(void);`
#### `bool verifyStatus(SCH16T_status *Status);`
#### `int getStatus(SCH16T_status *Status);`
#### `char *getSnbr(void);`