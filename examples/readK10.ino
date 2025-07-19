#include "SCH16T.h"

#define FILTER_RATE         280.0f      // Hz, LPF1 Nominal Cut-off Frequency (-3dB).
#define FILTER_ACC12        240.0f
#define FILTER_ACC3         240.0f
#define SENSITIVITY_RATE1   200.0f     // LSB / dps, DYN1 Nominal Sensitivity for 20 bit data.
#define SENSITIVITY_RATE2   200.0f
#define SENSITIVITY_ACC1    3200.0f     // LSB / m/s2, DYN1 Nominal Sensitivity for 20 bit data.
#define SENSITIVITY_ACC2    3200.0f
#define SENSITIVITY_ACC3    3200.0f     // LSB / m/s2, DYN1 Nominal Sensitivity for 20 bit data.
#define DECIMATION_RATE     32          // DEC5, Output sample rate decimation.
#define DECIMATION_ACC      32

#define SPI_OBJECT          SPI
#define CS_PIN              10
#define RESET_PIN           -1          //leave as -1 if not used

SCH16T_K10 imu(SPI_OBJECT, CS_PIN, RESET_PIN);

char serial_num[15];
int  init_status = SCH16T_ERR_OTHER;
SCH16T_filter         Filter;
SCH16T_sensitivity    Sensitivity;
SCH16T_decimation     Decimation;

void setup() {
    Serial.begin(115200);
    SPI_OBJECT.begin();    // Initialize SPI hardware before initializing sensor

    delay(3000);

    Filter.Rate12 = FILTER_RATE;
    Filter.Acc12  = FILTER_ACC12;
    Filter.Acc3   = FILTER_ACC3;

    Sensitivity.Rate1 = SENSITIVITY_RATE1;
    Sensitivity.Rate2 = SENSITIVITY_RATE2;
    Sensitivity.Acc1  = SENSITIVITY_ACC1;
    Sensitivity.Acc2  = SENSITIVITY_ACC2;
    Sensitivity.Acc3  = SENSITIVITY_ACC3;

    Decimation.Rate2 = DECIMATION_RATE;
    Decimation.Acc2  = DECIMATION_ACC;

    while (init_status != SCH16T_OK)
    {
        init_status = imu.begin(Filter, Sensitivity, Decimation, false);
        if (init_status != SCH16T_OK) {
            Serial.println("ERROR");
            delay(3000);
        }
    }

    // Read serial number from the sensor.
    strcpy(serial_num, imu.getSnbr());
    Serial.print("Serial Number: ");
    Serial.println(serial_num);
}

void loop() {
    imu.getData(&raw);
    imu.convertData(&raw, &result);

    Serial.print("Gyro X: ");
    Serial.println(result.Rate1[SCH16T_axis::AXIS_X]);
    Serial.print("Gyro Y: ");
    Serial.println(result.Rate1[SCH16T_axis::AXIS_Y]);
    Serial.print("Gyro Z: ");
    Serial.println(result.Rate1[SCH16T_axis::AXIS_Z]);
    Serial.println();

    Serial.print("Accel X: ");
    Serial.println(result.Acc1[SCH16T_axis::AXIS_X]);
    Serial.print("Accel Y: ");
    Serial.println(result.Acc1[SCH16T_axis::AXIS_Y]);
    Serial.print("Accel Z: ");
    Serial.println(result.Acc1[SCH16T_axis::AXIS_Z]);
    Serial.println();

    Serial.print("Temperature: ");
    Serial.println(result.Temp);
    Serial.println();
    Serial.println();

    delay(100);
}