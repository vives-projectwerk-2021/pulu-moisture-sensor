# pulu-moisture-sensor
library for the pulu moisture sensor

## example
```c
#include "mbed.h"
#include "pulu-moisture-sensor/src/MoistureSensors.h"

I2C i2c(I2C_SDA, I2C_SCL);

MoistureSensors moist(&i2c);

int main()
{
    while (1) 
    {
        uint16_t samplesPerMeasurment = 5;
        bool calibratedMeasurments = false;
        
        int16_t results[4];
        moist.readFdcChannels(results, samplesPerMeasurment, calibratedMeasurments);

        ThisThread::sleep_for(500ms);

        printf("Raw moisture data:\t%i\t%i\t%i\t%i\t \r\n", results[0], results[1], results[2], results[3]);
    }
}
```