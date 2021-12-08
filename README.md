# pulu-moisture-sensor
library for the pulu moisture sensor

## example
```c
#include "mbed.h"
#include "FDC1004.h"

I2C i2c(I2C_SDA, I2C_SCL);

FDC1004 moist(&i2c, 0x50 << 1);

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