# pulu-moisture-sensor
library for the pulu moisture sensor

## example
```c
#include "mbed.h"
#include "MoistureSensors.h"

float interpolation(float x0, float y0, float x1, float y1, float xp)
{
    // Linear Interpolation 
    return y0 + ((y1-y0)/(x1-x0)) * (xp - x0);
}

DigitalIn button1(MY_BUTTON1);
DigitalIn button2(MY_BUTTON2);

I2C i2c(I2C_SDA, I2C_SCL);

MoistureSensors moist(&i2c);

int main()
{
    while (1) 
    {        
        if (!button1) {
            moist.calibrateFdcHighestPoint();
        }

        if (!button2) {
            moist.calibrateFdcLowestPoint();
        }

        int16_t results[4];
        moist.readFdcChannels(results);

        ThisThread::sleep_for(500ms);

        printf("Raw moisture data:\t%i\t%i\t%i\t%i\t \r\n", results[0], results[1], results[2], results[3]);
    }
}
```