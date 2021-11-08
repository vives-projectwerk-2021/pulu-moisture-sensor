#ifndef MOISTURESENSORS_H
#define MOISTURESENSORS_H
#include "I2C.h"
#include "mbed.h"
#include <cstdint>

class MoistureSensors {
    // public methods
    public:
        // constructor
        MoistureSensors(I2C *i2c);
    
        // reset the fdc chip
        void ResetFDM();

        // get all four calibrated measurments
        void readFdcChannels(int16_t *results);

        // trigger calibration for the lowest possible capacitance (moisture content)
        void calibrateFdcLowestPoint();

        // trigger calibration for the highest possible capacitance (moisture content)
        void calibrateFdcHighestPoint();

    // public variables
    public:
        // returns true if both the high and low sides have been calibrated
        //bool isFdcCalibrated = (calibrationStatus == 0b11); 

    private:
        I2C *i2c;

    private:
        const int channels = 4;
        const int calibrationSampleCount = 100;     //take average from 100 samples for calibration
        const int calibrationMargin = 300;
    
    // adresses and pointers
    private:
        const char MoistAddr = 0xA0;                 // FDC1004 address:    0x50<<1

        const char FDC_CHANNEL_CONFIG_POINTERS[4] = {0x08, 0x09, 0x0A, 0x0B};
        const char FDC_READ_CHANNEL_POINTERS[4] = {0x00, 0x02, 0x04, 0x06};
        const char FDC_CHANNEL_OFSET_POINTERS[4] = {0x0D, 0x0E, 0x0F, 0x10};
        const char FDC_CHANNEL_GAIN_POINTERS[4] = {0x11, 0x12, 0x13, 0x14};
        const char FDC_CONFIG_POINTER = 0x0C;

    // rutime updated variables
    private:
        int16_t lowerValues[4] = {1460, 989, 1127, 2131};
        int16_t higherValues[4] = {25383, 25271, 24838, 26431};
        int calibrationStatus = 0b11;               // calobration status b[1] = done high, b[0] = done low

    // private methods
    private:
        // get all four uncalibrated measurments
        void readUncalibratedFdcChannels(int16_t *results);

        // checks if the measurment is completed
        int checkMeasurmenStatus();

        // read single fdc channel measurments
        int16_t readFdcChannel(int ch, bool calibrated);

        void writeConfigRegisters(int channel, double gain, int16_t offset);

        void waitForMeasurment();

        void getAverage(int16_t *values, int numSamples, bool calibrated);
};

#endif