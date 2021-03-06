#ifndef FDC1004_H
#define FDC1004_H
#include "I2C.h"
#include "mbed.h"
#include <cstdint>

#define CHANNELS 4

class FDC1004 {
    // public methods
    public:
        // constructor
        FDC1004(I2C *i2c, uint8_t address);
    
        // reset the fdc chip
        bool ResetFDM();

        // get all four calibrated measurments
        bool readFdcChannels(int16_t *results, int numSamples = 1, bool calibrated = false);

        // trigger calibration for the lowest possible capacitance (moisture content)
        bool calibrateFdcLowestPoint();

        // trigger calibration for the highest possible capacitance (moisture content)
        bool calibrateFdcHighestPoint();

        void setCalibrationLowestPoint(int16_t* values);
        void setCalibrationHighestPoint(int16_t* values);

    // public variables
    public:
        // returns true if both the high and low sides have been calibrated
        //bool isFdcCalibrated = (calibrationStatus == 0b11); 

    private:
        I2C *i2c;
        uint8_t address;

    private:
        const int calibrationSampleCount = 100;     //take average from 100 samples for calibration
        const int calibrationMargin = 300;
    
    // adresses and pointers
    private:
        const char FDC_CHANNEL_CONFIG_POINTERS[CHANNELS] = {0x08, 0x09, 0x0A, 0x0B};
        const char FDC_READ_CHANNEL_POINTERS[CHANNELS] = {0x00, 0x02, 0x04, 0x06};
        const char FDC_CHANNEL_OFSET_POINTERS[CHANNELS] = {0x0D, 0x0E, 0x0F, 0x10};
        const char FDC_CHANNEL_GAIN_POINTERS[CHANNELS] = {0x11, 0x12, 0x13, 0x14};
        const char FDC_CONFIG_POINTER = 0x0C;

    // rutime updated variables
    private:
        int16_t lowerValues[CHANNELS] = {1460, 989, 1127, 2131};
        int16_t higherValues[CHANNELS] = {25383, 25271, 24838, 26431};
        int calibrationStatus = 0b11;               // calobration status b[1] = done high, b[0] = done low

    // private methods
    private:
        // checks if the measurment is completed
        int checkMeasurmenStatus();

        // read single fdc channel measurments
        bool readFdcChannel(int16_t &value, int ch, bool calibrated);

        bool writeConfigRegisters(int channel, double gain, int16_t offset);

        void waitForMeasurment();
};

#endif