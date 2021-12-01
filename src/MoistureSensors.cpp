#include "mbed.h"
#include "MoistureSensors.h"
#include <cstdint>
#include <cstdio>

MoistureSensors::MoistureSensors(I2C *i2c)
{
    this->i2c = i2c;
}

void MoistureSensors::ResetFDM()
{
    char cmd[3] {FDC_CONFIG_POINTER, 0x80, 0x00};
    i2c->write(MoistAddr, cmd, 3);           // Write adress/command byte, then register address
}

int MoistureSensors::checkMeasurmenStatus() 
{
    char cmd[2] = {FDC_CONFIG_POINTER, 0x00};
    i2c->write(MoistAddr, cmd, 1);
    i2c->read(MoistAddr, cmd, 2);
    uint16_t conf = cmd[0]<<8|cmd[1];
    
    return conf & 0x0f;
}

int16_t MoistureSensors::readFdcChannel(int channel, bool calibrated) 
{
    double gain;
    int16_t offset;

    // calculate gain and offset for the lower and higher values
    if (calibrated && calibrationStatus == 0b11) {
        gain = (pow(2,15)-0.0)/(higherValues[channel]-lowerValues[channel]);
        offset = -lowerValues[channel];
    }
    else {
        gain = 1;
        offset = 0;
    }

    writeConfigRegisters(channel, gain, offset);

    // wait for the device to complete measurment
    waitForMeasurment();

    char cmd[3] = {FDC_READ_CHANNEL_POINTERS[channel], 0x00};

    i2c->write(MoistAddr, cmd, 1);
    i2c->read(MoistAddr, cmd, 2);

    int16_t output = cmd[0]<<8|cmd[1];

    ResetFDM();

    return output;
}

void MoistureSensors::readFdcChannels(int16_t *results, int numSamples, bool calibrated) 
{
    for (int j=0; j<CHANNELS; j++)
    {
        int sum = 0;
        for (int i=0; i<numSamples; i++) 
        {
            sum += readFdcChannel(j, calibrated);
        }
        results[j] = sum/numSamples;
    }
}

void MoistureSensors::calibrateFdcLowestPoint()
{
    // read uncalibrted measurments
    readFdcChannels(lowerValues, calibrationSampleCount, false);

    // apply some headroom for the calibration measurments
    for (int i=0; i<CHANNELS; i++)
    {
        lowerValues[i] -= calibrationMargin;
    }

    calibrationStatus |= 0b01;
}

void MoistureSensors::calibrateFdcHighestPoint()
{
    // read uncalibrted measurments
    readFdcChannels(higherValues, calibrationSampleCount, false);

    // apply some headroom for the calibration measurments
    for (int i=0; i<CHANNELS; i++)
    {
        higherValues[i] += calibrationMargin;
    }

    calibrationStatus |= 0b10;
}

void MoistureSensors::writeConfigRegisters(int channel, double gain, int16_t offset) 
{
    bool reset = false;                 // reset: reset the device
    int measurment_rate = 2;            // measurment rate: 1 = 100Hz, 2 = 200Hz, 3 = 400Hz
    bool repeat = false;                // repeat measurment: false = one shot measurment
    int enable = 0b1000 >> channel;     // enable selected channel

    uint16_t configure = 
        (reset << 15) | 
        (measurment_rate << 10) | 
        (repeat << 8) |
        (enable << 4) | 
        0x0000;

    // set global configuration register
    char data_global_conf[] = {
        FDC_CONFIG_POINTER,
        char((configure >> 8) & 0xff),
        char((0b1000 >> channel) << 4)};
    i2c->write(MoistAddr, data_global_conf, 3);

    // set channel configuration register
    char data_ch_conf[] = {
        FDC_CHANNEL_CONFIG_POINTERS[channel], 
        char(0b00011100 | (channel << 5)), 
        0x00};
    i2c->write(MoistAddr, data_ch_conf, 3);

    // set channel offset register
    char data_ch_offset[] = {
        FDC_CHANNEL_OFSET_POINTERS[channel], 
        char((offset >> 8) & 0xff), 
        char(offset & 0xff)};
    i2c->write(MoistAddr, data_ch_offset, 3);

    // set channel gain register
    int gainIntegerPart = int(gain);
    double gainDecimalPart = gain - gainIntegerPart;

    // clamp gain between 0 - 4
    if (gainIntegerPart >= 0b11)    gainIntegerPart = 0b11;
    if (gainIntegerPart <= 0b00)    gainIntegerPart = 0b00;

    int16_t gainReg = (gainIntegerPart << 14) | int(gainDecimalPart*pow(2, 14));

    char data_ch_gain[] = {
        FDC_CHANNEL_GAIN_POINTERS[channel], 
        char((gainReg >> 8) & 0xff), 
        char(gainReg & 0xff)};
    i2c->write(MoistAddr, data_ch_gain, 3);
}

void MoistureSensors::waitForMeasurment() 
{
    while (checkMeasurmenStatus() == 0) {
        ThisThread::sleep_for(1ms);
    }
}