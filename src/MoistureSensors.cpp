#include "mbed.h"
#include "PinMap.h"
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

    if (calibrated && calibrationStatus == 0b11) {
        gain = (pow(2,15)-0.0)/(higherValues[channel]-lowerValues[channel]);
        offset = -lowerValues[channel];
        //printf("gain:\t%i, offset:\t%i\r\n", int(gain*1000), offset);
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

void MoistureSensors::readFdcChannels(int16_t *results) 
{
    //printf("Lower values: \t%i\t%i\t%i\t%i\t \r\n", lowerValues[0], lowerValues[1], lowerValues[2], lowerValues[3]);
    //printf("Higher values: \t%i\t%i\t%i\t%i\t \r\n", higherValues[0], higherValues[1], higherValues[2], higherValues[3]);
    for (int i=0; i<channels; i++)
    {
        results[i] = readFdcChannel(i, true);
    }
}

void MoistureSensors::readUncalibratedFdcChannels(int16_t *results) 
{
    for (int i=0; i<channels; i++)
    {
        results[i] = readFdcChannel(i, false);
    }
}

void MoistureSensors::calibrateFdcLowestPoint()
{

    getAverage(lowerValues, calibrationSampleCount, false);

    // subtract 100 units margin to the lower values
    for (int i=0; i<channels; i++)
    {
        lowerValues[i] -= calibrationMargin;
    }

    calibrationStatus |= 0b01;
}

void MoistureSensors::calibrateFdcHighestPoint()
{
    getAverage(higherValues, calibrationSampleCount, false);

    // add x units margin to the higher values
    for (int i=0; i<channels; i++)
    {
        higherValues[i] += calibrationMargin;
    }

    calibrationStatus |= 0b10;
}

void MoistureSensors::writeConfigRegisters(int channel, double gain, int16_t offset) 
{
    bool reset = false;             // reset: reset the device
    int measurment_rate = 2;        // measurment rate: 1 = 100Hz, 2 = 200Hz, 3 = 400Hz
    bool repeat = false;             // repeat measurment: false = one shot measurment
    int enable = 0b1000 >> channel;    // enable selected channel

    uint16_t configure = 
        (reset << 15) | 
        (measurment_rate << 10) | 
        (repeat << 8) |
        (enable << 4) | 
        0x0000;

    // set global configuration register
    i2c->write(MoistAddr, (char[]){
        FDC_CONFIG_POINTER,
        char((configure >> 8) & 0xff),
        char((0b1000 >> channel) << 4)}, 3);

    // set channel configuration register
    i2c->write(MoistAddr, (char[]){
        FDC_CHANNEL_CONFIG_POINTERS[channel], 
        char(0b00011100 | (channel << 5)), 
        0x00}, 3);

    // set channel offset register
    i2c->write(MoistAddr, (char[]){
        FDC_CHANNEL_OFSET_POINTERS[channel], 
        char((offset >> 8) & 0xff), 
        char(offset & 0xff)}, 3);

    // set channel gain register
    int gainIntegerPart = int(gain);
    double gainDecimalPart = gain - gainIntegerPart;

    if (gainIntegerPart >= 0b11)    gainIntegerPart = 0b11;
    if (gainIntegerPart <= 0b00)    gainIntegerPart = 0b00;

    int16_t gainReg = (gainIntegerPart << 14) | int(gainDecimalPart*pow(2, 14));

    i2c->write(MoistAddr, (char[]){
        FDC_CHANNEL_GAIN_POINTERS[channel], 
        char((gainReg >> 8) & 0xff), 
        char(gainReg & 0xff)}, 3);

    // example
    // i2c.write(device_address, (char[]){reg_pointer, data1, data2}, 3);
}

void MoistureSensors::waitForMeasurment() 
{
    while (checkMeasurmenStatus() == 0) {
        ThisThread::sleep_for(1ms);
    }
}

void MoistureSensors::getAverage(int16_t *values, int numSamples, bool calibrated) 
{
    for (int j=0; j<channels; j++)
    {
        int sum = 0;
        for (int i=0; i<numSamples; i++) 
        {
            sum += readFdcChannel(j, calibrated);
        }
        values[j] = sum/numSamples;
    }
}