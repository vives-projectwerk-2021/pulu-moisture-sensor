#include "mbed.h"
#include "FDC1004.h"
#include <cstdint>
#include <cstdio>

FDC1004::FDC1004(I2C *i2c, uint8_t address)
{
    this->i2c = i2c;
    this->address = address;
}

/** Reset
 * @return 0 on success
 */
bool FDC1004::ResetFDM()
{
    char cmd[3] {FDC_CONFIG_POINTER, 0x80, 0x00};
    return i2c->write(address, cmd, 3);
}

int FDC1004::checkMeasurmenStatus() 
{
    char cmd[2] = {FDC_CONFIG_POINTER, 0x00};
    i2c->write(address, cmd, 1);
    i2c->read(address, cmd, 2);
    uint16_t conf = cmd[0]<<8|cmd[1];
    
    return conf & 0x0f;
}

/** 
 * @return 0 on success
 */
bool FDC1004::readFdcChannel(int16_t &value, int channel, bool calibrated) 
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

    bool err = writeConfigRegisters(channel, gain, offset);
    if(err) { return true; }

    // wait for the device to complete measurment
    waitForMeasurment();

    char cmd[3] = {FDC_READ_CHANNEL_POINTERS[channel], 0x00};
    bool ack;
    ack = i2c->write(address, cmd, 1);
    if(ack) { return true; }
    ack = i2c->read(address, cmd, 2);
    if(ack) { return true; }

    int16_t output = cmd[0]<<8|cmd[1];
    value = output;

    return ResetFDM();
}

/** 
 * @return 0 on success
 */
bool FDC1004::readFdcChannels(int16_t *results, int numSamples, bool calibrated) 
{
    for (int j=0; j<CHANNELS; j++)
    {
        int sum = 0;
        for (int i=0; i<numSamples; i++) 
        {
            int16_t value;
            bool err = readFdcChannel(value, j, calibrated);
            if(err) 
            { 
                for(int c=0; c<CHANNELS; c++) {
                    results[c] = 0;
                }
                return true;
            }
            sum += value;
        }
        results[j] = sum/numSamples;
    }
    return false;
}

/** 
 * @return 0 on success
 */
bool FDC1004::calibrateFdcLowestPoint()
{
    // read uncalibrted measurments
    bool err = readFdcChannels(lowerValues, calibrationSampleCount, false);
    if(err) { return true; }

    // apply some headroom for the calibration measurments
    for (int i=0; i<CHANNELS; i++)
    {
        lowerValues[i] -= calibrationMargin;
    }

    calibrationStatus |= 0b01;
    return false;
}

/** 
 * @return 0 on success
 */
bool FDC1004::calibrateFdcHighestPoint()
{
    // read uncalibrted measurments
    bool err = readFdcChannels(higherValues, calibrationSampleCount, false);
    if(err) { return true; }

    // apply some headroom for the calibration measurments
    for (int i=0; i<CHANNELS; i++)
    {
        higherValues[i] += calibrationMargin;
    }

    calibrationStatus |= 0b10;
    return false;
}

void FDC1004::setCalibrationLowestPoint(int16_t* values) {
    for (int i=0; i<CHANNELS; i++)
    {
        lowerValues[i] = values[i];
    }
    calibrationStatus |= 0b01;
}
void FDC1004::setCalibrationHighestPoint(int16_t* values) {
    for (int i=0; i<CHANNELS; i++)
    {
        higherValues[i] = values[i];
    }
    calibrationStatus |= 0b10;
}

/** 
 * @return 0 on success
 */
bool FDC1004::writeConfigRegisters(int channel, double gain, int16_t offset) 
{
    bool ack;
    bool reset = false;                 // reset: reset the device
    int measurment_rate = 1;            // measurment rate: 1 = 100Hz, 2 = 200Hz, 3 = 400Hz
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
    ack = i2c->write(address, data_global_conf, 3);
    if(ack) { return true; }

    // set channel configuration register
    char data_ch_conf[] = {
        FDC_CHANNEL_CONFIG_POINTERS[channel], 
        char(0b00011100 | (channel << 5)), 
        0x00};
    ack = i2c->write(address, data_ch_conf, 3);
    if(ack) { return true; }

    // set channel offset register
    char data_ch_offset[] = {
        FDC_CHANNEL_OFSET_POINTERS[channel], 
        char((offset >> 8) & 0xff), 
        char(offset & 0xff)};
    ack = i2c->write(address, data_ch_offset, 3);
    if(ack) { return true; }

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
    ack = i2c->write(address, data_ch_gain, 3);
    if(ack) { return true; }
    return false;
}

void FDC1004::waitForMeasurment() 
{
    while (checkMeasurmenStatus() == 0) {
        ThisThread::sleep_for(1ms);
    }
}