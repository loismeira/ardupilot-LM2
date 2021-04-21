#pragma once

#include <inttypes.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

struct adc_report_s {
    //uint64_t sampletime; // sample time in us for debugging
    float data;
};

class AP_ADC_ADS1115
{
public:
    AP_ADC_ADS1115();
    ~AP_ADC_ADS1115();

    bool init(uint8_t addr);

    uint8_t get_channels_number() const
    {
        return _channels_number;
    }

private:
    static const uint8_t _channels_number;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    uint16_t            _gain;
    uint8_t           _addr;
    int                 _channel_to_read;
    adc_report_s        *_samples;

    void _update();
    bool _start_conversion(uint8_t channel);

    float _convert_register_data_to_mv(int16_t word) const;
    void Log_Write_ADC(adc_report_s* rep, uint8_t i);
};
