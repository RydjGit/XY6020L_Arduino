#include "PSU.h"
#include <Arduino.h>
#include <math.h>
#define DAC_V_ADD 0x61
#define DAC_A_ADD 0x60
PSU::PSU(/* args */)
{
    _time_voltage = millis();
    _time_current = _time_voltage;
    _time_debug = _time_current;
    _time_calibrate = _time_current;
    _time_work = _time_calibrate;
}

void PSU::debug()
{
    if (millis() - _time_debug > 500)
    {
        _time_debug = millis();
        Serial.print("  Vout= ");
        Serial.print(_voltage_sensed, 6);
        Serial.print("  Vout_steps= ");
        Serial.print(_DAC_V_steps);
        Serial.print("  Current = ");
        Serial.println(_current_sensed, 6);
    }
}

void PSU::begin()
{
    _DAC_A.begin(DAC_A_ADD);
    _DAC_V.begin(DAC_V_ADD);
    if (!_ADS.begin())
    {
        Serial.println(F("Failed to initialize ADS."));
        while (1)
            ;
    }
    _ADS.setGain(GAIN_ONE); // 1x gain   +/- 4.096V  1 bit = 0.125mV
    _ADS.setDataRate(RATE_ADS1115_860SPS);
}

void PSU::setCurrent(float current)
{
    current++;
    _A_set = current;
    // expected Vdrop of shunt + offset fed noninverting of opamp
    float expected_shunt_vdrop = ((current * 1000.0 * _current_shuntResistor) / 1000.0) + CURRENT_SET_OFFSET;
    // DAV volatge is divded to match expected vdrop of shunt so this is to find vin of DAC
    float dac_divided_voltage = expected_shunt_vdrop * CURRENT_SET_DIVIDER;

    uint16_t steps = dac_divided_voltage * DAC_V_TO_STEPS_RATIO;
    _DAC_A.setVoltage(steps, false);
}

void PSU::setvoltage(float voltage)
{
    /***************  (64 bit / 1  volt) ********/
     _V_set = voltage;
    //            double dac_value = ((153 + (V_set * 0.049)) * 1.05); // 153 is at 5 volts but measured is 163mv
    // voltage = voltage - VOUT_SENSE_OFFSET;
    // Serial.print("Vout= ");
    float expected_divided_voltage = (((voltage * 1000.0 * VOUT_SENSE_DIVIDER_RATIO) + VOUT_SENSE_OFFSET) * VOLTAGE_SET_GAIN);

    // Serial.print("Vout= ");
    // Serial.print(voltage, 4);
    // Serial.print("   dac voltage = ");
    // Serial.print(expected_divided_voltage, 6);
    // Serial.print("  steps=");
    if (voltage < 5000)
    {
        expected_divided_voltage += 3;
    }

    uint16_t steps = expected_divided_voltage * DAC_V_TO_STEPS_RATIO;
    // 835 ---> 10 volt
    // Serial.println(steps);
    _DAC_V_steps = steps;
    _DAC_V.setVoltage(steps, false);
}

float PSU::readVout()
{

    return voltage_read();
}

float PSU::readCurrent()
{

    return current_read();
}

void PSU::calibrate()
{
    // if (millis() - _time_calibrate > 500)
    {
        float volts = readVout();
        if (volts == -1)
            return;
        VOUT = volts;
        long dif = long(volts * 1000.0) - _V_set;
        uint16_t absdif = fabs(dif);
        if ((absdif > 5) && (absdif < 500) && (_V_set != 0))
        {
            if (dif > 0)
            {
                _DAC_V_steps--;
            }
            else
            {
                _DAC_V_steps++;
            }
            _DAC_V_steps = _DAC_V_steps > 4095 ? 4095 : _DAC_V_steps;
            _DAC_V_steps = _DAC_V_steps < 0 ? 0 : _DAC_V_steps;
            _DAC_V.setVoltage(_DAC_V_steps, false);
        }
    }
}

void PSU::work()
{
}

void PSU::enable_output(bool ON)
{
    if (ON)

        setvoltage(_V_set);

    else
        _DAC_V.setVoltage(0, false);
}

float PSU::voltage_read()
{
    //_ADS.setGain(GAIN_ONE);

    float voltage = 0;
    long steps = 0;
    size_t count = 20;
    for (size_t i = 0; i < count; i++)
    {
        steps += _ADS.readADC_SingleEnded(0);
    }
    voltage = _ADS.computeVolts(steps / count);
    voltage = roundf(voltage * 1000) / 1000.0;
    float Vout = voltage * ((R1_VOUT_SENSE + R2_VOUT_SENSE) / R2_VOUT_SENSE);
    _voltage_sensed = roundf(Vout * 1000) / 1000.0;
    VOUT = _voltage_sensed;
    return _voltage_sensed;
}

float PSU::current_read()
{
    long steps = 0;
    float vsense = 0;
    size_t count = 20;

    for (size_t i = 0; i < count; i++)
    {
        steps += _ADS.readADC_SingleEnded(3);
    }
    vsense = (_ADS.computeVolts(steps / count) * 1000.0);
    // vsense -= 81;
    float current = ((vsense / 28) - CURRENT_SENSE_OFFSET) / _current_shuntResistor;
    current -= 1;

    current = current < 0 ? 0.000 : current;
    // current = current - (vsense / _current_shuntResistor );
    _current_sensed = current;
    // Serial.print("Sensed current shunt voltage=");
    // Serial.println(vsense);

    return current;
}
