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
    //current++;
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
    float expected_divided_voltage = (((voltage * 1000.0 * VOUT_SET_DIVIDER_RATIO) + VOUT_SET_OFFSET) * VOLTAGE_SET_GAIN);

    Serial.print("Vout= ");
    Serial.print(voltage, 4);
    Serial.print("   dac voltage = ");
    Serial.print(expected_divided_voltage, 6);
    // Serial.print("  steps=");
    // if (voltage < 5000)
    // {
    //     expected_divided_voltage += 3;
    // }
   // expected_divided_voltage += 5; // GND opamp of xy6020L is 5 mv above arduino ground so it sees less voltage from dac
    Serial.print("   dac voltage = ");
    Serial.print(expected_divided_voltage, 6);
    uint16_t steps = expected_divided_voltage * DAC_V_TO_STEPS_RATIO;
    // 835 ---> 10 volt
    Serial.print("   steps= ");
    Serial.println(steps);
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
    if (millis() - _time_calibrate > 300)
    {
        _time_calibrate = millis();
        float volts = readVout();
        VOUT = volts;
        int16_t dif = (volts * 1000.0) - (_V_set * 1000.0);
        uint16_t absdif = fabs(dif);
        if ((absdif > 5) && (absdif < 500) && (_V_set != 0))
        {
            Serial.println("lets see if this is executed....");
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
        Serial.print("VoutSet = ");
        Serial.print(_V_set * 1000);
        Serial.print("  Voutsensed= ");
        Serial.print(volts, 3);
        Serial.print(" _V_set = ");
        Serial.print(_V_set * 1000);
        Serial.print(" volts = ");
        Serial.print(volts, 3);
        Serial.print(" absdif = ");
        Serial.print(absdif);
        Serial.print(" _DAC_V_steps = ");
        Serial.println(_DAC_V_steps);
    }
}

void PSU::work()
{
}

void PSU::enable_output(bool ON)
{
    if (ON)

        _DAC_V.setVoltage(_DAC_V_steps, false);

    else
        _DAC_V.setVoltage(0, false);
}

float PSU::MeasureTemp()
{
    static unsigned long DELAY_TEMP = millis();
    if (millis() - DELAY_TEMP < 1000)
        return -200;
    DELAY_TEMP = millis();
    const float BETA = 3950.0; // Beta coefficient of the thermistor
    const float R1 = 4300.0;   // 2 resistor of 10 k in paralell but measured gives 4.3-4.6 k
    const float R25 = 10000.0; // Resistance of R1 in ohms
    const float T25 = 298.15;  // Reference temperature in Kelvin (25°C = 298.15K)
    const float VCC = 3.3;     // Supply voltage in volts
    int16_t steps = _ADS.readADC_SingleEnded(2);
    float voltage = _ADS.computeVolts(steps);
    Serial.print("voltage = ");
    Serial.print(voltage);
    // Calculate the resistance of the NTC
    float current = (VCC - voltage) / R1;

    float resistance = voltage / current;
    Serial.print("  resistance = ");
    Serial.print(resistance);
    // Calculate temperature in Kelvin using the Beta equation
    float tempK = 1 / ((log(resistance / R25) / BETA) + (1 / T25));

    // Convert Kelvin to Celsius
    float tempC = tempK - 273.15;

    Serial.print("  being at temperature ");
    Serial.print(tempC);
    Serial.println(" degC,");
    return tempC;
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
    // voltage = roundf(voltage * 1000) / 1000.0;
    float Vout = voltage * ((R1_VOUT_SENSE + R2_VOUT_SENSE) / R2_VOUT_SENSE);
    _voltage_sensed = roundf(Vout * 1000) / 1000.0;
    VOUT = _voltage_sensed;
    return _voltage_sensed ; // since Dc ground is 20 mv above arduino ground , arduino always reads arround 20mv lower that true voltage
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
    //current -= 1;

    current = current < 0 ? 0.000 : current;
    // current = current - (vsense / _current_shuntResistor );
    _current_sensed = current;
    Serial.print("Sensed current shunt voltage=");
    Serial.println(vsense);

    return current;
}
