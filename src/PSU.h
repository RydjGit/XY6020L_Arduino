#ifndef PSU_H
#define PSU_H
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>
////I2C device found at address 0x48  ! ads1115
////I2C device found at address 0x61  ! current DAC
////I2C device found at address 0x60    voltage DAC

constexpr float DAC_VCC = 3300;
constexpr float OPAMP_VCC = 5000.0;
constexpr float R1_VOUT_SENSE = 101000.0f;
constexpr float R2_VOUT_SENSE = 5100.0f;
constexpr float CURRENT_SET_DIVIDER = (100000.0 + 10000.0 + 5100.0) / 5100.0;
constexpr float DAC_V_TO_STEPS_RATIO = 4095.0 / DAC_VCC;
constexpr float CURRENT_SENSE_OFFSET = OPAMP_VCC * (1000.0 / 3300000.0);                      // R1 =3.3 megaohm R2 =1k ohm vcc 5 volts val = !1.515 mv
constexpr float CURRENT_SET_OFFSET = OPAMP_VCC * (1000.0 / 1001000.0);                        // R1 = 1megaohm R2= 1k   val = ~ 5 mv
constexpr float VOUT_SET_OFFSET = (OPAMP_VCC * (100.0 / (3100.0))) * (100000.0 / 105100.0); // val = ~ 153 mv , out- is 30 mv above ground
constexpr float VOUT_SET_DIVIDER_RATIO = (5189.0 / 105189.0);                               // val =~ 0.0494;
constexpr float VOLTAGE_SET_GAIN = 1 + (5100.0 / 99000.0);                                    // APPROXI = 1.51
class PSU
{
private:
    float _currentGain = 28.0f;
    float _current_shuntResistor = 3.33f;
    Adafruit_MCP4725 _DAC_V;
    Adafruit_MCP4725 _DAC_A;
    Adafruit_ADS1115 _ADS;
    float _V_set = 0;
    float _A_set = 0;
    volatile uint32_t _DAC_A_steps = 0;
    volatile uint16_t _DAC_V_steps = 0;
    uint16_t _DAC_V_STEPS_Calculated = 0;
    float _voltage_sensed = 0;
    float _current_sensed = 0;
    long _time_voltage = 0;
    long _time_current = 0;
    long _time_debug = 0;
    long _time_calibrate = 0;
    long _time_work = 0;
    float _currentsum = 0;
    float _voltagesum = 0;
    unsigned long index = 0;

public:
    PSU();
    void begin();
    void debug();
    void setCurrent(float current);
    void setvoltage(float voltage);
    float readVout();
    float readCurrent();
    void calibrate();
    void work();
    void enable_output(bool ON);
    float MeasureTemp();
    float VOUT = 0;

private:
    float voltage_read();
    float current_read();
};

#endif