#include <Arduino.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>
#include <LibPrintf.h>
// #include <RotaryEncoder.h>
////I2C device found at address 0x48  ! ads1115
////I2C device found at address 0x61  ! current DAC
////I2C device found at address 0x60    voltage DAC
#define DAC_VCC 3300.0 // 4.88 VOLTS
#define R1_VIN 100000.0
#define R2_VIN 5100.0
void Read_serial(void);
Adafruit_MCP4725 DAC_V;
Adafruit_MCP4725 DAC_A;
Adafruit_ADS1115 ADS;
uint16_t V_set = 0;
uint16_t A_set = 0;
void setup()
{
    Serial.begin(115200);
    printf("staring...\n");
    Wire.begin();
    Wire.setClock(400000UL);
    DAC_A.begin(0x61);
    DAC_V.begin(0x60);
    if (!ADS.begin())
    {
        Serial.println("Failed to initialize ADS.");
        while (1)
            ;
    }
    ADS.setGain(GAIN_ONE);                // 1x gain   +/- 4.096V  1 bit = 0.125mV
    ADS.setDataRate(RATE_ADS1115_860SPS); // max rate 860 samples / second but it is still slow
    ADS.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
}

/***************************************************** */
unsigned long time = millis();
uint32_t DAC_A_steps = 0;
uint16_t DAC_V_steps = 0;
double vout_sense = 0;
double adc_volts = 0;
void loop()
{
    ////********ENTER Values In mV and in mAmps***********///
    while (Serial.available() > 0)
    {
        char command = Serial.read(); // Read the command character ('V' or 'A')

        if (command == 'v') // If command is 'V', read voltage
        {
            V_set = Serial.parseFloat();                                 // Read the voltage value
            double dac_value = ((158 + (V_set * 0.0494)) * 1.051) - 3; // 153 is at 5 volts but measured is 163mv
            DAC_V_steps = (4095.0 / DAC_VCC) * dac_value;
            DAC_V_steps = DAC_V_steps;
            printf(" voltage = %d  calculated V_VDAC voltage= %f   dac STEPS= %d\n", V_set, dac_value, DAC_V_steps);

            DAC_V.setVoltage(DAC_V_steps, false); // correct DAC steps error by adding 20 this is done using trial and error
        }
        else if (command == 'a') // If command is 'A', read current
        {
            A_set = Serial.parseInt();                      // Read the current value
            double dac_current_val = ((0.003 * A_set) + 5); // this is the voltage that should be fed into OPAMP (5 mv offset added from vcc)
            float voltage_out_dac_amps = (dac_current_val * (10000 + 100000 + 5100)) / 5100.0;
            DAC_A_steps = (4095.0 / DAC_VCC) * voltage_out_dac_amps;
            printf("current = %u Calculated SHUNT+OFFSET volateg= %f   DAC OUT steps=%d\n", A_set, dac_current_val, DAC_A_steps);
            DAC_A.setVoltage(DAC_A_steps, false);
        }
        else if (command == 's') // If command is 's', print sensed voltage
        {
            printf("sensed voltage = %f  steps now = %d  adc_volts = %f \n", vout_sense, DAC_V_steps, adc_volts);
        }
        else
        {
            printf("Invalid command. Use 'V' for voltage or 'A' for amps.\n");
        }
    }
    if (ADS.conversionComplete())
    {
        uint16_t adc = 0;
        adc = ADS.getLastConversionResults();
        adc_volts = ADS.computeVolts(adc);
        vout_sense = adc_volts * (R1_VIN + R2_VIN) / R2_VIN;
        ADS.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false); // non_blocking continous reading adc voltage
    }
    if (millis() - time > 100 && V_set != 0)
    {
        uint16_t current_step = DAC_V_steps;
        time = millis();
        int vout_mv = (vout_sense * 1000);
        int diff = (vout_mv - V_set);
        // printf("vout_mv=%d   diff=%d\n", vout_mv, diff);

        if (abs(diff) > 10)
        {
            //  printf("vout sense is not calbirated vout=%d diff=%d\n", vout_mv, diff);
            if (diff > 0)
                DAC_V_steps--;
            else
                DAC_V_steps++;

            DAC_V_steps = DAC_V_steps > 4095 ? 4095 : DAC_V_steps;
            DAC_V_steps = DAC_V_steps < 0 ? 0 : DAC_V_steps;
            if (current_step != DAC_V_steps)
                DAC_V.setVoltage(DAC_V_steps, false);
        }
    }
}
