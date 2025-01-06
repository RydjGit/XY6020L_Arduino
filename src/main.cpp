#include <Arduino.h>
#include <Adafruit_MCP4725.h>
// #include <RotaryEncoder.h>
////I2C device found at address 0x48  ! ads1115
////I2C device found at address 0x61  ! current DAC
////I2C device found at address 0x60    voltage DAC
#include <LibPrintf.h>
#define DAC_VCC 4880.0  //4.88 VOLTS
void Read_serial(void);
Adafruit_MCP4725 DAC_V;
Adafruit_MCP4725 DAC_A;
uint16_t V_set = 0;
uint16_t A_set = 0;
void setup()
{
    Serial.begin(115200);
    printf("staring...\n");
    Wire.begin();
 //   Wire.setClock(400000UL);
    DAC_A.begin(0x61);
    DAC_V.begin(0x60);
}

/***************************************************** */

void loop()
{
    ////ENTER Values In mV and in mAmps
    while (Serial.available() > 0)
    {
        char command = Serial.read(); // Read the command character ('V' or 'A')

        if (command == 'V') // If command is 'V', read voltage
        {
            V_set = Serial.parseInt(); // Read the voltage value
            double dac_value = (153 + (V_set * 0.0494))*1.051 ;
            double val = (4095.0 / DAC_VCC) * dac_value;
            printf(" voltage = %d  calculated V_VDAC voltage= %f   dac STEPS= %f\n", V_set, dac_value, val);

            DAC_V.setVoltage(val, false,100000);
        }
        else if (command == 'A') // If command is 'A', read current
        {
            A_set = Serial.parseInt();                     // Read the current value
            double dac_current_val = ((0.003 * A_set) + 5); // this is the voltage that should be fed into OPAMP
            float voltage_out_dac_amps = (dac_current_val * (10000 + 100000 + 5100)) / 5100.0;
            double val = (4095.0 / DAC_VCC) * voltage_out_dac_amps;
            printf("current = %d Calculated SHUNT+OFFSET volateg= %f   DAC OUT=%f\n", A_set, dac_current_val, val);
            DAC_A.setVoltage(val, false);
        }
        else
        {
            printf("Invalid command. Use 'V' for voltage or 'A' for amps.\n");
        }
    }
}
