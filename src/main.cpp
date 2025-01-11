#include <Arduino.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>
#include <LibPrintf.h> //this interfere with ssd1306 adafruit lib!!!!!
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Cytron_SSD1306.h>
// #include <RotaryEncoder.h>
////I2C device found at address 0x48  ! ads1115
////I2C device found at address 0x61  ! current DAC
////I2C device found at address 0x60    voltage DAC
#define DAC_VCC 3327.0 // 4.88 VOLTS
#define R1_VIN 100000.0
#define R2_VIN 5100.0
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Cytron_SSD1306 oled;
Adafruit_MCP4725 DAC_V;
Adafruit_MCP4725 DAC_A;
Adafruit_ADS1115 ADS;
uint16_t V_set = 0;
uint16_t A_set = 0;
uint16_t Adc_Counter = 0;
unsigned long time = millis();
uint32_t DAC_A_steps = 0;
uint16_t DAC_V_steps = 0;
uint16_t DAC_V_STEPS_Calculated = 0;
float vout_sense = 0;
float current_sense = 0;
float adc_volts = 0;
bool calibrate = false;
void Read_display_volts();
void read_display_current();
void setup()
{
    // Wire.begin();
    // Wire.setClock(400000UL);
    Serial.begin(115200);
    pinMode(9, OUTPUT);
    oled.begin();

    // delay(2000); // Pause for 2 seconds

    oled.clear();
    // oled.setFont(Arial_bold_14);
    oled.setCursor(0, 0);
    Serial.println(F("staring..."));
    printf("Hello form printf()\n");
    DAC_A.begin(0x61);
    DAC_V.begin(0x60);
    if (!ADS.begin())
    {
        Serial.println(F("Failed to initialize ADS."));
        while (1)
            ;
    }
    ADS.setGain(GAIN_ONE);                // 1x gain   +/- 4.096V  1 bit = 0.125mV
    ADS.setDataRate(RATE_ADS1115_860SPS); // max rate 860 samples / second but it is still slow
                                          // ADS.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
                                          // oled.println(F("Voltage: "));
}

/***************************************************** */

void loop()
{
    ////********ENTER Values In mV and in mAmps***********///
    while (Serial.available() > 0)
    {
        char command = Serial.read(); // Read the command character ('V' or 'A')

        if (command == 'v') // If command is 'V', read voltage
        {
            V_set = Serial.parseFloat();                                // Read the voltage value
            double dac_value = ((153 + (V_set * 0.049400509)) * 1.051); // 153 is at 5 volts but measured is 163mv
            DAC_V_steps = (4095.0 / DAC_VCC) * dac_value;
            printf(" voltage = %d  calculated V_VDAC voltage= %f   dac STEPS= %d\n", V_set, dac_value, DAC_V_steps);
            DAC_V_STEPS_Calculated = DAC_V_steps;
            DAC_V.setVoltage(DAC_V_steps, false); // correct DAC steps error by adding 20 this is done using trial and error
        }
        else if (command == 'a') // If command is 'A', read current
        {
            A_set = Serial.parseInt();                              // Read the current value
            double dac_current_val = ((0.003333333 * A_set) + 4.8); // this is the voltage that should be fed into OPAMP (5 mv offset added from vcc)
            float voltage_out_dac_amps = (dac_current_val * (10000 + 100000 + 5100)) / 5100.0;
            DAC_A_steps = (4095.0 / DAC_VCC) * voltage_out_dac_amps;
            printf("current = %u Calculated SHUNT+OFFSET volateg= %f  voltageDAC=%f DAC OUT steps=%d\n", A_set, dac_current_val,voltage_out_dac_amps, DAC_A_steps);
            DAC_A.setVoltage(DAC_A_steps+10, false);//+40/////*/*/*
        }
        else if (command == 's') // If command is 's', print sensed voltage
        {
            printf("sensed voltage = %f  steps now = %d  adc_volts = %f  current = %f \n", vout_sense, DAC_V_steps, adc_volts, current_sense);
        }
        else if (command = 'f')
        {
            digitalWrite(9, !digitalRead(9));
        }
        else
        {
            printf("Invalid command. Use 'V' for voltage or 'A' for amps.\n");
        }
    }
    Read_display_volts();
    read_display_current();
}

/***********************************************************************************s */
void Read_display_volts(void)
{
    int16_t adc;
    uint16_t samples = 200;
    for (size_t i = 0; i < samples; i++)
    {
        adc = ADS.readADC_SingleEnded(0);
        adc_volts += ADS.computeVolts(adc);
    }
    {

        // Adc_Counter = 0;
        calibrate = true;
        vout_sense = (adc_volts / samples) * ((R1_VIN + R2_VIN) / R2_VIN);
        adc_volts = 0;

        oled.setCursor(0, 0); // Avoid overlapping text
                              // Draw 2X-scale text

        oled.println(vout_sense, 6);
        // oled.print(F("steps:"));
        oled.println(DAC_V_steps);

        // non_blocking continous reading adc voltage
    }
    if (calibrate && V_set != 0)
    {
        calibrate = false;
        int mv = (vout_sense * 1000);
        int diff = (mv - V_set);
        // printf("vout_mv=%d   diff=%d\n", vout_mv, diff);
        int steps_diff = DAC_V_STEPS_Calculated - DAC_V_steps;
        if (abs(diff) > 15 && abs((steps_diff)) < 10)
        {
            //  printf("vout sense is not calbirated vout=%d diff=%d\n", vout_mv, diff);
            if (diff > 0)
                DAC_V_steps--;
            else
                DAC_V_steps++;

            DAC_V_steps = DAC_V_steps > 4095 ? 4095 : DAC_V_steps;
            DAC_V_steps = DAC_V_steps < 0 ? 0 : DAC_V_steps;
            DAC_V.setVoltage(DAC_V_steps, false);
        }
        else if (abs(diff) > 10)
        {
            DAC_V.setVoltage(DAC_V_STEPS_Calculated, false);
        }
    }
}
void read_display_current()
{
    float mv = 0;
    int adc = 0;
    uint16_t samples = 400;
    for (size_t i = 0; i < samples; i++)
    {
        adc = ADS.readADC_SingleEnded(2);
        mv += ADS.computeVolts(adc);
    }
    current_sense = (((mv * 1000.0 / samples) / 28) - 1.454) / 3.3333333;
    oled.println(current_sense, 6);
    oled.println(mv / samples, 6);
}
