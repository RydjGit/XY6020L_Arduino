#include <Arduino.h>
#include <Libprintf.h>
#include <U8x8lib.h>
#include <U8g2lib.h>
#include "PSU.h"
// #include <RotaryEncoder.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);
PSU psu;

void Read_display_volts();
void read_display_current();
void setup()
{
    // Wire.begin();
    // Wire.setClock(400000UL);
    Serial.begin(115200);
    Serial.println(F("staring..."));
    printf("Hello form printf()\n");
    pinMode(9, OUTPUT);
    oled.begin();
    oled.setFont(u8x8_font_pxplustandynewtv_n); // Choose a font
    psu.begin();
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
            float V_set = Serial.parseFloat(); // Read the voltage value
            psu.setvoltage(V_set);
        }
        else if (command == 'a') // If command is 'A', read current
        {
            float A_set = Serial.parseFloat();
            psu.setCurrent(A_set);
        }
        else if (command == 'd') // If command is 's', print sensed voltage
        {
            psu.debug();
        }
        else if (command == 'f')
        {
            digitalWrite(9, !digitalRead(9));
        }
        else
        {
            printf("Invalid command. Use 'V' for voltage or 'A' for amps.\n");
        }
    }
    psu.work();
    psu.calibrate();
    Read_display_volts();
    read_display_current();
    psu.debug();
}

/***********************************************************************************s */
void Read_display_volts(void)
{
    oled.setCursor(0, 0);
    float vout = psu.readVout();
    if (vout != -1)
    {
        oled.println(vout, 6);
    }

    // oled.print(vout);
    // oled.println(DAC_V_steps);
}
void read_display_current()
{
    oled.setCursor(0, 5);
    float current = psu.readCurrent();
    if (current != -1)
    {
        oled.println(current, 6);
    }

    //  oled.println(mv / samples, 6);
}
