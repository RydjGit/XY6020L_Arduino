#include <Arduino.h>
#include <Wire.h>
#include "PSU.h"
#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <HT1621B.h>
/***************************************************** */
#define BTN_SW 12
#define CS 11      // Chip Select
#define WR 10      // Write Clock
#define DATA_pin 9 // Data Line
#define Fan_pin 8
#define BTN_VA 7
#define LCD_BACKLIGHT 6
#define BTN_ROTAT 5
#define BTN_POWER 4
#define enc_B 3
#define enc_A 2
/********************************************************* */
volatile bool OUTPU_ENABLED = true;
/********************************************************* */
float Voltage = 5.00;
float Current = 1.00;
volatile bool Editing = false;
/********************************************************* */
HT1621B LCD(CS, WR, DATA_pin);
PSU psu;
/********************************************************* */
void SW_Pressed();
void VA_Pressed();
void ROTAT_pressed();
void POWER_Pressed();
void setupTimer();
/********************************************************* */

void setup()
{
    Serial.begin(115200);

    pinMode(CS, OUTPUT);
    pinMode(WR, OUTPUT);
    pinMode(DATA_pin, OUTPUT);
    pinMode(LCD_BACKLIGHT, OUTPUT);

    pinMode(BTN_ROTAT, INPUT_PULLUP);
    pinMode(BTN_POWER, INPUT_PULLUP);
    pinMode(BTN_SW, INPUT_PULLUP);
    pinMode(BTN_VA, INPUT_PULLUP);
    // IMPORATNAT: after pin initialisation CS WR DATA
    LCD.begin();

    attachPCINT(digitalPinToPCINT(BTN_POWER), POWER_Pressed, FALLING);
    attachPCINT(digitalPinToPCINT(BTN_ROTAT), ROTAT_pressed, FALLING);
    attachPCINT(digitalPinToPCINT(BTN_VA), VA_Pressed, FALLING);
    attachPCINT(digitalPinToPCINT(BTN_SW), SW_Pressed, FALLING);

    LCD.setupEncoder(enc_A, enc_B);

    digitalWrite(LCD_BACKLIGHT, HIGH);

    Serial.println("XY6020L Firmware by Dr.Ra'ed Jaradat ......   :)");
    psu.begin();
    Serial.println("XY6020L Firmware by Dr.Ra'ed Jaradat ......   :)");
}

/***************************************************** */
unsigned long updatetimer = millis();
void loop()
{
    psu.enable_output(OUTPU_ENABLED);

    if (millis() - updatetimer > 200)
    {
        float V = psu.readVout();
        float A = psu.readCurrent();
        LCD.print_voltage(psu.readVout());
        LCD.print_current(psu.readCurrent());
        if (!LCD.Get_editingStatus())
        {
            LCD.print_power(V * A);
        }
        updatetimer = millis();
    }

    if (Editing && OUTPU_ENABLED)
    {
        psu.setvoltage(Voltage);
        psu.setCurrent(Current);
    }

    LCD.Update();
    LCD.blink();
}

/*********************************************************************************** */
static volatile long debounce = millis();
#define BEDOUNCE_DURATION 250

void SW_Pressed()
{
    if ((millis() - debounce) < BEDOUNCE_DURATION)
    {
        return;
    }

    debounce = millis();
}
void VA_Pressed()
{
    if ((millis() - debounce) < BEDOUNCE_DURATION)
    {
        return;
    }
    debounce = millis();
    static volatile int mode = 0; // 0: Editing Off, 1: Voltage, 2: Amperage
    mode = (mode + 1) % 3;        // Cycle through 0 → 1 → 2 → 0
    switch (mode)
    {
    case 0:
        LCD.TggoleEditingMode(HT1621B::EditingMode::NONE, nullptr);
        Editing = false;
        break;

    case 1:
        LCD.TggoleEditingMode(HT1621B::EditingMode::Voltage, &Voltage);
        Editing = true;
        break;

    case 2:
        LCD.TggoleEditingMode(HT1621B::EditingMode::Amperage, &Current);
        Editing = true;
        break;
    default:
        break;
    }
}

void ROTAT_pressed()
{
    if ((millis() - debounce) < BEDOUNCE_DURATION)
    {
        return;
    }
    debounce = millis();

    if (LCD.Get_editingStatus())
    {
        LCD.Advance_Blink_Position();
    }
}

void POWER_Pressed()
{
    if ((millis() - debounce) < BEDOUNCE_DURATION)
    {
        return;
    }
    debounce = millis();
    static bool on_off = true;
    on_off = !on_off;
    LCD.output_on_off(on_off);
    OUTPU_ENABLED = !OUTPU_ENABLED;
}