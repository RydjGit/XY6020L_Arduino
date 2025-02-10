#include <Arduino.h>
#include <Wire.h>
#include "PSU.h"
#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <HT1621B.h>
/***************************************************** */
#define PIN_CC A7
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
volatile bool CALIBRATE = false;
volatile bool showTemperature = false;
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
void setupTimer1Interrupt();
/********************************************************* */
extern void Measure_temp(Adafruit_ADS1115 &ads);
/********************************************************* */

void setup()
{
    Serial.begin(115200);
    pinMode(PIN_CC, INPUT);

    pinMode(CS, OUTPUT);
    pinMode(WR, OUTPUT);
    pinMode(DATA_pin, OUTPUT);
    pinMode(LCD_BACKLIGHT, OUTPUT);

    pinMode(BTN_ROTAT, INPUT_PULLUP);
    pinMode(BTN_POWER, INPUT_PULLUP);
    pinMode(BTN_SW, INPUT_PULLUP);
    pinMode(BTN_VA, INPUT_PULLUP);

    pinMode(Fan_pin, OUTPUT);
    digitalWrite(Fan_pin, LOW);
    // IMPORATNAT: after pin initialisation CS WR DATA
    LCD.begin();

    attachPCINT(digitalPinToPCINT(BTN_POWER), POWER_Pressed, FALLING);
    attachPCINT(digitalPinToPCINT(BTN_ROTAT), ROTAT_pressed, FALLING);
    attachPCINT(digitalPinToPCINT(BTN_VA), VA_Pressed, FALLING);
    attachPCINT(digitalPinToPCINT(BTN_SW), SW_Pressed, FALLING);

    LCD.setupEncoder(enc_A, enc_B);

    digitalWrite(LCD_BACKLIGHT, HIGH);

    setupTimer1Interrupt();
    Serial.println("XY6020L Firmware by Dr.Ra'ed Jaradat ......   :)");
    psu.begin();
    Serial.println("XY6020L Firmware by Dr.Ra'ed Jaradat ......   :)");
}

/***************************************************** */
unsigned long updatetimer = millis();
float temp;
void loop()
{

    if (millis() - updatetimer > 100)
    {
        float V = psu.readVout();
        float A = psu.readCurrent();
        LCD.print_voltage(V);
        LCD.print_current(A, showTemperature && !LCD.Get_editingStatus());
        if (!LCD.Get_editingStatus())
        {
            if (!showTemperature)
            {
                LCD.print_power(V * A);
            }
        }
        updatetimer = millis();
    }

    if (Editing && OUTPU_ENABLED && !CALIBRATE)
    {
        psu.setvoltage(Voltage);
        psu.setCurrent(Current);
    }

    LCD.Update();
    LCD.blink();
    if (CALIBRATE)
    {
        psu.calibrate();
    }
    else
    {
        psu.enable_output(OUTPU_ENABLED);
    }
    LCD.Detect_CC();
    if (showTemperature && !LCD.Get_editingStatus())
    {
        temp = psu.MeasureTemp();
        LCD.display_tempreture(temp);
    }
    digitalWrite(Fan_pin, temp > 30 ? HIGH : LOW);
}

/*********************************************************************************** */
static volatile long debounce = millis();
#define BEDOUNCE_DURATION 150

void SW_Pressed()
{
    if ((millis() - debounce) < BEDOUNCE_DURATION)
    {
        return;
    }
    debounce = millis();
    LCD.Calibrating_symbole();
    CALIBRATE = !CALIBRATE;
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
void setupTimer1Interrupt()
{
    cli(); // Disable global interrupts

    TCCR1A = 0;                          // Normal mode
    TCCR1B = 0;                          // Clear registers
    TCCR1B |= (1 << WGM12);              // CTC mode (Clear Timer on Compare Match)
    TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
    OCR1A = 31248;                       // (16MHz / 1024) = 15625 ticks per second (2 sec → 31248, but ~1sec timing here)
    TIMSK1 |= (1 << OCIE1A);             // Enable Timer1 compare interrupt

    sei(); // Enable global interrupts
}

// Interrupt Service Routine (ISR) for Timer1
static unsigned int displayCounter = 0; // Increment counter every second

ISR(TIMER1_COMPA_vect)
{
    displayCounter++;

    if (displayCounter >= 7)
    { // Reset every 5 seconds
        displayCounter = 0;
    }

    if (displayCounter < 2)
    { // Show temperature for first 2 seconds
        showTemperature = true;
    }
    else
    { // Show power for remaining 3 seconds
        showTemperature = false;
    }
}