#pragma once

#include <comm.h>
#include <symbols.h>

const uint8_t seven_segment[10] = {
    0b11111010, // 0 (0xFA) → Segments A, F, E, D, B, C ON
    0b00001010, // 1 (0x0A) → Segments B, C ON
    0b10111100, // 2 (0xBC) → Segments A, B, G, E, D ON
    0b10011110, // 3 (0x9E) → Segments A, B, G, C, D ON
    0b01001110, // 4 (0x4E) → Segments F, G, B, C ON
    0b11010110, // 5 (0xD6) → Segments A, F, G, C, D ON
    0b11110110, // 6 (0xF6) → Segments A, F, G, C, D, E ON
    0b10001010, // 7 (0x8A) → Segments A, B, C ON
    0b11111110, // 8 (0xFE) → All segments ON
    0b11011110  // 9 (0xDE) → Segments A, B, C, D, F, G ON
};

class HT1621B
{
public:
    enum EditingMode
    {
        NONE,
        Voltage,
        Amperage
    };
    // Define a struct to hold the extracted digits and decimal point location

private:
    comm link;
    Digit PowerDigits[4]{18, 0, 20, 0, 22, 0, 24, 0};
    Digit VoltageDigits[4]{10, 0, 12, 0, 14, 0, 16, 0};
    Digit AmerageDigits[4]{4, 0, 2, 0, 0, 0, 8, 0};
    static uint8_t ENC_A;
    static uint8_t ENC_B;
    static float *_Blinking_Value;
    static volatile uint8_t _blink_position;
    static EditingMode _EditingMode;

    float _voltage_reading = 0;
    float _amperage_reading = 0;
    float _power_reading = 0;
    volatile bool _is_editing = false;
    volatile unsigned long blink_duration = millis();

    voltage_current sym_vol_curr;
    CV_SET_OFF_ON sym_on_off_SCV_SET;
    SCC_AH_W_H sym_Ah_w_h_SCC;
    WIFI_SOUND_LOCK_IN sym_wifi;

private:
    static void ReadEncoder();
    void setupTimer();
    static float Encoder_Multiplyer(int value);

public:
    void print_voltage(float v);
    void print_current(float current, bool is_temp);
    void print_power(float power);
    void begin();
    void setupEncoder(uint8_t EncA, uint8_t EncB);
    // void Set_EncoderData(float *_cnt);
    void output_on_off(bool on_off);
    void display_tempreture(float temp);
    void Advance_Blink_Position();
    void blink();
    void TggoleEditingMode(EditingMode mode, float *blinking_val);
    bool Get_editingStatus();
    void Update();
    void Calibrating_symbole( );
    void Detect_CC();
public:
    HT1621B(uint8_t _cs, uint8_t _wr, uint8_t _data);
    HT1621B() = delete;
    ~HT1621B() = default;
};
