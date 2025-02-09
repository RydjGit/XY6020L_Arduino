#include "HT1621B.h"
#include <Arduino.h>

// Define static member variables
uint8_t HT1621B::ENC_A = 2;
uint8_t HT1621B::ENC_B = 3;
float *HT1621B::_Blinking_Value = nullptr;
volatile uint8_t HT1621B::_blink_position = 0;
HT1621B::EditingMode HT1621B::_EditingMode = EditingMode::NONE;
/*vol_curr_sym{6,0},on_off_sym{19,0},Ah_w_h_sym{7, 0},wifi_sym{27,0}*/
HT1621B::HT1621B(uint8_t _cs, uint8_t _wr, uint8_t _data)
    : link(_cs, _wr, _data),
      sym_vol_curr(),       // Calls default constructor
      sym_on_off_SCV_SET(), // Calls default constructor
      sym_Ah_w_h_SCC(),     // Calls default constructor
      sym_wifi()
{
}

void HT1621B::ReadEncoder()
{
    if (_Blinking_Value == nullptr)
        return;
    
    static uint8_t old_AB = 3;                                                               // Lookup table index
    static int8_t encval = 0;                                                                // Encoder value
    static const int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0}; // Lookup table

    old_AB <<= 2; // Remember previous state

    if (digitalRead(ENC_A))
        old_AB |= 0x02; // Add current state of pin A
    if (digitalRead(ENC_B))
        old_AB |= 0x01; // Add current state of pin B

    encval += enc_states[(old_AB & 0x0f)];

    float MAX = _EditingMode == EditingMode::Amperage ? 20.00 : 60.00;

    // Update _Blinking_Value if encoder has rotated a full indent, that is at least 4 steps
    if (encval > 3)
    { // Four steps forward

        *_Blinking_Value = *_Blinking_Value + Encoder_Multiplyer(1); // Update _Blinking_Value
        *_Blinking_Value = *_Blinking_Value < 0 ? 0 : *_Blinking_Value;
        *_Blinking_Value = *_Blinking_Value > MAX ? MAX : *_Blinking_Value;
        encval = 0;
    }
    else if (encval < -3)
    { // Four steps backward

        *_Blinking_Value = *_Blinking_Value + Encoder_Multiplyer(-1); // Update _Blinking_Value
        *_Blinking_Value = *_Blinking_Value < 0 ? 0 : *_Blinking_Value;
        *_Blinking_Value = *_Blinking_Value > MAX ? MAX : *_Blinking_Value;
        encval = 0;
    }
}

float HT1621B::Encoder_Multiplyer(int value)
{
    switch (_blink_position)
    {
    case 0:
        return value / 100.0;
    case 1:
        return value / 10.0;
    case 2:
        return value;
    case 3:
        return value * 10.0;
    default:
        break;
    }
    return value;
}

void HT1621B::print_voltage(float v)
{
    _voltage_reading = v;
    uint16_t K = v * 100;

    for (int i = 0; i < 4; i++)
    {
        VoltageDigits[i].data = seven_segment[K % 10];
        if (i == 2)
        {
            VoltageDigits[i].data |= 0x01;
        }

        K /= 10;
        link.SendByte(VoltageDigits[i].address, VoltageDigits[i].data);
    }
}

void HT1621B::print_current(float current)
{
    _amperage_reading = current;
    uint16_t K = current * 100;

    for (int i = 0; i < 4; i++)
    {
        AmerageDigits[i].data = seven_segment[K % 10];
        if (i == 2)
        {
            AmerageDigits[i].data |= 0x01;
        }

        K /= 10;
        link.SendByte(AmerageDigits[i].address, AmerageDigits[i].data);
    }
}

void HT1621B::print_power(float power, bool is_tempreture)
{
    _power_reading = power;
    uint16_t K = power * 100;

    for (int i = 0; i < 4; i++)
    {
        PowerDigits[i].data = seven_segment[K % 10];
        if (i == 2)
        {
            PowerDigits[i].data |= 0x01;
        }

        K /= 10;
        link.SendByte(PowerDigits[i].address, PowerDigits[i].data);
    }
}

void HT1621B::begin()
{
    link.begin();
    link.SendNibble(sym_on_off_SCV_SET.address, sym_on_off_SCV_SET.data);
    link.SendNibble(sym_vol_curr.address, sym_vol_curr.data);
    link.SendNibble(sym_Ah_w_h_SCC.address, sym_Ah_w_h_SCC.data);
    link.SendNibble(sym_wifi.address, sym_wifi.data);
}

void HT1621B::output_on_off(bool on_off)
{
    sym_on_off_SCV_SET.ON = on_off;
    sym_on_off_SCV_SET.OFF = !sym_on_off_SCV_SET.ON;
}

void HT1621B::Advance_Blink_Position()
{
    _blink_position++;
    _blink_position = _blink_position > 3 ? 0 : _blink_position;
}

void HT1621B::blink()
{
    if (!_is_editing)
    {
        return;
    }

    static bool blink_state = false;

    if (millis() - blink_duration >= 500) // Toggle every 500ms
    {
        blink_duration = millis();
        blink_state = !blink_state;
    }

    if (blink_state)
    {
        print_power(*_Blinking_Value); // Show the digit
    }
    else
    {
        Digit D = PowerDigits[_blink_position];
        D.data = 0x00; // Clear the digit
        link.SendByte(D.address, D.data);
    }
}

// void HT1621B::Set_EncoderData(float *_cnt)
// {
//     _Blinking_Value = _cnt;
// }

void HT1621B::TggoleEditingMode(EditingMode mode, float *blinking_val)
{
    // static volatile int mode = 0; // 0: Editing Off, 1: Voltage, 2: Amperage
    // mode = (mode + 1) % 3;        // Cycle through 0 → 1 → 2 → 0
    _Blinking_Value = blinking_val;
    if (mode == 0)
    {
        _is_editing = false;
        sym_on_off_SCV_SET.SET = 0;
        sym_on_off_SCV_SET.SET_CV = 0;
        sym_Ah_w_h_SCC.SET_CC = 0;
        _blink_position = 0;
    }
    else if (mode == 1)
    {
        _is_editing = true;
        _EditingMode = EditingMode::Voltage;
        sym_on_off_SCV_SET.SET = 1;
        sym_on_off_SCV_SET.SET_CV = 1;
        sym_Ah_w_h_SCC.SET_CC = 0;
    }
    else
    {
        _is_editing = true;
        _EditingMode = EditingMode::Amperage;
        sym_on_off_SCV_SET.SET = 1;
        sym_on_off_SCV_SET.SET_CV = 0;
        sym_Ah_w_h_SCC.SET_CC = 1;
    }
}
void HT1621B::setupEncoder(uint8_t EncA, uint8_t EncB)
{

    ENC_A = EncA;
    ENC_B = EncB;
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), ReadEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), ReadEncoder, CHANGE);
}
bool HT1621B::Get_editingStatus()
{
    return _is_editing;
}

void HT1621B::Update()
{
    link.SendNibble(sym_Ah_w_h_SCC.address, sym_Ah_w_h_SCC.data);
    link.SendNibble(sym_on_off_SCV_SET.address, sym_on_off_SCV_SET.data);
    link.SendNibble(sym_vol_curr.address, sym_vol_curr.data);
    link.SendNibble(sym_wifi.address, sym_wifi.data);
}
