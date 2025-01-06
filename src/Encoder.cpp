
#include <Arduino.h>

// #define ENC_A 3
// #define ENC_B 2
unsigned long _lastIncReadTime = micros();
unsigned long _lastDecReadTime = micros();
constexpr int _pauseLength = 25000;
constexpr int _fastIncrement = 10;
unsigned long counter;


void setup_encoders(){
      // pinMode(ENC_A, INPUT_PULLUP);
    // pinMode(ENC_B, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);
    // analogReference(EXTERNAL);
    //  analogRead(0);
}

void read_encoder()
{
    // Encoder interrupt routine for both pins. Updates counter
    // if they are valid and have rotated a full indent

    static uint8_t old_AB = 3;                                                               // Lookup table index
    static int8_t encval = 0;                                                                // Encoder value
    static const int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0}; // Lookup table

    old_AB <<= 2; // Remember previous state

    //   if (digitalRead(ENC_A))
    old_AB |= 0x02; // Add current state of pin A
                    //   if (digitalRead(ENC_B))
    old_AB |= 0x01; // Add current state of pin B

    encval += enc_states[(old_AB & 0x0f)];

    // Update counter if encoder has rotated a full indent, that is at least 4 steps
    if (encval > 3)
    { // Four steps forward
        int changevalue = 1;
        if ((micros() - _lastIncReadTime) < _pauseLength)
        {
            changevalue = _fastIncrement * changevalue;
        }
        _lastIncReadTime = micros();
        counter = counter + changevalue; // Update counter
        encval = 0;
    }
    else if (encval < -3)
    { // Four steps backward
        int changevalue = -1;
        if ((micros() - _lastDecReadTime) < _pauseLength)
        {
            changevalue = _fastIncrement * changevalue;
        }
        _lastDecReadTime = micros();
        counter = counter + changevalue; // Update counter
        encval = 0;
    }
}