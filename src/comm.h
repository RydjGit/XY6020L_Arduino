#pragma once

#include <Arduino.h>

#define BIAS_1_2_2_COMMONS 0x20 // ab=00, 2 commons option
#define BIAS_1_2_3_COMMONS 0x24 // ab=01, 3 commons option
#define BIAS_1_2_4_COMMONS 0x28 // ab=10, 4 commons option

// BIAS 1/3 LCD bias options (common selection)
#define BIAS_1_3_2_COMMONS 0x21 // ab=00, 2 commons option
#define BIAS_1_3_3_COMMONS 0x25 // ab=01, 3 commons option
#define BIAS_1_3_4_COMMONS 0x29 // ab=10, 4 commons option

class comm
{
private:
    void sendBit(bool bitVal);

    void SendCommand(uint8_t command);
    uint8_t _cs;
    uint8_t _wr;
    uint8_t _data;

public:
    comm(uint8_t CS,
         uint8_t RW,
         uint8_t Data);
    void SendByte(uint8_t address, uint8_t data);
    void SendNibble(uint8_t address, uint8_t data);
    void SendChainData(uint8_t startadd,uint8_t* data,uint8_t size);
    ~comm();
    void begin();
    void clear_screen();
};
