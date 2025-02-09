

#include "comm.h"

/*non class function*/
static unsigned char swapNibbles(unsigned char x)
{
    return (x << 4) | (x >> 4);
}
/*class definitions*/
/****************************************************************************** */
void comm::sendBit(bool bitVal)
{
    digitalWrite(_wr, LOW);
    delayMicroseconds(4);
    digitalWrite(_data, bitVal);
    delayMicroseconds(4);
    digitalWrite(_wr, HIGH);
}

void comm::SendByte(uint8_t address, uint8_t data)
{
    digitalWrite(_cs, LOW);

    // Send 3-bit write prefix (101)
    sendBit(1);
    sendBit(0);
    sendBit(1);

    for (int i = 5; i >= 0; i--)
    {
        sendBit(bitRead(address, i));
    }

    if (address < 6)
    {
        data = swapNibbles(data);
    }

    for (int i = 0; i < 8; i++)
    {
        sendBit(bitRead(data, i));
    }

    digitalWrite(_cs, HIGH);
}

void comm::SendNibble(uint8_t address, uint8_t data)
{
    digitalWrite(_cs, LOW);

    // Send 3-bit write prefix (101)
    sendBit(1);
    sendBit(0);
    sendBit(1);

    for (int i = 5; i >= 0; i--)
    {
        sendBit(bitRead(address, i));
    }

    if (address < 6)
    {
        data = swapNibbles(data);
    }

    for (int i = 0; i < 4; i++)
    {
        sendBit(bitRead(data, i));
    }

    digitalWrite(_cs, HIGH);
}

void comm::SendChainData(uint8_t startadd, uint8_t *data, uint8_t size)
{
    digitalWrite(_cs, LOW);

    // Send 3-bit write prefix (101)
    sendBit(1);
    sendBit(0);
    sendBit(1);

    for (int i = 5; i >= 0; i--)
    {
        sendBit(bitRead(startadd, i));
    }

    for (int i = 0; i < size; i++)
    {
        for (size_t o = 0; o < 8; o++)
        {
            sendBit(bitRead(data[i], o));
        }
    }
    digitalWrite(_cs, HIGH);
}

void comm::SendCommand(uint8_t command)
{
    digitalWrite(_cs, LOW);

    // Send 3-bit command prefix (100)
    sendBit(1);
    sendBit(0);
    sendBit(0);
    for (int i = 7; i >= 0; i--)
    {
        sendBit(bitRead(command, i));
    }
    sendBit(0); // Ensure 9th bit is correctly handled
    digitalWrite(_cs, HIGH);
}

comm::comm(uint8_t Cs,
           uint8_t Rw,
           uint8_t _Data) : _cs(Cs), _wr(Rw), _data(_Data)
{
}

comm::~comm()
{
}

void comm::begin()
{
    // oRder of these commands is important
    SendCommand(0x18);               // RC 256K (Use on-chip 256kHz oscillator)
    SendCommand(0x0A);               // WDT DIS (Disable watchdog timer)
    SendCommand(BIAS_1_3_4_COMMONS); // 1/3duty 4com
    SendCommand(0x04);               // disable time based output
    SendCommand(0x01);               // SYS EN (Enable system oscillator)
    clear_screen();
    SendCommand(0x03); // LCD ON (Turn on LCD)
}

void comm::clear_screen()
{
    for (uint8_t addr = 0; addr <= 0b011111; addr++)
    {
        SendByte(addr, 0x00);
    }
}
