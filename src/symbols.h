struct Digit
{
   const uint8_t address;
    uint8_t data;
};

struct voltage_current
{
    uint8_t address;
    union
    {
        uint8_t data;
        struct
        {
            uint8_t CV : 1;
            uint8_t V : 1;
            uint8_t CC : 1;
            uint8_t A : 1;
        };
    };

    voltage_current() : address(6), data(0b1011) {}
};

struct SCC_AH_W_H
{
    uint8_t address;
    union
    {
        uint8_t data;
        struct
        {
            uint8_t SET_CC : 1;
            uint8_t AH : 1;
            uint8_t W : 1;
            uint8_t H : 1;
        };
    };
    /*H_W_AH_CC*/
    SCC_AH_W_H() : address(7), data(0b0100) {}
};

struct WIFI_SOUND_LOCK_IN
{
    uint8_t address;
    union
    {
        uint8_t data;
        struct
        {
            uint8_t WIFI : 1;
            uint8_t SOUND : 1;
            uint8_t LOCK : 1;
            uint8_t IN : 1;
        };
    };

    WIFI_SOUND_LOCK_IN() : address(27), data(0) {}
};

struct CV_SET_OFF_ON
{
    uint8_t address;
    union
    {
        uint8_t data;
        struct
        {
            uint8_t SET_CV : 1;
            uint8_t SET : 1;
            uint8_t OFF : 1;
            uint8_t ON : 1;
        };
    };

    CV_SET_OFF_ON() : address(26), data(0b1000) {}
};
