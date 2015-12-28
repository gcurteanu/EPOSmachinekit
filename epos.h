/*
    EPOS error table
*/

#include <data.h>

typedef struct {
    UNS16           error_code;
    const char *    message;
} epos_error_t;

extern epos_error_t epos_error_table[];

const char * epos_error_text (UNS16 errCode);

#ifndef EPOS_ERROR_TABLE
#define EPOS_ERROR_TABLE

epos_error_t epos_error_table[] = {
    {0x1000, "Generic error"},
    {0x2310, "Over Current error"},
    {0x3210, "Over Voltage error"},
    {0x3220, "Under Voltage error"},
    {0x4210, "Over Temperature error"},
    {0x5113, "Supply voltage (+5V) too low"},
    {0x6100, "Internal software error"},
    {0x6320, "Software Parameter error"},
    {0x7320, "Sensor Position error"},
    {0x8110, "CAN Overrun Error (Objects lost)"},
    {0x8111, "CAN Overrun Error"},
    {0x8120, "CAN Passive Mode Error"},
    {0x8130, "CAN Life Guard Error"},
    {0x8150, "CAN Transmit COB-ID collision"},
    {0x81FD, "CAN Bus Off"},
    {0x81FE, "CAN Rx Queue Overrun"},
    {0x81FF, "CAN Tx Queue Overrun"},
    {0x8210, "CAN PDO length Error"},
    {0x8611, "Following Error"},
    {0xFF01, "Hall Sensor Error"},
    {0xFF02, "Index Processing Error"},
    {0xFF03, "Encoder Resolution Error"},
    {0xFF04, "Hallsensor not found Error"},
    {0xFF06, "Negative Limit Error"},
    {0xFF07, "Positive Limit Error"},
    {0xFF08, "Hall Angle detection Error"},
    {0xFF09, "Software Position Limit Error"},
    {0xFF0A, "Position Sensor Breach"},
    {0xFF0B, "System Overloaded"}
};

const char * epos_error_text (UNS16 errCode) {
    int i;
    
    for (i = 0; i < sizeof(epos_error_table) / sizeof(epos_error_t); i++)
        if (epos_error_table[i].error_code == errCode)
            return epos_error_table[i].message;
        
    return "(unknown)";
}

#endif