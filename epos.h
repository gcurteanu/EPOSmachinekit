/*
    EPOS error table
*/

#include <data.h>

typedef struct {
    UNS16           error_code;
    const char *    message;
    const char *    error_description;
} epos_error_t;

extern epos_error_t epos_error_table[];

const char * epos_error_text (UNS16 errCode);

#ifndef EPOS_ERROR_TABLE
#define EPOS_ERROR_TABLE

epos_error_t epos_error_table[] = {
    {0x0000, "No error", "No error is present"},
    {0x1000, "Generic error", "Unspecific error occurred"},
    {0x2310, "Over Current error", "Short circuit in the motor winding\n\
Power supply can not supply enough acceleration current\n\
Too high Controller Gains (Velocity control parameter set, Position control parameter set)\n\
Profile acceleration and/or Profile deceleration too high\n\
Damaged power stage"},
    {0x3210, "Over Voltage error", "The power supply voltage is too high"},
    {0x3220, "Under Voltage error", "The supply voltage is too low for operation.\n\
The power supply can’t supply the acceleration current"},
    {0x4210, "Over Temperature error", "The temperature at the device power stage is too high (only on EPOS 24/5,\n\
EPOS 70/10 and MCD EPOS 60 W)"},
    {0x5113, "Supply voltage (+5V) too low", "There is a overload on internal generated 5V supply by the hall sensor connector or\n\
encoder connector (only on EPOS 24/5)"},
    {0x6100, "Internal software error", "Internal software error occurred"},
    {0x6320, "Software Parameter error", "Too high Target position with too low Profile velocity"},
    {0x7320, "Sensor Position error", "The detected position from position sensor is no longer valid in case of:\n\
- Changed Position Sensor Parameters\n\
- Wrong Position Sensor Parameters\n\
- Other Errors which influences the absolute position detection (Hall Sensor Error, Encoder Index Error, ...)"},
    {0x8110, "CAN Overrun Error (Objects lost)",""},
    {0x8111, "CAN Overrun Error", ""},
    {0x8120, "CAN Passive Mode Error", ""},
    {0x8130, "CAN Life Guard Error", ""},
    {0x8150, "CAN Transmit COB-ID collision", ""},
    {0x81FD, "CAN Bus Off", ""},
    {0x81FE, "CAN Rx Queue Overrun", ""},
    {0x81FF, "CAN Tx Queue Overrun", ""},
    {0x8210, "CAN PDO length Error", ""},
    {0x8611, "Following Error", ""},
    {0xFF01, "Hall Sensor Error", ""},
    {0xFF02, "Index Processing Error", ""},
    {0xFF03, "Encoder Resolution Error", ""},
    {0xFF04, "Hallsensor not found Error", ""},
    {0xFF06, "Negative Limit Error", ""},
    {0xFF07, "Positive Limit Error", ""},
    {0xFF08, "Hall Angle detection Error", ""},
    {0xFF09, "Software Position Limit Error", ""},
    {0xFF0A, "Position Sensor Breach", ""},
    {0xFF0B, "System Overloaded", ""}
};

const char * epos_error_text (UNS16 errCode) {
    int i;
    
    for (i = 0; i < sizeof(epos_error_table) / sizeof(epos_error_t); i++)
        if (epos_error_table[i].error_code == errCode)
            return epos_error_table[i].message;
        
    return "(unknown)";
}

#endif