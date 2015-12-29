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

#define MAX_EPOS_DRIVES     5
#define EPOS_MAX_ERRORS     32

typedef enum {
    EPOS_START      = 0x0000,
    EPOS_NOTREADY   = 0x0100,
    EPOS_SOD        = 0x0140,
    EPOS_RSO        = 0x0121,
    EPOS_SWO        = 0x0123,
    EPOS_REFRESH    = 0x4123,
    EPOS_MEASURE    = 0x4133,
    EPOS_OPEN       = 0x0137,
    EPOS_QUICKS     = 0x0117,
    EPOS_FRAD       = 0x010F,
    EPOS_FRAE       = 0x011F,
    EPOS_FAULT      = 0x0108,   
} EPOS_DS402_state_t;

typedef struct {
    
    // the list of the EPOS slaves we control
    UNS8        epos_slaves[MAX_EPOS_DRIVES];
    UNS8        epos_slave_count;
    
    // holds the DCF data for the nodes
    dcfset_t    dcf_data;
    
    // holds the drive errors
    UNS32       slave_err[MAX_EPOS_DRIVES][EPOS_MAX_ERRORS+1];
    
    CO_Data*    d;
    
    // enabled or disabled
    char        drive_enabled;
    // initialised or not
    char        drive_initialised;
    // state of the drive
    char        drive_state;

    // EPOS_State is the slave state. Used internally by the drive
    UNS16       EPOS_State[MAX_EPOS_DRIVES];
        
    // CNC interface
    char        enable[MAX_EPOS_DRIVES];         // true - drive enabled, false - drive disabled
    int         control_type[MAX_EPOS_DRIVES];   // 0 - position control, 1 - velocity control
    int         maxvel[MAX_EPOS_DRIVES];	     // maximal drive velocity
    int         maxaccel[MAX_EPOS_DRIVES];       // maximal drive acceleration

    int         counts[MAX_EPOS_DRIVES];         // raw encoder / position from the drive
    float       position_scale[MAX_EPOS_DRIVES]; // scale for positioning. position = counts / position-scale

    float       position_cmd[MAX_EPOS_DRIVES];   // position command (for position control)
    float       velocity_cmd[MAX_EPOS_DRIVES];   // velocity command (for velocity control)

    float       position_fb[MAX_EPOS_DRIVES];    // position feedback
    float       velocity_fb[MAX_EPOS_DRIVES];    // velocity feedback

    char        fault[MAX_EPOS_DRIVES];          // drive is faulted

    // extra stuff

    char        home[MAX_EPOS_DRIVES];           // set to high to initialise homing
    char        homing_done[MAX_EPOS_DRIVES];    // set to high when drive is homed

    char        lock[MAX_EPOS_DRIVES];           // engage the lock brake
    char        is_locked[MAX_EPOS_DRIVES];      // lock brake is engaged and axis is confirmed locked

    // GPIO pins
    char        home_sw_pin[MAX_EPOS_DRIVES];    // home switch pin
    char        enable_pin[MAX_EPOS_DRIVES];     // enable pin
    char        in_a_pin[MAX_EPOS_DRIVES];       // IN A
    char        in_b_pin[MAX_EPOS_DRIVES];       // IN B
    char        in_c_pin[MAX_EPOS_DRIVES];       // IN C
    char        in_d_pin[MAX_EPOS_DRIVES];       // IN D
    char        in_e_pin[MAX_EPOS_DRIVES];       // IN E
    char        in_f_pin[MAX_EPOS_DRIVES];       // IN F

    float       in_ana1[MAX_EPOS_DRIVES];        // Analog 1 in
    float       in_ana2[MAX_EPOS_DRIVES];        // Analog 2 in

    char        out_a_pin[MAX_EPOS_DRIVES];      // OUT A
    char        out_b_pin[MAX_EPOS_DRIVES];      // OUT B
    char        out_c_pin[MAX_EPOS_DRIVES];      // OUT C
    char        out_d_pin[MAX_EPOS_DRIVES];      // OUT D

} EPOS_drive_t;
