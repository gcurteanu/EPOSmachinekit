/*
    EPOS error table
*/
#ifndef __EPOS_H__
#define __EPOS_H__

#include <data.h>

#include "dcf.h"

#define SET_BIT(val, bitIndex) val |= (1 << bitIndex)
#define CLEAR_BIT(val, bitIndex) val &= ~(1 << bitIndex)
#define TOGGLE_BIT(val, bitIndex) val ^= (1 << bitIndex)
#define BIT_IS_SET(val, bitIndex) (val & (1 << bitIndex))

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

typedef enum {
    PPM_Ready = 0x00,
    PPM_Sent = 0x10,
    PPM_Acknowledged = 0x11,
    PPM_Running = 0x01,
} PPM_State_t;

typedef struct {
    // the CanFestival object
    CO_Data*    d;
    
    // the list of the EPOS slaves we control
    UNS8        epos_slaves[MAX_EPOS_DRIVES];
    UNS8        epos_slave_count;
    
    // holds the DCF data for initializing the nodes
    dcfset_t    dcf_data;
    
    // holds the drive errors signalled via EMCY
    UNS32       slave_err[MAX_EPOS_DRIVES][EPOS_MAX_ERRORS+1];
    
    // EPOS_State is the slave state. Used internally by the drive routines
    UNS16       EPOS_State[MAX_EPOS_DRIVES];
    
    PPM_State_t EPOS_PPMState[MAX_EPOS_DRIVES];
    
    // enabled or disabled
    char        drive_enabled;
    // initialised or not
    char        drive_initialised;
    // state of the drive
    char        drive_state;

        
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

extern EPOS_drive_t EPOS_drive;

int     epos_initialize_master (CO_Data * d, const char * dcf_file);
int     epos_setup_sdo (UNS8 slaveid, int idx);
int     epos_setup_rx_pdo (UNS8 slaveid, int idx);
int     epos_setup_tx_pdo (UNS8 slaveid, int idx);
int     epos_add_slave (UNS8 slaveid);

// EPOS PPM routines
int     epos_can_do_PPM (int idx);
int     epos_do_move (int idx);

#endif