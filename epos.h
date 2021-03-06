/*
    EPOS error table
*/
#ifndef __EPOS_H__
#define __EPOS_H__

#include <data.h>
#include "dcf.h"
#include "eposconfig.h"

typedef struct {
    UNS16           error_code;
    const char *    message;
    const char *    error_description;
} epos_error_t;

extern epos_error_t epos_error_table[];

const char * epos_error_text (UNS16 errCode);

/* the possible CiA 402 states (updated with vendor specific items for Maxon EPOS) */
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

/*
Profile Position Mode state machine states
New set point: ___|````````|____________ (ControlWord)
Set point ACK: ________|``````````|_____ (StatusWord)
State        : RDY|SENT|ACK|RUN...|RDY..
*/
typedef enum {
    PPM_Ready = 0x00,
    PPM_Sent = 0x10,
    PPM_Acknowledged = 0x11,
    PPM_Running = 0x01,
} PPM_State_t;

/* Maxon EPOS drive modes */
typedef enum {
    EPOS_MODE_HMM = 6,  // homing
    EPOS_MODE_PVM = 3,  // profile velocity
    EPOS_MODE_PPM = 1,  // profile position
    EPOS_MODE_POS = -1, // position mode
    EPOS_MODE_VEL = -2, // velocity mode
    EPOS_MODE_AMP = -3, // current mode
    EPOS_MODE_DIA = -4, // diagnostic mode
    EPOS_MODE_MEM = -5, // master encoder mode
    EPOS_MODE_SDM = -6, // step/direction mode
} EPOS_DriveMode_t;

typedef struct {
    UNS16   idx;
    UNS8    sub;
    UNS32   size;
    void    *data;
} SDO_item_t;

#define MAX_SDO_ITEMS   16

typedef enum {
    SDO_READ,
    SDO_WRITE,
    SDO_INIT,
    SDO_FINISH,
} SDO_transfer_type_t;

typedef struct {
    SDO_item_t  items[MAX_SDO_ITEMS];
    int         count;
    int         cursor;
    UNS8        state;
    UNS32       error;
    SDO_transfer_type_t
                type;
} SDO_transfer_t;

typedef struct {
    // the CanFestival object
    CO_Data*    d;
    
    // the list of the EPOS slaves we control
    UNS8        epos_slaves[EPOS_MAX_DRIVES];
    UNS8        epos_slave_count;
    
    // holds the DCF data for initializing the nodes
    dcfset_t    dcf_data;
    
    // holds the drive errors signalled via EMCY
    UNS32       slave_err[EPOS_MAX_DRIVES][EPOS_MAX_ERRORS+1];
    
    // EPOS_State is the slave state. Used internally by the drive routines
    UNS16       EPOS_State[EPOS_MAX_DRIVES];
    
    PPM_State_t EPOS_PPMState[EPOS_MAX_DRIVES];
    
    // enabled or disabled
    char        drive_enabled;
    // initialised or not
    char        drive_initialised;
    // state of the drive
    char        drive_state;

        
    // CNC interface
    char        enable[EPOS_MAX_DRIVES];         // true - drive enabled, false - drive disabled
    int         control_type[EPOS_MAX_DRIVES];   // 0 - position control, 1 - velocity control
    int         maxvel[EPOS_MAX_DRIVES];	     // maximal drive velocity
    int         maxaccel[EPOS_MAX_DRIVES];       // maximal drive acceleration

    int         counts[EPOS_MAX_DRIVES];         // raw encoder / position from the drive
    float       position_scale[EPOS_MAX_DRIVES]; // scale for positioning. position = counts / position-scale

    float       position_cmd[EPOS_MAX_DRIVES];   // position command (for position control)
    float       velocity_cmd[EPOS_MAX_DRIVES];   // velocity command (for velocity control)

    float       position_fb[EPOS_MAX_DRIVES];    // position feedback
    float       velocity_fb[EPOS_MAX_DRIVES];    // velocity feedback

    char        fault[EPOS_MAX_DRIVES];          // drive is faulted

    // extra stuff

    char        home[EPOS_MAX_DRIVES];           // set to high to initialise homing
    char        homing_done[EPOS_MAX_DRIVES];    // set to high when drive is homed

    char        lock[EPOS_MAX_DRIVES];           // engage the lock brake
    char        is_locked[EPOS_MAX_DRIVES];      // lock brake is engaged and axis is confirmed locked

    // GPIO pins
    char        home_sw_pin[EPOS_MAX_DRIVES];    // home switch pin
    char        enable_pin[EPOS_MAX_DRIVES];     // enable pin
    char        in_a_pin[EPOS_MAX_DRIVES];       // IN A
    char        in_b_pin[EPOS_MAX_DRIVES];       // IN B
    char        in_c_pin[EPOS_MAX_DRIVES];       // IN C
    char        in_d_pin[EPOS_MAX_DRIVES];       // IN D
    char        in_e_pin[EPOS_MAX_DRIVES];       // IN E
    char        in_f_pin[EPOS_MAX_DRIVES];       // IN F

    float       in_ana1[EPOS_MAX_DRIVES];        // Analog 1 in
    float       in_ana2[EPOS_MAX_DRIVES];        // Analog 2 in

    char        out_a_pin[EPOS_MAX_DRIVES];      // OUT A
    char        out_b_pin[EPOS_MAX_DRIVES];      // OUT B
    char        out_c_pin[EPOS_MAX_DRIVES];      // OUT C
    char        out_d_pin[EPOS_MAX_DRIVES];      // OUT D

    
    // SDO transfer data, per node
    SDO_transfer_t  sdos[EPOS_MAX_DRIVES];
} EPOS_drive_t;

extern EPOS_drive_t EPOS_drive;

int     epos_initialize_master (CO_Data * d, const char * dcf_file);
int     epos_setup_sdo (UNS8 slaveid, int idx);
int     epos_setup_rx_pdo (UNS8 slaveid, int idx);
int     epos_setup_tx_pdo (UNS8 slaveid, int idx);
int     epos_add_slave (UNS8 slaveid);

// EPOS PPM routines
void    update_PPM (int idx);
int     epos_can_do_PPM (int idx);
int     epos_do_move_PPM (int idx, INTEGER32 position);

// EPOS generic routines
void    epos_set_absolute (int idx);
void    epos_set_relative (int idx);
void    epos_set_continuous (int idx);
void    epos_set_segmented (int idx);
void    epos_halt (int idx);
void    epos_execute (int idx);
int     epos_in_position (int idx);

// EPOS drive control
void    epos_enable_drive (int idx);
void    epos_disable_drive (int idx);
void    epos_fault_reset (int idx);
int     epos_drive_operational (int idx);
int     epos_drive_faulted (int idx);
int     epos_drive_disabled (int idx);

void    epos_set_mode (int idx, EPOS_DriveMode_t);
EPOS_DriveMode_t    epos_get_mode (int idx);




#endif