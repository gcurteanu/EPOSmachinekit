#ifndef __EPOS_DS302_H__
#define __EPOS_DS302_H__

#include "eposconfig.h"

// 0x1F80 bits
#define DS302_DEVICE_NMT_MASTER         0x01    // is the device the NMT master
#define DS302_DEVICE_START_ALL_SLAVES   0x02    // perform a broadcast to start all slaves
#define DS302_DEVICE_MANUAL_OPERATIONAL 0x04    // enter operational state manually
#define DS302_DEVICE_MANUAL_START_SLAVE 0x08    // application will start the slaves
#define DS302_DEVICE_ONERROR_RESETALL   0x10    // if mandatory errors out, reset all nodes (including self)
#define DS302_DEVICE_FLYING_MASTER      0x20    // not implemented / supported
#define DS302_DEVICE_ONERROR_STOPALL    0x40    // if mandatory errors out, stop all nodes (including self)

// 0x1F89 - maximum time to wait for all mandatory slaves

// 0x1F81 network list
#define DS302_NL_IS_SLAVE               0x01
#define DS302_NL_ONBOOT_START_ERROR_CTL 0x02    // this was made obsolete
#define DS302_NL_ONBOOT_START_SLAVE     0x04
#define DS302_NL_MANDATORY              0x08
#define DS302_NL_DONOT_RESET            0x10
#define DS302_NL_SW_VERIFY              0x20
#define DS302_NL_SW_UPDATE              0x40
// the second byte is the retry factor
#define DS302_NL_RETRY_FACTOR(a)        (((a) >> 8) & 0xFF)
// the 3 and 4 bytes are the guard time
#define DS302_NL_GUARD_TIME(a)          (((a) >> 16) & 0xFFFF)

/*
    BootSlave state machine
*/
typedef enum {
    SM_BOOTSLAVE_INITIAL,
    SM_BOOTSLAVE_GET_DEVTYPE,
    SM_BOOTSLAVE_GET_ID1,
    SM_BOOTSLAVE_GET_ID2,
    SM_BOOTSLAVE_GET_ID3,
    SM_BOOTSLAVE_GET_ID4,
    SM_BOOTSLAVE_DECIDE_BC,
    // B path here
    SM_BOOTSLAVE_DO_CONFVER_CHECK,
    SM_BOOTSLAVE_VERIFY_CONFVER_1,
    SM_BOOTSLAVE_VERIFY_CONFVER_2,
    SM_BOOTSLAVE_DOWNLOAD_CONFIG,
    SM_BOOTSLAVE_START_ERRCTL,
    SM_BOOTSLAVE_WAIT_HB,
    SM_BOOTSLAVE_START_NODEGUARD,
    SM_BOOTSLAVE_ERRCTL_STARTED,
    SM_BOOTSLAVE_START_SLAVE,       
    SM_BOOTSLAVE_NUM_STATES,
} _sm_BootSlave_States;

typedef enum {
    SM_Initialised,    // SM initialised but not run
    SM_InProgress,     // executed OK but waiting for IO
    SM_OK,              // finished OK
    SM_Error,           // finished with CAN errors
    SM_ErrA,            // workflow codes
    SM_ErrB,            
    SM_ErrC,            
    SM_ErrD,    
    SM_ErrE,            
    SM_ErrF,            
    SM_ErrG,            
    SM_ErrH,
    SM_ErrI,
    SM_ErrJ,
    SM_ErrK,
    SM_ErrL,
    SM_ErrM,
    SM_ErrN,
    SM_ErrO,
} _sm_BootSlave_Codes;

// code to text table
extern const char* _sm_BootSlave_CodeToText[];
#define SM_ERR_MSG(code) _sm_BootSlave_CodeToText[(code)]

/*
 * State machine definitions
 *
 *
 *
 *
 */

typedef enum {
    MachInit = 0,
    MachRun = 1,
    MachStop = 2,
} _mach_state_t;

/*
  Macros DEFINES a state machine structure
  The machine callback table is UNIQUE (not per instance). It needs to be built once
*/
#define DECLARE_SM_TYPE(SM_NAME,SM_STATE_ENUM,SM_FUNCTION_TYPE,SM_CUSTOMDATA_TYPE)	\
typedef struct {\
    _mach_state_t                           machine_op;\
    SM_STATE_ENUM                           machine_state;\
    int                                     step_iter;\
    SM_CUSTOMDATA_TYPE                      machine_data;\
    SM_FUNCTION_TYPE                        *machine_callbacks;\
} SM_NAME##_SMtype;\
extern SM_FUNCTION_TYPE SM_NAME##_machine_callbacks[];

// Declares and initialises the function table for the machine type
#define INIT_SM_TYPE(SM_NAME,SM_STATE_ENUM,SM_FUNCTION_TYPE,...) SM_FUNCTION_TYPE SM_NAME##_machine_callbacks[] = { __VA_ARGS__ };

// Get the machine data type for instance creation
#define DECLARE_SM(SM_NAME,INST_NAME) SM_NAME##_SMtype	INST_NAME
#define DECLARE_SM_EXT(SM_NAME,INST_NAME) extern SM_NAME##_SMtype  INST_NAME

/*
  Macro INITIALISES a previously declared state machine
*/
#define INIT_SM(SM_NAME,INST_NAME,INITIAL_STATE) \
    INST_NAME.machine_op = MachInit;\
    INST_NAME.machine_state = INITIAL_STATE;\
    INST_NAME.step_iter = 0;\
    INST_NAME.machine_callbacks = SM_NAME##_machine_callbacks;

/* starts a initialised state machine */
#define START_SM(INST_NAME,...) \
    if (INST_NAME.machine_op != MachRun) { INST_NAME.machine_op = MachRun; INST_NAME.machine_callbacks[INST_NAME.machine_state] (__VA_ARGS__); }

/* runs a previously started state machine */
#define RUN_SM(INST_NAME,...) \
    if (INST_NAME.machine_op == MachRun) { INST_NAME.machine_callbacks[INST_NAME.machine_state] (__VA_ARGS__); }

/* switches the state machine to a new state */
#define SWITCH_SM(INST_NAME,NEW_STATE,...) \
    if (INST_NAME.machine_op == MachRun) {\
        INST_NAME.machine_state = NEW_STATE; \
        INST_NAME.step_iter = 0; \
        INST_NAME.machine_callbacks[INST_NAME.machine_state] (__VA_ARGS__); }

/* stops the state machine */
#define STOP_SM(INST_NAME)      INST_NAME.machine_op = MachStop

/* get the state machine's current state */
#define RUNNING_SM(INST_NAME)   (INST_NAME.machine_op == MachRun)
#define STOPPED_SM(INST_NAME)   (INST_NAME.machine_op == MachStop)

/* To be used for run-once-at-start code */
#define INITIAL_SM(INST_NAME)   (INST_NAME.step_iter++ == 0)

/* Macro to access the state machine's data */
#define DATA_SM(INST_NAME)      INST_NAME.machine_data

/*
  Boot up machine
*/
typedef enum {
    MB_INITIAL,
    MB_BOOTPROC,
    MB_OPERWAIT,
    MB_SLAVESTART,
} _sm_BootMaster_States;

/*
    Declare the BOOTMASTER state machine type
*/
DECLARE_SM_TYPE(BOOTMASTER, _sm_BootMaster_States, TimerCallback_t, int);
/*
    Declare the _masterBoot SM instance
*/
DECLARE_SM_EXT(BOOTMASTER, _masterBoot);

/*
  Helper functions
*/
/*
ds302 bitcheck takes the OD index/subindex and a bitmask to apply.
Works on 32 bit values only.
Uses direct OD access, doesn't do any endian stuff, takes the value as present in memory (BEWARE)
returns -1 for error, and 0/1 based on (OD & bitmask) == bitmask
*/
int ds302_bitcheck_32 (CO_Data* d, UNS16 idx, UNS8 subidx, UNS32 bitmask);

/*
ds302 NL node in list verifies if the specified slave is in the network list
Eq. verifies if it has the slave bit set for it's index under 0x1F81
*/
int ds302_nl_node_in_list(CO_Data* d, UNS8 nodeid);


typedef enum {
    BootUnused = 0,         // unused state machine, no slave defined
    BootInitialised = 1,    // boot initialised, we have a slave at this address
    BootAttempted = 2,      // boot was attempted for a optional slave
    BootRunning = 3,        // boot process is running
    BootCompleted = 4,      // boot completed ok for the slave
    BootTimedOut = 5,       // boot timed out for the slave
    BootError = 6,          // boot completed with error
} ds302_boot_state_t;

typedef struct {
    _sm_BootSlave_Codes     result;         // Result of the state machine
    ds302_boot_state_t      state;          // Result of the overall boot process
    UNS32                   errorCode;      // CAN error code
    
    char                    ViaDPath;       // In case we kept the slave UP during boot, don't reconfigure/start per DS-302

    UNS32                   Index1000;      // Device Type

    UNS32                   Index1018_1;    // Vendor ID. 0x0000 means don't care
    UNS32                   Index1018_2;    // Product code. 0x0000 means don't care
    UNS32                   Index1018_3;    // Revision number. 0x0000 means don't care
    UNS32                   Index1018_4;    // Product Code. 0x0000 means don't care

    UNS32                   Index1020_1;    // Configuration date. 0x0000 means don't care
    UNS32                   Index1020_2;    // Configuration time. 0x0000 means don't care

    uint64_t                bootStart;      // timestamp of the boot process start
    
    uint64_t                ecsStart;       // timestamp of the Error Control Service start for HB checks
    
    // DCF
    UNS32                   dcfCursor;      // DCF cursor for the SDO loads
    UNS8                    *dcfData;       // DCF data for the node
    UNS32                   dcfSize;        // DCF total data size for this slave
    UNS32                   dcfCount;       // DCF count of entries in the domain
    UNS32                   dcfState;       // DCF state. 0 - writeInit, 1 - writeInProgress. Used to determine current state in the callback
    UNS32                   dcfLoadCount;   // DCF items loaded so far
} _bootSlave_data_t;

DECLARE_SM_TYPE(BOOTSLAVE, _sm_BootSlave_States, SDOCallback_t, _bootSlave_data_t);

/*
    DS 302 global structure holding all the DS 302 information
*/

typedef struct {
    UNS16   errCode;
    UNS8    errReg;
    UNS8    errData[5];
} emcy_frame_t;

typedef struct {
    emcy_frame_t    errors[EPOS_MAX_ERRORS];
    int             errCount;
} device_errors_t;

typedef struct {
        ds302_boot_state_t      bootState;                                  // DS-302 overall boot state
        
        DECLARE_SM (BOOTSLAVE, _bootSlave[NMT_MAX_NODE_ID]);                // boot slave machines
        DECLARE_SM (BOOTMASTER, _masterBoot);                               // master boot machine
        
        SDOCallback_t           bootFinished;                               // boot finished callback
        
        device_errors_t         deviceErrors[NMT_MAX_NODE_ID];              // the error stack
} ds302_t;

extern ds302_t     ds302_data;

/* the clock function */
uint64_t rtuClock();

/* Initialise the DS-302 boot */
void    ds302_init (CO_Data*);
/* starts the DS-302 boot sequence */
void    ds302_start (CO_Data *);
/* init the boot for a slave */
void    ds302_init_slaveSM (CO_Data*, UNS8);
/* boot a slave */
void    ds302_boot_slave (CO_Data*, UNS8);
/* Gets the overall DS-302 boot status */
ds302_boot_state_t  ds302_status (CO_Data *);
/* Gets a slave's boot status */
ds302_boot_state_t  ds302_node_status (CO_Data*, UNS8);
/* Gets a slave's boot result */
_sm_BootSlave_Codes ds302_node_result (CO_Data*, UNS8);
/* Gets a slave's CAN error */
UNS32   ds302_node_error (CO_Data*, UNS8);

/* EMCY error handling routines */
/* 
    add error nodeid, error code, error register, error data[5]
    returns 1 on success, 0 on failure
*/
int     ds302_add_error (UNS8, UNS16, UNS8, const UNS8*);
/*
    clears the errors for a node
    clear errors nodeid
*/
void    ds302_clear_errors (UNS8);
/*
    checks the node health (NMT,Boot,errors)
    node healthy nodeid
*/
int     ds302_node_healthy (CO_Data*, UNS8);

/* DCF data defines/routines */
int     ds302_get_next_dcf (UNS8 *data, UNS32 *cursor, UNS16 *idx, UNS8 *subidx, UNS32 *size, UNS32 *value);
/* Loads the DCF data in the local dict for the master nodeid */
int     ds302_load_dcf_local (CO_Data*);
/* set the HB for a node */
int     ds302_setHeartbeat (CO_Data*, UNS8 nodeid, UNS16 heartbeat);

#endif
