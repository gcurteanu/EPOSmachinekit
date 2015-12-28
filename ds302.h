



/*
    Debug defines
*/
#ifdef USE_XENO
#define eprintf(...) printf (__VA_ARGS__)
#else
#define eprintf(...) printf (__VA_ARGS__)
#endif

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

void ds302_preOperational_preBoot (CO_Data*);
void ds302_preOperational_postBoot (CO_Data*);
void ds302_slaveBootprocess (CO_Data*);

void _sm_BootSlave_initial(CO_Data*, UNS8);
void _sm_BootSlave_getDeviceType(CO_Data*, UNS8);
void _sm_BootSlave_getIdentification_1(CO_Data*, UNS8);
void _sm_BootSlave_getIdentification_2(CO_Data*, UNS8);
void _sm_BootSlave_getIdentification_3(CO_Data*, UNS8);
void _sm_BootSlave_getIdentification_4(CO_Data*, UNS8);
void _sm_BootSlave_decideBCPath(CO_Data*, UNS8);
// insert B path here
void _sm_BootSlave_doConfigurationVersionChecks(CO_Data*, UNS8);
void _sm_BootSlave_verifyConfigurationVersion_1(CO_Data*, UNS8);
void _sm_BootSlave_verifyConfigurationVersion_2(CO_Data*, UNS8);
void _sm_BootSlave_downloadConfiguration(CO_Data*, UNS8);
void _sm_BootSlave_startErrorControlService(CO_Data*, UNS8);
void _sm_BootSlave_waitHeartbeat(CO_Data*, UNS8);
void _sm_BootSlave_startNodeGuard(CO_Data*, UNS8);
void _sm_BootSlave_errorControlStarted(CO_Data*, UNS8);  // here we need to check if we went D/noconfig and exit
void _sm_BootSlave_startSlave(CO_Data*, UNS8);

typedef enum {
    SM_Initialised = -2,    // SM initialised but not run
    SM_InProgress = -1,     // executed OK but waiting for IO
    SM_OK = 0,              // finished OK
    SM_Error = 1,           // finished with CAN errors
    SM_ErrA = 2,            // workflow codes
    SM_ErrB = 3,            
    SM_ErrC = 4,            
    SM_ErrD = 5,    
    SM_ErrE = 6,            
    SM_ErrF = 7,            
    SM_ErrG = 8,            
    SM_ErrH = 9,
    SM_ErrI = 10,
    SM_ErrJ = 11,
    SM_ErrK = 12,
    SM_ErrL = 13,
    SM_ErrM = 14,
    SM_ErrN = 15,
    SM_ErrO = 16,
} _sm_BootSlave_Codes;


typedef struct {
    // machine codes, current machine state
    _sm_BootSlave_Codes     machine_state;
    // error code from the CAN routines
    UNS32                   error_code;
    // curent state of the state machine. 0 is the first (entry) default state
    _sm_BootSlave_States    current_state;
    // current iteration of the step. 0 is the first iteration
    int                     step_iter;

    // machine data
    char    ViaDPath;

    UNS32   Index1000;

    UNS32   Index1018_1;
    UNS32   Index1018_2;
    UNS32   Index1018_3;
    UNS32   Index1018_4;

    UNS32   Index1020_1;
    UNS32   Index1020_2;

    uint64_t    start_time;
} _sm_BootSlave;

// state machine definitions

extern _sm_BootSlave ds302_slaveBootSM[NMT_MAX_NODE_ID];

#define SM_DATA(nodeID,varname) ds302_slaveBootSM[(nodeID)].varname

#define SM_SWITCH_STATE(new_state,d,nodeID) {\
        ds302_slaveBootSM[(nodeID)].current_state = (new_state); \
        ds302_slaveBootSM[(nodeID)].step_iter = 0; \
        _sm_BootSlave_StateTable[ds302_slaveBootSM[(nodeID)].current_state] ((d),(nodeID)); }

#define SM_RUN_MACHINE(d,nodeID) {\
    _sm_BootSlave_StateTable[ds302_slaveBootSM[(nodeID)].current_state] ((d),(nodeID)); }


#define SM_IS_FINISHED(nodeID) (ds302_slaveBootSM[(nodeID)].machine_state != SM_InProgress)

#define SM_INIT(nodeID) {\
        ds302_slaveBootSM[(nodeID)].current_state = SM_BOOTSLAVE_INITIAL; \
        ds302_slaveBootSM[(nodeID)].machine_state = SM_InProgress; \
        ds302_slaveBootSM[(nodeID)].error_code = 0; \
        ds302_slaveBootSM[(nodeID)].step_iter = 0; \
        ds302_slaveBootSM[(nodeID)].ViaDPath = 0; }

#define SM_INITIAL(nodeID) (ds302_slaveBootSM[(nodeID)].step_iter++ == 0)

#define SM_ERROR(nodeID,code) ds302_slaveBootSM[(nodeID)].machine_state = (code) 

/*
    Extern data structure definitions
*/
// code to text table
extern const char* _sm_BootSlave_CodeToText[];
#define SM_ERR_MSG(code) _sm_BootSlave_CodeToText[(code)]


/*
  Macro DEFINES a state machine structure
  The machine callback table is UNIQUE (not per instance). It needs to be built once
*/

typedef enum {
    MachInit = 0,
    MachRun = 1,
    MachStop = 2,
} _mach_state_t;

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

/*
  Macro RUNS the state machine
*/
#define START_SM(INST_NAME,...) \
    if (INST_NAME.machine_op != MachRun) { INST_NAME.machine_op = MachRun; INST_NAME.machine_callbacks[INST_NAME.machine_state] (__VA_ARGS__); }

#define RUN_SM(INST_NAME,...) \
    if (INST_NAME.machine_op == MachRun) { INST_NAME.machine_callbacks[INST_NAME.machine_state] (__VA_ARGS__); }

#define SWITCH_SM(INST_NAME,NEW_STATE,...) \
    if (INST_NAME.machine_op == MachRun) {\
        INST_NAME.machine_state = NEW_STATE; \
        INST_NAME.step_iter = 0; \
        INST_NAME.machine_callbacks[INST_NAME.machine_state] (__VA_ARGS__); }

#define STOP_SM(INST_NAME)      INST_NAME.machine_op = MachStop

#define RUNNING_SM(INST_NAME)   (INST_NAME.machine_op == MachRun)
#define STOPPED_SM(INST_NAME)   (INST_NAME.machine_op == MachStop)

#define INITIAL_SM(INST_NAME)   (INST_NAME.step_iter++ == 0)

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
        ds302_boot_state_t      bootState;                                  // DS-302 overall boot state
        
        DECLARE_SM (BOOTSLAVE, _bootSlave[NMT_MAX_NODE_ID]);                // boot slave machines
        DECLARE_SM (BOOTMASTER, _masterBoot);                               // master boot machine
} ds302_t;

extern ds302_t     ds302_data;

/* Initialise the DS-302 boot */
void    ds302_init (CO_Data*);
/* starts the DS-302 boot */
void    ds302_start (CO_Data *);
/* Gets the DS-302 boot status */
ds302_boot_state_t   ds302_status (CO_Data *);


/* DCF data defines/routines */
int     ds302_get_next_dcf (UNS8 *data, UNS32 *cursor, UNS16 *idx, UNS8 *subidx, UNS32 *size, UNS32 *value);
/* Loads the DCF data in the local dict for the master nodeid */
int     ds302_load_dcf_local (CO_Data*);
/* set the HB for a node */
int     ds302_setHeartbeat (CO_Data*, UNS8 nodeid, UNS16 heartbeat);
