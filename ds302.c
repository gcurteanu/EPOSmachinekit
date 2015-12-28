#include "canfestival.h"
#include "EPOScontrol.h"
#include "data.h"
#include <time.h>
#include "ds302.h"

//#define DS302_DEBUG(msg)
#define DS302_DEBUG(...) eprintf(__VA_ARGS__)

// max time in us for a boot
#define NODE_BOOT_TIME 10*1000*1000

// gets the clock in microsecs
uint64_t rtuClock()
{
    struct timespec	tp;
    uint64_t	result;

    if(clock_gettime(CLOCK_MONOTONIC, &tp) < 0) {
        DS302_DEBUG("CLOCK_GETTIME ERROR!!!\n");
    }

    result = (tp.tv_nsec / 1000) + (tp.tv_sec * 1000000);
    return result;
}

void _sm_BootMaster_initial (CO_Data*, UNS32);
void _sm_BootMaster_bootproc (CO_Data*, UNS32);
void _sm_BootMaster_operwait (CO_Data*, UNS32);
void _sm_BootMaster_slavestart (CO_Data*, UNS32);

INIT_SM_TYPE(BOOTMASTER,_sm_BootMaster_States,TimerCallback_t,
    _sm_BootMaster_initial,
    _sm_BootMaster_bootproc,
    _sm_BootMaster_operwait,
    _sm_BootMaster_slavestart);

//DECLARE_SM(BOOTMASTER, _masterBoot);

INIT_SM_TYPE(BOOTSLAVE,_sm_BootSlave_States,SDOCallback_t,
    _sm_BootSlave_initial,
    _sm_BootSlave_getDeviceType,
    _sm_BootSlave_getIdentification_1,
    _sm_BootSlave_getIdentification_2,
    _sm_BootSlave_getIdentification_3,
    _sm_BootSlave_getIdentification_4,
    _sm_BootSlave_decideBCPath,
    _sm_BootSlave_doConfigurationVersionChecks,
    _sm_BootSlave_verifyConfigurationVersion_1,
    _sm_BootSlave_verifyConfigurationVersion_2,
    _sm_BootSlave_downloadConfiguration,
    _sm_BootSlave_startErrorControlService,
    _sm_BootSlave_waitHeartbeat,
    _sm_BootSlave_startNodeGuard,
    _sm_BootSlave_errorControlStarted,
    _sm_BootSlave_startSlave);

// declare the master ds302_data structure
ds302_t     ds302_data;

// 0x1F80 bits
#define DS302_DEVICE_NMT_MASTER         0x01    // is the device the NMT master
#define DS302_DEVICE_START_ALL_SLAVES   0x02    // perform a broadcast to start all slaves
#define DS302_DEVICE_MANUAL_OPERATIONAL 0x04    // enter operational state manually
#define DS302_DEVICE_MANUAL_START_SLAVE 0x08    // application will start the slaves
#define DS302_DEVICE_ONERROR_MANDATORY  0x10    // reset ALL slaves if a mandatory has an error

// 0x1F89 - maximum time to wait for all mandatory slaves

// 0x1F81 network list
#define DS302_NL_IS_SLAVE               0x01
#define DS302_NL_ONBOOT_START_ERROR_CTL 0x02
#define DS302_NL_ONBOOT_START_SLAVE     0x04
#define DS302_NL_MANDATORY              0x08
#define DS302_NL_DONOT_RESET            0x10
#define DS302_NL_SW_VERIFY              0x20
#define DS302_NL_SW_UPDATE              0x40
// the second byte is the retry factor
#define DS302_NL_RETRY_FACTOR(a)        (((a) >> 8) & 0xFF)
// the 3 and 4 bytes are the guard time
#define DS302_NL_GUARD_TIME(a)          (((a) >> 16) & 0xFFFF)

// holds the boot signals
// ds302_boot_state_t ds302_boot_data[NMT_MAX_NODE_ID];

// move to a pointer based state machine, using functions that are also callback capable for SDOs
// type for functions is SDOCallback_t

/*
 BootSlave state machine
*/

/* Commented, moving to the ds302_data structure
SDOCallback_t const _sm_BootSlave_StateTable[ SM_BOOTSLAVE_NUM_STATES ] = {
    _sm_BootSlave_initial,
    _sm_BootSlave_getDeviceType,
    _sm_BootSlave_getIdentification_1,
    _sm_BootSlave_getIdentification_2,
    _sm_BootSlave_getIdentification_3,
    _sm_BootSlave_getIdentification_4,
    _sm_BootSlave_decideBCPath,
    _sm_BootSlave_doConfigurationVersionChecks,
    _sm_BootSlave_verifyConfigurationVersion_1,
    _sm_BootSlave_verifyConfigurationVersion_2,
    _sm_BootSlave_downloadConfiguration,
    _sm_BootSlave_startErrorControlService,
    _sm_BootSlave_waitHeartbeat,
    _sm_BootSlave_startNodeGuard,
    _sm_BootSlave_errorControlStarted,
    _sm_BootSlave_startSlave,
};

// holds the state machines for each slave boot process
_sm_BootSlave ds302_slaveBootSM[NMT_MAX_NODE_ID];
*/

const char* _sm_BootSlave_CodeToText[] = {
    "OK: finished OK",
    "CAN: generic CAN error",
    "A: node no longer in network list",         
    "B: no response on 0x1000 received",
    "C: device type (0x1000) did not match expected",
    "D: vendor id (0x1018) did not match expected",
    "E: slave did not respond to status check. Slave is HB producer",
    "F: slave did not respond to status check. Slave is a NG slave",
    "G: application software version check failed, no values defined",
    "H: application software version check failed, 0x1F52 mismatch",
    "I: application software version check failed, 0x1027 mismatch",
    "J: Automatic configuration download failed",
    "K: heartbeat failure during Error Control Service start",            
    "L: slave was initially operational",
    "M: product code (0x1018) did not match expected",
    "N: revision number (0x1018) did not match expected",
    "O: serial number (0x1018) did not match expected",
};

static int sleep_ms ( unsigned long ms )
{
    if ( ms >= 1000 ) ms = 1000;
    return usleep ( ms * 1000 );
}

void _sm_BootSlave_initial(CO_Data* d, UNS8 nodeid)
{
    // dummy checking for 0x1F81 bit 0

    DS302_DEBUG("_sm_BootSlave_initial (%d)\n", nodeid);

    if (!ds302_nl_node_in_list(d, nodeid)) {
        DS302_DEBUG("_sm_BootSlave_initial Error A (%d)\n", nodeid);
        //SM_ERROR(nodeid, SM_ErrA);
        DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrA;
        //stop the machine
        STOP_SM(ds302_data._bootSlave[nodeid]);
        return;
    } else {
        DS302_DEBUG("_sm_BootSlave_initial switch to SM_BOOTSLAVE_GET_DEVTYPE (%d)\n", nodeid);
        // go to the next state
        //SM_SWITCH_STATE(SM_BOOTSLAVE_GET_DEVTYPE,d,nodeid)
        SWITCH_SM(ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_GET_DEVTYPE, d, nodeid);
        return;
    }
}

void _sm_BootSlave_getDeviceType(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_getDeviceType (%d)\n", nodeid);

    //if (SM_INITIAL(nodeid)) {
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {
        DS302_DEBUG("_sm_BootSlave_getDeviceType initial run (%d)\n", nodeid);

        // code for the first run only
        // read 0x1000 0x00
        // self callback
        readNetworkDictCallbackAI (d, nodeid, 0x1000, 0x00, 0, _sm_BootSlave_getDeviceType, 0);
        // do nothing else on the first run
        return;
    }
    DS302_DEBUG("_sm_BootSlave_getDeviceType second+ call (%d)\n", nodeid);

    // we end here on callback

    UNS32	size = sizeof(DATA_SM(ds302_data._bootSlave[nodeid]).Index1000);
    UNS8	retcode = getReadResultNetworkDict (d, nodeid, &DATA_SM(ds302_data._bootSlave[nodeid]).Index1000, &size,
            &DATA_SM(ds302_data._bootSlave[nodeid]).errorCode);

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS) {
        DS302_DEBUG("_sm_BootSlave_getDeviceType SDO op in progress (%d)\n", nodeid);
        // do nothing, outside of callback call
        return;
    }

    /* Finalise last SDO transfer with this node */
    closeSDOtransfer(d, nodeid, SDO_CLIENT);

    if(retcode != SDO_FINISHED) {
        DS302_DEBUG("_sm_BootSlave_getDeviceType SDO error (%d) = %x\n", nodeid, retcode);
        //SM_ERROR(nodeid, SM_ErrB);
        DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrB;
        STOP_SM(ds302_data._bootSlave[nodeid]);
        return;
    }
    
    // we end here only if transfer was ok
    DS302_DEBUG("_sm_BootSlave_getDeviceType no issues, logic proceeding (%d)\n", nodeid);

    // we have the data in Index1000
    // compare against the expected value in 0x1F84
    // dummy check for now
    if (0) {
            //SM_ERROR(nodeid, SM_ErrC);
            DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrC;
            STOP_SM(ds302_data._bootSlave[nodeid]);
            return;
    } else {
        // everything OK, see if we need to check IDs or not
        // dummy check for now
        if (1) {
            // switch to pulling the IDs
            //SM_SWITCH_STATE(SM_BOOTSLAVE_GET_ID1,d,nodeid)
            SWITCH_SM(ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_GET_ID1, d, nodeid);
            return;
        } else {
            // skip pulling the IDs chain
            //SM_SWITCH_STATE(SM_BOOTSLAVE_DECIDE_BC,d,nodeid)
            SWITCH_SM(ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_DECIDE_BC, d, nodeid);
            return;
        }
    }
}

void _sm_BootSlave_getIdentification_1(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_getIdentification_1\n");

    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {
        DS302_DEBUG("_sm_BootSlave_getIdentification_1 initial call\n");

        // code for the first run only  
        // read 0x1018 0x01                       
        // self callback      
        readNetworkDictCallbackAI (d, nodeid, 0x1018, 0x01, 0, _sm_BootSlave_getIdentification_1, 0);
        // do nothing else on the first run
        return;                   
    }

    DS302_DEBUG("_sm_BootSlave_getIdentification_1 second call+\n");

    // we end here on callback

    UNS32   size = sizeof(DATA_SM(ds302_data._bootSlave[nodeid]).Index1018_1);
    UNS8    retcode = getReadResultNetworkDict (d, nodeid, &DATA_SM(ds302_data._bootSlave[nodeid]).Index1018_1, &size,
            &DATA_SM(ds302_data._bootSlave[nodeid]).errorCode);

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS)
            // do nothing, outside of callback call
            return;

    /* Finalise last SDO transfer with this node */
    closeSDOtransfer(d, nodeid, SDO_CLIENT);

    if(retcode != SDO_FINISHED) {
            //SM_ERROR(nodeid, SM_ErrD);
            DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrD;
            STOP_SM(ds302_data._bootSlave[nodeid]);
            return;
    }    
    
    // verify against required data in 0x1F85
    if (0) {
        // mismatch, stop process
        //SM_ERROR(nodeid, SM_ErrD);
        DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrD;
        STOP_SM(ds302_data._bootSlave[nodeid]);
        return;
    } else {
        // go to the next one
        //SM_SWITCH_STATE(SM_BOOTSLAVE_GET_ID2,d,nodeid)
        SWITCH_SM(ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_GET_ID2, d, nodeid);
        return;
    }
}

void _sm_BootSlave_getIdentification_2(CO_Data* d, UNS8 nodeid)
{
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {      
        // code for the first run only  
        // read 0x1018 0x02                       
        // self callback      
        readNetworkDictCallbackAI (d, nodeid, 0x1018, 0x02, 0, _sm_BootSlave_getIdentification_2, 0);
        // do nothing else on the first run
        return;                   
    }

    // we end here on callback

    UNS32   size = sizeof(DATA_SM(ds302_data._bootSlave[nodeid]).Index1018_2);
    UNS8    retcode = getReadResultNetworkDict (d, nodeid, &DATA_SM(ds302_data._bootSlave[nodeid]).Index1018_2, &size, 
            &DATA_SM(ds302_data._bootSlave[nodeid]).errorCode);

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS)
        // do nothing, outside of callback call
        return;

    /* Finalise last SDO transfer with this node */
    closeSDOtransfer(d, nodeid, SDO_CLIENT);  
        
    if(retcode != SDO_FINISHED) {
        //SM_ERROR(nodeid, SM_ErrM);
        DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrM;
        STOP_SM(ds302_data._bootSlave[nodeid]);
        return;
    }

    // verify against required data in 0x1F86
    if (0) {                  
        // mismatch, stop process
        // SM_ERROR(nodeid, SM_ErrM);
        DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrM;
        STOP_SM(ds302_data._bootSlave[nodeid]);
        return;
    } else {                               
        // go to the next one               
        //SM_SWITCH_STATE(SM_BOOTSLAVE_GET_ID3,d,nodeid)
        SWITCH_SM(ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_GET_ID3, d, nodeid);
        return;
    }
}

void _sm_BootSlave_getIdentification_3(CO_Data* d, UNS8 nodeid)
{
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {      
        // code for the first run only  
        // read 0x1018 0x03                       
        // self callback      
        readNetworkDictCallbackAI (d, nodeid, 0x1018, 0x03, 0, _sm_BootSlave_getIdentification_3, 0);
        // do nothing else on the first run
        return;                   
    }

    // we end here on callback

    UNS32   size = sizeof(DATA_SM(ds302_data._bootSlave[nodeid]).Index1018_3);
    UNS8    retcode = getReadResultNetworkDict (d, nodeid, &DATA_SM(ds302_data._bootSlave[nodeid]).Index1018_3, &size, 
            &DATA_SM(ds302_data._bootSlave[nodeid]).errorCode);

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS)
        // do nothing, outside of callback call
        return;

    /* Finalise last SDO transfer with this node */
    closeSDOtransfer(d, nodeid, SDO_CLIENT);  

    if(retcode != SDO_FINISHED) {
        // SM_ERROR(nodeid, SM_ErrN);
        DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrN;
        STOP_SM(ds302_data._bootSlave[nodeid]);
        return;
    }

    // verify against required data in 0x1F87
    if (0) {                  
        // mismatch, stop process
        //SM_ERROR(nodeid, SM_ErrN);
        DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrN;
        STOP_SM(ds302_data._bootSlave[nodeid]);
        return;
    } else {                               
        // go to the next one               
        //SM_SWITCH_STATE(SM_BOOTSLAVE_GET_ID4,d,nodeid)
        SWITCH_SM(ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_GET_ID4, d, nodeid);
        return;
    }
}

void _sm_BootSlave_getIdentification_4(CO_Data* d, UNS8 nodeid)
{
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {      
        // code for the first run only  
        // read 0x1018 0x04                       
        // self callback      
        readNetworkDictCallbackAI (d, nodeid, 0x1018, 0x04, 0, _sm_BootSlave_getIdentification_4, 0);
        // do nothing else on the first run
        return;                   
    }

    // we end here on callback

    UNS32   size = sizeof(DATA_SM(ds302_data._bootSlave[nodeid]).Index1018_4);
    UNS8    retcode = getReadResultNetworkDict (d, nodeid, &DATA_SM(ds302_data._bootSlave[nodeid]).Index1018_4, &size, 
            &DATA_SM(ds302_data._bootSlave[nodeid]).errorCode);

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS)
        // do nothing, outside of callback call
        return;

    /* Finalise last SDO transfer with this node */
    closeSDOtransfer(d, nodeid, SDO_CLIENT);  

    if(retcode != SDO_FINISHED) {
        // SM_ERROR(nodeid, SM_ErrO);
        DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrO;
        STOP_SM(ds302_data._bootSlave[nodeid]);
        return;
    }

    // verify against required data in 0x1F85
    if (0) {                  
        // mismatch, stop process
        //SM_ERROR(nodeid, SM_ErrO);
        DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrO;
        STOP_SM(ds302_data._bootSlave[nodeid]);
        return;
    } else {                               
        // go to the next one               
        //SM_SWITCH_STATE(SM_BOOTSLAVE_DECIDE_BC,d,nodeid)
        SWITCH_SM(ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_DECIDE_BC, d, nodeid);
        return;
    }
}

void _sm_BootSlave_decideBCPath(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_decideBCPath\n");
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {      
        // code for the first run only
        // switch to path C, path B is not implemented yet
        // SM_SWITCH_STATE(SM_BOOTSLAVE_DO_CONFVER_CHECK,d,nodeid)
        SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_DO_CONFVER_CHECK, d, nodeid);
        return;
    }	
}

void _sm_BootSlave_doConfigurationVersionChecks(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_doConfigurationVersionChecks\n");
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {      
        // code for the first run only  
       
        // see if we have 0x1F26 and 0x1F27 defined in the config for the node
        // if so, get the device info, if not download config

        if (0) {
            // we don't have the values, go straight to download
            //SM_SWITCH_STATE(SM_BOOTSLAVE_DOWNLOAD_CONFIG,d,nodeid)
            SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_DOWNLOAD_CONFIG, d, nodeid);
            return;
        } else {
            // we have the values, get them and compare them
            //SM_SWITCH_STATE(SM_BOOTSLAVE_VERIFY_CONFVER_1,d,nodeid)
            SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_VERIFY_CONFVER_1, d, nodeid);
            return;
        }
    }  
}

void _sm_BootSlave_verifyConfigurationVersion_1(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_verifyConfigurationVersion_1\n");
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {      
        // code for the first run only  
        // read 0x1020 0x01           
        // self callback      
        readNetworkDictCallbackAI (d, nodeid, 0x1020, 0x01, 0, _sm_BootSlave_verifyConfigurationVersion_1, 0);
        // do nothing else on the first run
        return;                
    }  

    // we end here on callback

    UNS32   size = sizeof(DATA_SM(ds302_data._bootSlave[nodeid]).Index1020_1);
    UNS8    retcode = getReadResultNetworkDict (d, nodeid, &DATA_SM(ds302_data._bootSlave[nodeid]).Index1020_1, &size, 
            &DATA_SM(ds302_data._bootSlave[nodeid]).errorCode);

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS)
        // do nothing, outside of callback call
        return;

    if(retcode != SDO_FINISHED) {
        // here we have an error, but it is NOT fatal!!!
        // SM_ERROR(nodeid, SM_ErrO);
        // set the data to -1 to indicate failure to retrieve
        //SM_DATA(nodeid,Index1020_1) = -1;
        DATA_SM(ds302_data._bootSlave[nodeid]).Index1020_1 = -1;
    }
    /* Finalise last SDO transfer with this node */
    closeSDOtransfer(d, nodeid, SDO_CLIENT);  

    // verify against required data in 0x1F26
    if (0) {                  
        // mismatch, go directly to download
        //SM_SWITCH_STATE(SM_BOOTSLAVE_DOWNLOAD_CONFIG,d,nodeid)
        SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_DOWNLOAD_CONFIG, d, nodeid);
        return;
    } else {                               
        // go to the next one               
        //SM_SWITCH_STATE(SM_BOOTSLAVE_VERIFY_CONFVER_2,d,nodeid)
        SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_VERIFY_CONFVER_2, d, nodeid);
        return;
    }
}

void _sm_BootSlave_verifyConfigurationVersion_2(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_verifyConfigurationVersion_2\n");
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {      
        // code for the first run only         
        // read 0x1020 0x02 
        // self callback      
        readNetworkDictCallbackAI (d, nodeid, 0x1020, 0x02, 0, _sm_BootSlave_verifyConfigurationVersion_2, 0);
        // do nothing else on the first run
        return;                                  
    }  

    // we end here on callback

    UNS32   size = sizeof(DATA_SM(ds302_data._bootSlave[nodeid]).Index1020_2); 
    UNS8    retcode = getReadResultNetworkDict (d, nodeid, &DATA_SM(ds302_data._bootSlave[nodeid]).Index1020_2, &size, 
            &DATA_SM(ds302_data._bootSlave[nodeid]).errorCode);

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS)
        // do nothing, outside of callback call
        return;

    if(retcode != SDO_FINISHED) {
        // here we have an error, but it is NOT fatal!!!
        // SM_ERROR(nodeid, SM_ErrO);     
        // set the data to -1 to indicate failure to retrieve
        //SM_DATA(nodeid,Index1020_2) = -1;
        DATA_SM(ds302_data._bootSlave[nodeid]).Index1020_2 = -1;
    }
    /* Finalise last SDO transfer with this node */
    closeSDOtransfer(d, nodeid, SDO_CLIENT);  

    // verify against required data in 0x1F27
    if (1) {                  
        // mismatch, go directly to download
        //SM_SWITCH_STATE(SM_BOOTSLAVE_DOWNLOAD_CONFIG,d,nodeid)
        SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_DOWNLOAD_CONFIG, d, nodeid);
        return;
    } else {                               
        // configuration is at the exepected levels, skip configuration download
        // go to start error control services   
        //SM_SWITCH_STATE(SM_BOOTSLAVE_START_ERRCTL,d,nodeid)
        SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_START_ERRCTL, d, nodeid);
        return;
    }
}

void _sm_BootSlave_downloadConfiguration(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_downloadConfiguration\n");
    //// this is dummy for now. Doesn't do anything other than keep the SM functioning correctly
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {
        // code for the first run only
        DS302_DEBUG("ConciseDCF initialization for slave %d\n", nodeid);
        // initialises the DCF pointers and data
        
        const indextable *	Object1F22;
        UNS32		        errorCode;

        Object1F22 = (*d->scanIndexOD)(d, 0x1F22, &errorCode);
        if (errorCode != OD_SUCCESSFUL) {
            DS302_DEBUG("ConciseDCF for %d: can not get data for 0x1F22\n", nodeid);
            DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrJ;
            STOP_SM(ds302_data._bootSlave[nodeid]);
            return;
        }

        // verify that 1F22 has an entry for me
        if (!(nodeid < Object1F22->bSubCount)) {
            // problem, no data
            DS302_DEBUG("ConciseDCF for %d: data for 0x1F22 does not include this slave (%d subcount)\n", nodeid, Object1F22->bSubCount);
            DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrJ;
            STOP_SM(ds302_data._bootSlave[nodeid]);
            return;
        }
        
        // get the raw data
        DATA_SM(ds302_data._bootSlave[nodeid]).dcfData = Object1F22->pSubindex[nodeid].pObject;
        DATA_SM(ds302_data._bootSlave[nodeid]).dcfSize = Object1F22->pSubindex[nodeid].size;
        DATA_SM(ds302_data._bootSlave[nodeid]).dcfCursor = 4; // see below why 4
        DATA_SM(ds302_data._bootSlave[nodeid]).dcfState = 0;
        
        // why 4? Because DCF data is UNS32 number of entries and then idx/subidx/datasize/data ... 4 means we have at least the size
        if (DATA_SM(ds302_data._bootSlave[nodeid]).dcfData == NULL || DATA_SM(ds302_data._bootSlave[nodeid]).dcfSize < 4) {
            // problem, empty data
            // is this a problem? Not having DCF data? Maybe we don't want to configure this slave?
            DS302_DEBUG("ConciseDCF for %d: data for 0x1F22 does not include this slave (empty data)\n", nodeid);
            DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrJ;
            STOP_SM(ds302_data._bootSlave[nodeid]);
            return;
        }
        
        // get the DCF count
        DATA_SM(ds302_data._bootSlave[nodeid]).dcfCount = DATA_SM(ds302_data._bootSlave[nodeid]).dcfData[0] | 
            DATA_SM(ds302_data._bootSlave[nodeid]).dcfData[1]<<8 | 
            DATA_SM(ds302_data._bootSlave[nodeid]).dcfData[2]<<16 | 
            DATA_SM(ds302_data._bootSlave[nodeid]).dcfData[3]<<24;
            
        DATA_SM(ds302_data._bootSlave[nodeid]).dcfLoadCount = 0;
        DS302_DEBUG("ConciseDCF for %d: initialised OK with %d entries to load\n", nodeid, DATA_SM(ds302_data._bootSlave[nodeid]).dcfCount);
    }

    /* the main SDO write loop takes place HERE. Init write, wait complete, repeat for next */
    // loop while we still think we have data to load
    while (DATA_SM(ds302_data._bootSlave[nodeid]).dcfCount > DATA_SM(ds302_data._bootSlave[nodeid]).dcfLoadCount) {
        
        if (DATA_SM(ds302_data._bootSlave[nodeid]).dcfState == 0) {
            
            UNS16   idx;
            UNS8    subidx;
            UNS32   size;
            UNS32   value;
            
            // it's the start of a new data item
            int retcode = ds302_get_next_dcf (
                DATA_SM(ds302_data._bootSlave[nodeid]).dcfData,
                &DATA_SM(ds302_data._bootSlave[nodeid]).dcfCursor,
                &idx, &subidx, &size, &value);
                
            if (retcode < 0) {
                // error, bug off

                DS302_DEBUG("ConciseDCF for %d: GOT DCF ERROR. Had %d, did %d\n", nodeid,
                    DATA_SM(ds302_data._bootSlave[nodeid]).dcfCount,
                    DATA_SM(ds302_data._bootSlave[nodeid]).dcfLoadCount);

                DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrJ;
                STOP_SM(ds302_data._bootSlave[nodeid]);
                return;                

            } else if (retcode == 0) {
                // EOS
                // Odd, we hit EOS before the stated number of items
                DS302_DEBUG("ConciseDCF for %d: apparently, we SHORT LOADED. Got EOS. Had %d, did %d\n", nodeid,
                    DATA_SM(ds302_data._bootSlave[nodeid]).dcfCount,
                    DATA_SM(ds302_data._bootSlave[nodeid]).dcfLoadCount);
                DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrJ;
                STOP_SM(ds302_data._bootSlave[nodeid]);
                return;                
            }
            
            // at this point I have the data, so I can proceed
            DATA_SM(ds302_data._bootSlave[nodeid]).dcfState = 1;
            
            UNS8 retcode2 = writeNetworkDictCallBackAI (d, nodeid,
                idx, subidx, 
                size, 0, &value, 
                _sm_BootSlave_downloadConfiguration,
                0,  // endianize? Probably need 1 since we do direct translation of values
                0   // block mode
                );
            
            if (retcode2 != 0) {
                // hit a send error
                DS302_DEBUG("ConciseDCF for %d: GOT DCF SDO SEND ERROR. Had %d, did %d\n", nodeid,
                    DATA_SM(ds302_data._bootSlave[nodeid]).dcfCount,
                    DATA_SM(ds302_data._bootSlave[nodeid]).dcfLoadCount);

                    DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrJ;
                STOP_SM(ds302_data._bootSlave[nodeid]);
                return;                                
            }
            
            // at this point we sent it
            // we let the loop run, should end up on "else" and get a IN PROGRESS and return waiting for complete on callback

            DS302_DEBUG("ConciseDCF for %d: started load of next item. Did %d, have %d\n", nodeid,
                DATA_SM(ds302_data._bootSlave[nodeid]).dcfLoadCount,
                DATA_SM(ds302_data._bootSlave[nodeid]).dcfCount);
            
        } else {
            // it's the continuation of a previous data item
            UNS8    retcode = getWriteResultNetworkDict (d, nodeid, &DATA_SM(ds302_data._bootSlave[nodeid]).errorCode);

            if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS)
                // do nothing, outside of callback call. Stupid
                return;

                /* Finalise last SDO transfer with this node */
            closeSDOtransfer(d, nodeid, SDO_CLIENT);  

            if(retcode != SDO_FINISHED) {
                // we had an error situation, abort
                DS302_DEBUG("ConciseDCF for %d: ABORT due to SDO error, %d/%x\n", nodeid, retcode, DATA_SM(ds302_data._bootSlave[nodeid]).errorCode);
                DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrJ;
                STOP_SM(ds302_data._bootSlave[nodeid]);
                return;
            }

            // write completed ok, switch state to 0 and redo call
            DATA_SM(ds302_data._bootSlave[nodeid]).dcfState = 0;
            // also increment the load count
            DATA_SM(ds302_data._bootSlave[nodeid]).dcfLoadCount++;

            DS302_DEBUG("ConciseDCF for %d: completed load of item %d. Have %d\n", nodeid,
                DATA_SM(ds302_data._bootSlave[nodeid]).dcfLoadCount,
                DATA_SM(ds302_data._bootSlave[nodeid]).dcfCount);
        }
    }
    
    DS302_DEBUG("ConciseDCF for %d: apparently, we loaded everything at this point. Had %d, did %d\n", nodeid,
        DATA_SM(ds302_data._bootSlave[nodeid]).dcfCount,
        DATA_SM(ds302_data._bootSlave[nodeid]).dcfLoadCount);
    
    if (0) {
        // we had an error in the configuration download
        //SM_ERROR(nodeid, SM_ErrJ);
        DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrJ;
        STOP_SM(ds302_data._bootSlave[nodeid]);
        return;
    } else {
        // configuration downloaded OK, go to error control
        //SM_SWITCH_STATE(SM_BOOTSLAVE_START_ERRCTL,d,nodeid)
        SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_START_ERRCTL, d, nodeid);
        return;
    }
}

void _sm_BootSlave_startErrorControlService(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_startErrorControlService\n");
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {      
        // code for the first run only
    
        // display node state for fun
        e_nodeState slavestate = getNodeState (d, nodeid);
        DS302_DEBUG("Node state for slave %d is %x\n", nodeid, slavestate);
        
        // is the heartbeat consumer non-zero for this node
        if (1) {
            
            // non-zero consumer, use HB
            //SM_SWITCH_STATE(SM_BOOTSLAVE_WAIT_HB,d,nodeid)
            SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_WAIT_HB, d, nodeid);
            return;
        } else {
            // zero consumer, use NodeGuard
            // is the node still on the Network list (0x1F81 bit 0)
            if (0) {
                // if yes, start node guard for it
                // verify node guard for it, 0x1F81 bits 2,3
                if (0) {
                    //node guard time non-zero
                    // start Node Guard for the node
                    //SM_SWITCH_STATE(SM_BOOTSLAVE_START_NODEGUARD,d,nodeid)
                    SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_START_NODEGUARD, d, nodeid);
                    return;
                } else {
                    //node guard time zero
                    // go to error control started
                    //SM_SWITCH_STATE(SM_BOOTSLAVE_ERRCTL_STARTED,d,nodeid)
                    SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_ERRCTL_STARTED, d, nodeid);
                    return;
                }
            } else {
                // if no, error control completed OK
                // go to error control started
                //SM_SWITCH_STATE(SM_BOOTSLAVE_ERRCTL_STARTED,d,nodeid)
                SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_ERRCTL_STARTED, d, nodeid);
                return;
            }
        }
    }
// nothing happens for subsequent runs, should not end here
}

void _sm_BootSlave_waitHeartbeat(CO_Data* d, UNS8 nodeid)
{    
    DS302_DEBUG("_sm_BootSlave_waitHeartbeat\n");
    // dummy for now
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {
        
        // register the start here. We are going to wait
        DATA_SM(ds302_data._bootSlave[nodeid]).ecsStart = rtuClock();
        
        //SM_SWITCH_STATE(SM_BOOTSLAVE_ERRCTL_STARTED,d,nodeid)
        //SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_ERRCTL_STARTED, d, nodeid);
        //return;
    }
    
    e_nodeState slavestate = getNodeState (d, nodeid);

    if (slavestate == Operational || slavestate == Pre_operational || slavestate == Stopped) {
        // means we have a heartbeat here
        DS302_DEBUG("Node state for slave %d is %x. We have a heartbeat\n", nodeid, slavestate);
        SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_ERRCTL_STARTED, d, nodeid);
        return;
    }
    
    // check if time elapsed
    uint64_t    elapsedTime = rtuClock() - DATA_SM (ds302_data._bootSlave[nodeid]).ecsStart;
    // need to update to the proper value
    if (elapsedTime > 2*1000*1000) {
        // allow for the HB time to see a change
        
        DS302_DEBUG("HB wait time for %d elapsed (%d), we have a problem\n", nodeid, elapsedTime);
        DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrK;
        STOP_SM(ds302_data._bootSlave[nodeid]);
        return;        
    }
    
    // register the alarm for this one for 100ms
    // is this an ugly hack or what? Looks like we have an ID collision
    SetAlarm (d, nodeid | nodeid << 24, (TimerCallback_t)_sm_BootSlave_waitHeartbeat, MS_TO_TIMEVAL(100), 0);
}

void _sm_BootSlave_startNodeGuard(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_startNodeGuard\n");
    // dummy for now
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {
        //SM_SWITCH_STATE(SM_BOOTSLAVE_ERRCTL_STARTED,d,nodeid)
        SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_ERRCTL_STARTED, d, nodeid);
        return;
    }
}

void _sm_BootSlave_errorControlStarted(CO_Data* d, UNS8 nodeid)  // here we need to check if we went D/noconfig and exit
{
    DS302_DEBUG("_sm_BootSlave_errorControlStarted\n");
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {
        // simple decisional node, have we went via D path or not
        if (DATA_SM(ds302_data._bootSlave[nodeid]).ViaDPath) {
            //SM_ERROR(nodeid, SM_ErrL);
            DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_ErrL;
            STOP_SM(ds302_data._bootSlave[nodeid]);
            return;
        } else {
            //SM_SWITCH_STATE(SM_BOOTSLAVE_START_SLAVE,d,nodeid)
            SWITCH_SM (ds302_data._bootSlave[nodeid], SM_BOOTSLAVE_START_SLAVE, d, nodeid);
            return;
        }
    }
}

void _sm_BootSlave_startSlave(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_startSlave\n");
    if (INITIAL_SM(ds302_data._bootSlave[nodeid])) {
        
        // am I allowed to start the nodes? 0x1F80 bit 3
        if (ds302_bitcheck_32(d, 0x1F80, 0x00, DS302_DEVICE_MANUAL_START_SLAVE) == 0) {
            // yes, I am
            // do I have to start each one individually? 0x1F80 bit 1
            if (ds302_bitcheck_32(d, 0x1F80, 0x00, DS302_DEVICE_START_ALL_SLAVES) == 0) {
                // yes, I have
                // well, then start the node

                // ...
                masterSendNMTstateChange (d, nodeid, NMT_Start_Node);

                // we're done here
                //SM_ERROR(nodeid, SM_OK);
                DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_OK;
                STOP_SM(ds302_data._bootSlave[nodeid]);
                return;
            } else {
                // no I don't
                // is my state Operational?
                if (getState(d) == Operational) {
                    // yes it is
                    // start the node
                    
                    // ...
                    masterSendNMTstateChange (d, nodeid, NMT_Start_Node);
        
                    // we're done here
                    //SM_ERROR(nodeid, SM_OK);
                    DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_OK;
                    STOP_SM(ds302_data._bootSlave[nodeid]);
                    return;
                } else {
                    // nope, we're done here
                    //SM_ERROR(nodeid, SM_OK);
                    DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_OK;
                    STOP_SM(ds302_data._bootSlave[nodeid]);
                    return;
                }
            }
        } else {
            // nope
            // we're done here
            //SM_ERROR(nodeid, SM_OK);
            DATA_SM(ds302_data._bootSlave[nodeid]).result = SM_OK;
            STOP_SM(ds302_data._bootSlave[nodeid]);
            return;
        }
    } else {

        // we're at the end of the road and we've been called TWICE
        eprintf ("BootNode: start slave called twice, odd!\n");
    }
}



/*
  Helper functions
*/

/*
  General bit checking for OD
  in: index, subindex, bitmask
  out: -1 on error, 0 or 1 depending on bitmask ((data & bitmask) == bitmask)
*/
int	ds302_bitcheck_32 (CO_Data* d, UNS16 idx, UNS8 subidx, UNS32 bitmask)
{
    const indextable *      ODindex;
    UNS32                   errorCode;

    ODindex = (*d->scanIndexOD)(d, idx, &errorCode);
    if (errorCode != OD_SUCCESSFUL)
        return -1;

    // check for subidx to be present
    if (subidx > ODindex->bSubCount)
        return -1;

    return (*(UNS32 *)ODindex->pSubindex[subidx].pObject & bitmask) == bitmask;
}

int	ds302_nl_keepalive_nodes_present(CO_Data* d)
{
    DS302_DEBUG("ds302_nl_keepalive_nodes_present\n");
    const indextable *	Object1F81;
    UNS32		errorCode;

    Object1F81 = (*d->scanIndexOD)(d, 0x1F81, &errorCode);
    if (errorCode != OD_SUCCESSFUL)
        return -1;

    // check all the objects for the presence of keepalive
    int	nodeid;
    for (nodeid = 1; nodeid < Object1F81->bSubCount; nodeid++) {
        if ((*(UNS32 *)Object1F81->pSubindex[nodeid].pObject & DS302_NL_DONOT_RESET) && (*(UNS32 *)Object1F81->pSubindex[nodeid].pObject & DS302_NL_IS_SLAVE))
            return 1;
    }

    return 0;
}

int	ds302_nl_send_reset_to_non_keepalive(CO_Data* d)
{
    const indextable *      Object1F81;
    UNS32           	errorCode;

    Object1F81 = (*d->scanIndexOD)(d, 0x1F81, &errorCode);
    if (errorCode != OD_SUCCESSFUL)
            return 0;

    // check all the objects for the presence of keepalive
    int     nodeid;
    for (nodeid = 1; nodeid < Object1F81->bSubCount; nodeid++) {
        if (*(UNS32 *)Object1F81->pSubindex[nodeid].pObject & DS302_NL_IS_SLAVE) {
            // node is in list
            if ((*(UNS32 *)Object1F81->pSubindex[nodeid].pObject & DS302_NL_DONOT_RESET) == 0) {
                // node does not have keepalive set, send reset comm
                eprintf ("Send reset to node id %d\n", nodeid);
                masterSendNMTstateChange (d, nodeid, NMT_Reset_Comunication);
            } else {
                eprintf ("DO NOT send reset to node id %d, has KEEPALIVE set\n", nodeid);
            }
        }
    }
}


/*
  verifies if node in network list
*/
int	ds302_nl_node_in_list(CO_Data* d, UNS8 nodeid)
{
    const indextable *      Object1F81;
    UNS32           errorCode;

    Object1F81 = (*d->scanIndexOD)(d, 0x1F81, &errorCode);
    if (errorCode != OD_SUCCESSFUL)
        return 0;

    if (nodeid < Object1F81->bSubCount && nodeid > 0) {

        return (*(UNS32 *)Object1F81->pSubindex[nodeid].pObject & DS302_NL_IS_SLAVE) != 0;
    } else {

        return 0;
    }
}

/*
  verifies if node is a mandatory slave
*/

int     ds302_nl_mandatory_node(CO_Data* d, UNS8 nodeid)
{
    const indextable *      Object1F81;
    UNS32           errorCode;

    Object1F81 = (*d->scanIndexOD)(d, 0x1F81, &errorCode);
    if (errorCode != OD_SUCCESSFUL)
        return -1;

    if (nodeid < Object1F81->bSubCount && nodeid > 0) {

        return ((*(UNS32 *)Object1F81->pSubindex[nodeid].pObject & DS302_NL_IS_SLAVE) != 0) && ((*(UNS32 *)Object1F81->pSubindex[nodeid].pObject & DS302_NL_MANDATORY) != 0);
    } else {

        return 0;
    }
}

int ds302_all_mandatory_booted (CO_Data* d)
{
    const indextable *      Object1F81;
    UNS32                   errorCode;

    Object1F81 = (*d->scanIndexOD)(d, 0x1F81, &errorCode);
    if (errorCode != OD_SUCCESSFUL)
        return -1;

    // check all the objects for the boot status if mandatory slaves
    int     nodeid;  
    for (nodeid = 1; nodeid < Object1F81->bSubCount; nodeid++) {
        if (*(UNS32 *)Object1F81->pSubindex[nodeid].pObject & DS302_NL_IS_SLAVE) {
            // node is in list
            if (*(UNS32 *)Object1F81->pSubindex[nodeid].pObject & DS302_NL_MANDATORY) {
                // node is mandatory
                if (DATA_SM(ds302_data._bootSlave[nodeid]).state != BootCompleted) {
                    DS302_DEBUG("Mandatory slave not booted\n");
                    eprintf ("Slave ID is %d, state is %d\n", nodeid, DATA_SM(ds302_data._bootSlave[nodeid]).state);
                    return 0;
                }
            }
        }
    }

    return 1;
}

void _sm_BootMaster_initial (CO_Data* d, UNS32 idx)
{
    eprintf ("_sm_BootMaster_initial ENTRY\n");

    if (INITIAL_SM(ds302_data._masterBoot)) {

        // no longer doing data init
        /* initialize data structures */
        //int     i;

        // initialize the boot state machines   
        /*for (i = 0; i < NMT_MAX_NODE_ID; i++)
                SM_INIT (i);*/
    
        // initialize the boot signals
        /*for (i = 0; i < NMT_MAX_NODE_ID; i++)
                ds302_boot_data[i] = BootInitialised;*/

        if (ds302_bitcheck_32(d, 0x1F80, 0x00, DS302_DEVICE_NMT_MASTER) != 1) {
            // enter slave state
            DS302_DEBUG("I am not a master, so not booting\n");
            // not needed, stop the boot machine
            STOP_SM(ds302_data._masterBoot);
            return;
        }


        if (ds302_nl_keepalive_nodes_present(d)) {

            DS302_DEBUG("Keep alive some, send Reset Comm\n");
            // send NMT_Reset_Comms to the nodes where keepalive not set
            ds302_nl_send_reset_to_non_keepalive(d);

        } else {

            DS302_DEBUG("Broadcast, send Reset Comm\n");
            // send a broadcast NMT_Reset_Comms
            masterSendNMTstateChange (d, 0, NMT_Reset_Comunication);
        }
    }

    // go straight to the boot process, always
    SWITCH_SM(ds302_data._masterBoot, MB_BOOTPROC, d, idx);
}

void _sm_BootMaster_bootproc (CO_Data* d, UNS32 idx)
{
    int	slaveid;

    eprintf ("_sm_BootMaster_bootproc ENTRY (%d)\n", idx);

    // this is called multiple times until all mandatory slaves are done
    // there is no initial at this point, it's lower in the process

    // verify if we are done here
    if (ds302_all_mandatory_booted(d)) {
        // switch to the next step
        SWITCH_SM(ds302_data._masterBoot, MB_OPERWAIT, d, idx);
        return;
    }

    // if we are not yet done, process the state machines
    if (INITIAL_SM(ds302_data._masterBoot)) {

        // this is the first run, so start all the state machines for the slaves in the network list
        eprintf ("_sm_BootMaster_bootproc INITIAL\n");

        for (slaveid=1; slaveid<NMT_MAX_NODE_ID; slaveid++)
            if (ds302_nl_node_in_list(d, slaveid)) {
                // it's a slave, so start it's machine
                
                // mark the machine as used
                DATA_SM (ds302_data._bootSlave[slaveid]).state = BootInitialised;
                
                // mark the start time
                //SM_DATA(i,start_time) = rtuClock();
                DATA_SM(ds302_data._bootSlave[slaveid]).bootStart = rtuClock();
                
                //EnterMutex();
                //SM_RUN_MACHINE(d, i);
                START_SM(ds302_data._bootSlave[slaveid], d, slaveid);
                //LeaveMutex();
            } else {
                
                // it's unused based on state, BootUnused.
                
                // zero out the start time to indicate unused
                //SM_DATA(i,start_time) = 0;
            }
        // at this point we're done here
    } 

    // additional calls
    eprintf ("_sm_BootMaster_bootproc N+1\n");

    // process the entire state machine list, until all are done
    // if all are done, re-do the mandatory check and switch, or RAISE HELL (all machines done but NO JOY on mandatory)
    int	all_slaves_booted = 1;
    
    for (slaveid=1; slaveid<NMT_MAX_NODE_ID; slaveid++) {
        if (DATA_SM (ds302_data._bootSlave[slaveid]).state != BootUnused)
        {
            // machine is in use, so work it
            if (RUNNING_SM(ds302_data._bootSlave[slaveid])) {
                // machine still running, skip it
                DS302_DEBUG("Slave boot %d still running\n");
                all_slaves_booted = 0;
                continue;
            }
            
            // at this point we found a used SM that is no longer running
            if (!STOPPED_SM(ds302_data._bootSlave[slaveid])) {
                // small sanity checking, it SHOULD be stopped
                DS302_DEBUG ("Found a used SM not running and NOT stopped, %d", slaveid);
            }
            
            // we're sure the machine finished running
            
            DS302_DEBUG("Slave boot %d finished\n");
            
            _sm_BootSlave_Codes     result = DATA_SM(ds302_data._bootSlave[slaveid]).result;
            
            //if not mandatory slave, signal boot attempted
            if (!ds302_nl_mandatory_node(d, slaveid)) {
                DATA_SM (ds302_data._bootSlave[slaveid]).state = BootAttempted;
            }
            
            if (result == SM_ErrB) {
                // get current time
                uint64_t		elapsedTime = rtuClock() - DATA_SM (ds302_data._bootSlave[slaveid]).bootStart;
                
                DS302_DEBUG ("Got status B for SM %d, elapsed time %d\n", slaveid, elapsedTime);
                
                // check if it's mandatory
                if (ds302_nl_mandatory_node(d, slaveid)) {
                    if (elapsedTime > NODE_BOOT_TIME) {
                        // boot expired for this
                        DS302_DEBUG("Boot expired for mandatory slave %d (time %d)\n", slaveid, elapsedTime);
                        // signal result
                        DATA_SM (ds302_data._bootSlave[slaveid]).state = BootTimedOut;
                        continue;
                        
                    } else {
                        // re-initialize the state machine
                        
                        DS302_DEBUG("Boot time not expired for slave %d, rescheduling\n", slaveid);
                        INIT_SM (BOOTSLAVE, ds302_data._bootSlave[slaveid], SM_BOOTSLAVE_INITIAL);
                        START_SM (ds302_data._bootSlave[slaveid], d, slaveid);
                        
                        // mark that we're not done yet
                        all_slaves_booted = 0;
                        continue;
                    }
                } else {
                    // regular slave that finished. What do I do here???
                    DS302_DEBUG("Optional slave %d ended with status B. Now what?", slaveid);
                }
            } else if (result != SM_OK) {
                // we ended with a boot error and it wasn't B
                DS302_DEBUG("Result is NOT OK, BootTimedOut for %d <|==\n", slaveid);
                // signal state
                DATA_SM (ds302_data._bootSlave[slaveid]).state = BootError;
            } else {
                // we ended with a successfull run
                DS302_DEBUG("Result is OK, BootCompleted for %d <---\n", slaveid);
                // signal state
                DATA_SM (ds302_data._bootSlave[slaveid]).state = BootCompleted;
            }
        } else {
            // unused machine, skip
            continue;
        }
    }


    if (all_slaves_booted) {
        // all the slaves have completed boot
        // re-verify if we are done here
        if (ds302_all_mandatory_booted(d)) {
            // switch to the next step
            SWITCH_SM(ds302_data._masterBoot, MB_OPERWAIT, d, idx);
            return;
        } else {
            // well, crap
            DS302_DEBUG ("ALL DONE BUT NOT FINISHED!!!\n");
            STOP_SM(ds302_data._masterBoot);
            return;
        }
    } else {
        // we still have slaves to manage
        //DS302_DEBUG("BM: not all done, waiting 100ms\n");
        // set alarm for 100ms for this function
        //DS302_DEBUG("_sm_BootMaster_bootproc ALARM SET to %d\n", ++idx);
        SetAlarm (d, idx, _sm_BootMaster_bootproc, MS_TO_TIMEVAL(100), 0);
    }
}

void _sm_BootMaster_operwait (CO_Data* d, UNS32 idx)
{
    DS302_DEBUG ("_sm_BootMaster_operwait ENTRY (%d)\n", idx);

    if (INITIAL_SM(ds302_data._masterBoot)) {
        if (!ds302_all_mandatory_booted(d)) {
            eprintf ("We should not be here, but here we are! Not ALL mandatory booted OK\n");
            STOP_SM(ds302_data._masterBoot);
            return;
        }

        // shall I go Operational automagically?
        if (ds302_bitcheck_32(d, 0x1F80, 0x00, DS302_DEVICE_MANUAL_OPERATIONAL) == 0) {
            // automatic go to operational

            DS302_DEBUG("Entering operational\n");
            setState(d, Operational);

            // switch to start slaves
            SWITCH_SM(ds302_data._masterBoot, MB_SLAVESTART, d, idx);
            return;
        }
    }

    DS302_DEBUG ("_sm_BootMaster_operwait N+1 (timer based I hope)\n");

    // verify if we're operational yet
    if (getState (d) == Operational) {
        DS302_DEBUG("Was put externally into operational, switching to MB_SLAVESTART\n");
        
        // switch to start slaves
        SWITCH_SM(ds302_data._masterBoot, MB_SLAVESTART, d, idx);
        return;
    }

    DS302_DEBUG("_sm_BootMaster_bootproc ALARM SET to %d\n", ++idx);
    SetAlarm (d, idx, _sm_BootMaster_bootproc, MS_TO_TIMEVAL(100), 0);
}

void _sm_BootMaster_slavestart (CO_Data* d, UNS32 idx)
{
    DS302_DEBUG ("_sm_BootMaster_slavestart ENTRY (%d)\n", idx);

        if (INITIAL_SM(ds302_data._masterBoot)) {

            // Only execute this when I am operational
            // this enables multiple executions for this function
            if ((ds302_bitcheck_32(d, 0x1F80, 0x00, DS302_DEVICE_MANUAL_START_SLAVE) == 0) && getState(d) == Operational) {
        
                    DS302_DEBUG("Start slaves once operational\n");

                    // decide on broadcast or individual based on DS302_DEVICE_START_ALL_SLAVES

                    if (ds302_bitcheck_32(d, 0x1F80, 0x00, DS302_DEVICE_MANUAL_START_SLAVE) == 1) {
                            DS302_DEBUG("Start slaves with a broadcast\n");
                            masterSendNMTstateChange (d, 0, NMT_Start_Node);
                    } else {
                            DS302_DEBUG("Start slaves individually\n");
                            //start slaves individually
                            int slaveid;
                            int myid = getNodeId(d);
                            for (slaveid=1; slaveid < NMT_MAX_NODE_ID; slaveid++) {
                                    // make sure to skip myself
                                    if ((slaveid != myid) && ds302_nl_node_in_list(d, slaveid))
                                            masterSendNMTstateChange (d, slaveid, NMT_Start_Node);
                            }
                    }
            }
    }

    DS302_DEBUG("MasterBoot - STOP STATE MACHINE (%d). We completed the boot process\n", idx);
    // should not end up here, but just in case
    STOP_SM(ds302_data._masterBoot);
    
    // mark boot completed
    ds302_data.bootState = BootCompleted;
}

void ds302_boot_slave (CO_Data* d, UNS8 slaveid)
{
    if (ds302_nl_node_in_list(d, slaveid)) {
        ds302_init_slaveSM (d, slaveid);
        // mark the machine as used
        DATA_SM (ds302_data._bootSlave[slaveid]).state = BootInitialised;
        // mark the start time
        DATA_SM(ds302_data._bootSlave[slaveid]).bootStart = rtuClock();
        START_SM(ds302_data._bootSlave[slaveid], d, slaveid);
    }
}

void ds302_init_slaveSM (CO_Data* d, UNS8 slaveid)
{
    // init the slave SM
    INIT_SM (BOOTSLAVE, ds302_data._bootSlave[slaveid], SM_BOOTSLAVE_INITIAL);
    // init the slave data
    DATA_SM (ds302_data._bootSlave[slaveid]).state = BootUnused;
    DATA_SM (ds302_data._bootSlave[slaveid]).result = SM_Initialised;
    DATA_SM (ds302_data._bootSlave[slaveid]).ViaDPath = 0;
    DATA_SM (ds302_data._bootSlave[slaveid]).bootStart = 0;
    DATA_SM (ds302_data._bootSlave[slaveid]).Index1000 = 0x0;
    DATA_SM (ds302_data._bootSlave[slaveid]).Index1018_1 = 0x0;
    DATA_SM (ds302_data._bootSlave[slaveid]).Index1018_2 = 0x0;
    DATA_SM (ds302_data._bootSlave[slaveid]).Index1018_3 = 0x0;
    DATA_SM (ds302_data._bootSlave[slaveid]).Index1018_4 = 0x0;
    DATA_SM (ds302_data._bootSlave[slaveid]).Index1020_1 = 0x0;
    DATA_SM (ds302_data._bootSlave[slaveid]).Index1020_2 = 0x0;
}

/* initialises the DS 302 structure */
void ds302_init (CO_Data* d)
{
    // init the DS-302 master data
    ds302_data.bootState = BootInitialised;
    INIT_SM (BOOTMASTER, ds302_data._masterBoot, MB_INITIAL);
    
    // initialize the slave state machines
    int slaveid;
    for (slaveid = 1; slaveid < NMT_MAX_NODE_ID; slaveid ++){
        // init the slave SM
        INIT_SM (BOOTSLAVE, ds302_data._bootSlave[slaveid], SM_BOOTSLAVE_INITIAL);
        // init the slave data
        DATA_SM (ds302_data._bootSlave[slaveid]).state = BootUnused;
        DATA_SM (ds302_data._bootSlave[slaveid]).result = SM_Initialised;
        DATA_SM (ds302_data._bootSlave[slaveid]).ViaDPath = 0;
        DATA_SM (ds302_data._bootSlave[slaveid]).bootStart = 0;
        DATA_SM (ds302_data._bootSlave[slaveid]).Index1000 = 0x0;
        DATA_SM (ds302_data._bootSlave[slaveid]).Index1018_1 = 0x0;
        DATA_SM (ds302_data._bootSlave[slaveid]).Index1018_2 = 0x0;
        DATA_SM (ds302_data._bootSlave[slaveid]).Index1018_3 = 0x0;
        DATA_SM (ds302_data._bootSlave[slaveid]).Index1018_4 = 0x0;
        DATA_SM (ds302_data._bootSlave[slaveid]).Index1020_1 = 0x0;
        DATA_SM (ds302_data._bootSlave[slaveid]).Index1020_2 = 0x0;
    }
    
    // ready to proceed. Need to decide if we boot ON-DEMAND or we rely on boot messages
    // relying on boot messages seems to be a problem
    // we can WAIT for a boot message before starting the boot process for example
    // in order to ensure the slave is ready to process commands
    
}

void ds302_start (CO_Data* d)
{
    ds302_data.bootState = BootRunning;
    
    // start the boot with a idx above the CAN IDs
    START_SM(ds302_data._masterBoot, d, NMT_MAX_NODE_ID);
}

ds302_boot_state_t  ds302_status (CO_Data* d)
{
    return ds302_data.bootState;
}

/*
    Gets the DCF data at current cursor and updates the cursor to the next one
    returns 1 if ok
    returns 0 if EOS
    returns -1 if error
*/
int     ds302_get_next_dcf (UNS8 *data, UNS32 *cursor, UNS16 *idx, UNS8 *subidx, UNS32 *size, UNS32 *value)
{
    // question... How to detect EOS??? We're going sequentially
    // if index == 0 means EOS??? This is one idea, but pretty stupid
    if (data == NULL || (*cursor < 4))
        return -1;
    
    *idx = data[(*cursor)++] | data[(*cursor)++] << 8;
    if (*idx == 0)
        return 0;
    
    *subidx = data[(*cursor)++];
    *size = data[(*cursor)++] | data[(*cursor)++] << 8 | data[(*cursor)++] << 16 | data[(*cursor)++] << 24;
    
    if (*size > 4 || *size < 1)
        return -1;
    
    // and now load the data. we can only treat 32 bit values max
    switch (*size) {
        
        case 1:
            *value = data[(*cursor)++];
            break;
        case 2:
            *value = data[(*cursor)++] | data[(*cursor)++] << 8;
            break;
        case 3:
            DS302_DEBUG("Got a 3 byte data, this is stupid!!!\n");
            *value = data[(*cursor)++] | data[(*cursor)++] << 8 | data[(*cursor)++] << 16;
            break;
        case 4:
            *value = data[(*cursor)++] | data[(*cursor)++] << 8 | data[(*cursor)++] << 16 | data[(*cursor)++] << 24;
            break;
    }
    
    return 1;
}

/*
    Loads the DCF data for the local node
*/
int     ds302_load_dcf_local (CO_Data* d)
{
    const indextable *	Object1F22;
    UNS32               errorCode;
    UNS8                nodeid = getNodeId(d);

    Object1F22 = (*d->scanIndexOD)(d, 0x1F22, &errorCode);
    if (errorCode != OD_SUCCESSFUL) {
        DS302_DEBUG("ConciseDCF for %d: can not get data for 0x1F22\n", nodeid);
        return -1;
    }

    // verify that 1F22 has an entry for me
    if (!(nodeid < Object1F22->bSubCount)) {
        // problem, no data
        DS302_DEBUG("ConciseDCF for %d: data for 0x1F22 does not include the master (%d subcount)\n", nodeid, Object1F22->bSubCount);
        return -1;
    }
        
    // get the raw data
    UNS8* dcfData = Object1F22->pSubindex[nodeid].pObject;
    UNS32 dcfSize = Object1F22->pSubindex[nodeid].size;
    UNS32 dcfCursor = 4; // see below why 4
        
    // why 4? Because DCF data is UNS32 number of entries and then idx/subidx/datasize/data ... 4 means we have at least the size
    if (dcfData == NULL || dcfSize < 4) {
        // problem, empty data
        // is this a problem? Not having DCF data? Maybe we don't want to configure this slave?
        DS302_DEBUG("ConciseDCF for %d: data for 0x1F22 does not include the master (empty data)\n", nodeid);
        return -1;
    }

    DS302_DEBUG("We have data size of %d\n", dcfSize);
        
    // get the DCF count
    UNS32 dcfCount = dcfData[0] | dcfData[1]<<8 | dcfData[2]<<16 | dcfData[3]<<24;
    UNS32 dcfLoadCount = 0;
    
    DS302_DEBUG("ConciseDCF for %d: initialised OK with %d entries to load\n", nodeid, dcfCount);


    while (dcfCount > dcfLoadCount) {
        
        UNS16   idx;
        UNS8    subidx;
        UNS32   size;
        UNS32   value;
        
        // it's the start of a new data item
        int retcode = ds302_get_next_dcf (dcfData, &dcfCursor,
            &idx, &subidx, &size, &value);
                
        if (retcode < 0) {
            // error, bug off
            DS302_DEBUG("ConciseDCF for %d: GOT DCF ERROR. Had %d, did %d\n", nodeid, dcfCount, dcfLoadCount);
            return -1;
        } else if (retcode == 0) {
            // EOS
            // Odd, we hit EOS before the stated number of items
            DS302_DEBUG("ConciseDCF for %d: apparently, we SHORT LOADED. Got EOS. Had %d, did %d\n", nodeid, dcfCount, dcfLoadCount);
            return -1;
        }
            
        // at this point I have the data, so I can proceed
        errorCode = writeLocalDict (d,
            idx, subidx, 
            &value, &size,
            0 
            );
            
        if (errorCode != OD_SUCCESSFUL) {
            // hit a send error
            DS302_DEBUG("ConciseDCF for %d: GOT DCF DICT WRITE ERROR. Had %d, did %d\n", nodeid, dcfCount, dcfLoadCount);
            return -1;
        }
        
        dcfLoadCount++;
    }
    
    return dcfLoadCount;
}

int ds302_setHeartbeat (CO_Data* d, UNS8 nodeid, UNS16 heartbeat) {
    
    UNS32               errorCode;
    UNS32               hbdata;
    UNS32               size = sizeof (hbdata);
    
    if (nodeid < 1 || nodeid > NMT_MAX_NODE_ID)
        return 0;
    
    hbdata = nodeid << 16 | heartbeat;
    
    errorCode = writeLocalDict (d,
        0x1016, nodeid, 
        &hbdata, &size,
        0 
        );

    if (errorCode != OD_SUCCESSFUL) {
        // hit a send error
        return 0;
    }
    
    return 1;
}