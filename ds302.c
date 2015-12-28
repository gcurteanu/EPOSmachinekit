#include "canfestival.h"
#include "EPOScontrol.h"
#include "data.h"
#include <time.h>
#include "ds302.h"

//#define DS302_DEBUG(msg)
#define DS302_DEBUG(...) eprintf(__VA_ARGS__)

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

DECLARE_SM(BOOTMASTER, _masterBoot);

void test() {
DECLARE_SM(BOOTMASTER,myMachine);
INIT_SM(BOOTMASTER,myMachine,0);
RUN_SM(myMachine, NULL, 0);
}

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
ds302_boot_state_t ds302_boot_data[NMT_MAX_NODE_ID];

// move to a pointer based state machine, using functions that are also callback capable for SDOs
// type for functions is SDOCallback_t

/*
 BootSlave state machine
*/

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


/*
  Does the initialisation part of DS 302
*/
void ds302_preOperational_preBoot (CO_Data* d)
{
    /* dummy vars pending real structures */
    int	lssrequired = 0;

    eprintf ("ds302_preOperational_preBoot ENTRY\n");

    /* initialize data structures */
    int	i;

    // initialize the boot state machines	
    for (i = 0; i < NMT_MAX_NODE_ID; i++)
        SM_INIT (i);

    // initialize the boot signals
    for (i = 0; i < NMT_MAX_NODE_ID; i++)
        ds302_boot_data[i] = BootInitialised;

    if (ds302_bitcheck_32(d, 0x1F80, 0x00, DS302_DEVICE_NMT_MASTER) != 1) {
        // enter slave state
        DS302_DEBUG("I am not a master, so not booting\n");
        // not needed
        return;
    }

    if (lssrequired)
        // no LSS support for now
        return;

    if (ds302_nl_keepalive_nodes_present(d)) {

        DS302_DEBUG("Keep alive some, send Reset Comm\n");
        // send NMT_Reset_Comms to the nodes where keepalive not set
        ds302_nl_send_reset_to_non_keepalive(d);

    } else {

        DS302_DEBUG("Broadcast, send Reset Comm\n");
        // send a broadcast NMT_Reset_Comms
        masterSendNMTstateChange (d, 0, NMT_Reset_Comunication);
    }

    eprintf ("ds302_preOperational_preBoot DONE\n");
}

/*
  does the DS 302 after the boot slave process finished
*/
void ds302_preOperational_postBoot (CO_Data* d)
{
    int	startallslaves = 1;

    // boot process

    eprintf ("ds302_preOperational_postBoot ENTRY\n");

    if (ds302_all_mandatory_booted(d) != 1)
    {
        DS302_DEBUG("Not all mandatory slaves booted!\n");
        return;
    }

    if (ds302_bitcheck_32(d, 0x1F80, 0x00, DS302_DEVICE_MANUAL_OPERATIONAL) == 0) {

        DS302_DEBUG("Entering operational\n");
        setState(d, Operational);
    } else {
        
        DS302_DEBUG("Waiting to be put into operational\n");
        // wait in loop to be operational ???
        // there has to be a better way
        while (getState(d) != Operational)
            sleep (1);

        DS302_DEBUG("Was put into operational mode\n");
    }

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

    // boot up done

    eprintf ("ds302_preOperational_postBoot DONE\n");
}

static int sleep_ms ( unsigned long ms )
{
    if ( ms >= 1000 ) ms = 1000;
    return usleep ( ms * 1000 );
}

void ds302_slaveBootprocess (CO_Data* d)
{
    // bit tricky. Will have to do this on parallel threads?
    // 

    eprintf ("ds302_slaveBootprocess ENTRY\n");

    int optionalslave = 0;
    int nodeismandatory = 1;
    int bootexpired = 1;

    // start boot one slave in sequence
    // for slave in list...

    // for now, we only do slave 0x01

    _sm_BootSlave_Codes	result;

    // we can leverage the bootup callback here!
    while (1) {

        // run the state machine until DONE

        DS302_DEBUG("Run state machine for 0x01\n");
        eprintf ("SM for 0x01 is: %d %d %d\n", ds302_slaveBootSM[0x01].machine_state, ds302_slaveBootSM[0x01].current_state,  ds302_slaveBootSM[0x01].step_iter);

        // run the machine ONCE, since once triggered relies on callbacks
        //EnterMutex();
        SM_RUN_MACHINE(d, 0x01);
        //LeaveMutex();

        while (!SM_IS_FINISHED(0x01)) {
            // sleep 100ms
            sleep_ms (100);	
        }
        eprintf ("SM for 0x01 is: %d %d %d\n", ds302_slaveBootSM[0x01].machine_state, ds302_slaveBootSM[0x01].current_state,  ds302_slaveBootSM[0x01].step_iter);

        // machine finished
        
        eprintf ("Machine for node ID 0x01 finished. %d, %s\n", SM_DATA(0x01,machine_state), SM_ERR_MSG(SM_DATA(0x01,machine_state)));

        result = SM_DATA(0x01,machine_state);

        // signal on optional slave
        if (optionalslave)
            ds302_boot_data[0x01] = BootAttempted;

        if (result == SM_ErrB) {
            DS302_DEBUG("Result is B,  waiting\n");
            if (nodeismandatory && bootexpired) { 
                // inform application
                eprintf ("Can't start node 0x01 and it's mandatory\n");
                DS302_DEBUG("Boot expired for mandatory\n");
                break;
            } else {
                DS302_DEBUG("Sleeping for 1 second to retry\n");
                // sleep 1 second
                sleep (1);
            }

        } else if (result != SM_OK) {
            DS302_DEBUG("Result is NOT OK, BootTImedOut, exiting loop\n");
            // inform application
            eprintf ("Node start for 0x01 returned %d\n", result);
            // signal boot failed
            ds302_boot_data[0x01] = BootTimedOut;
            break;
        } else {
            DS302_DEBUG("Result is OK, BootCompleted, exiting loop\n");
            // everything is fine, exit loop
            // signal completed ok
            ds302_boot_data[0x01] = BootCompleted;
            break;
        }

    }

    DS302_DEBUG("Boot process done, checking slave statuses\n");
    while (ds302_boot_data[0x01] == BootInitialised) {
        // in this case we should WAIT until something DID happen with the node
        sleep (1);
    }

    eprintf ("ds302_slaveBootprocess DONE\n");
}

void _sm_BootSlave_initial(CO_Data* d, UNS8 nodeid)
{
    // dummy checking for 0x1F81 bit 0

    DS302_DEBUG("_sm_BootSlave_initial (%d)\n", nodeid);

    if (0) {
        DS302_DEBUG("_sm_BootSlave_initial Error A (%d)\n", nodeid);
        SM_ERROR(nodeid, SM_ErrA);
    } else {
        DS302_DEBUG("_sm_BootSlave_initial switch to SM_BOOTSLAVE_GET_DEVTYPE (%d)\n", nodeid);
        // go to the next state
        SM_SWITCH_STATE(SM_BOOTSLAVE_GET_DEVTYPE,d,nodeid)
    }
}

void _sm_BootSlave_getDeviceType(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_getDeviceType (%d)\n", nodeid);

    if (SM_INITIAL(nodeid)) {
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

    UNS32	size = sizeof(SM_DATA(nodeid,Index1000));
    UNS8	retcode = getReadResultNetworkDict (d, nodeid, &SM_DATA(nodeid,Index1000), &size, &SM_DATA(nodeid,error_code));

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS) {
        DS302_DEBUG("_sm_BootSlave_getDeviceType SDO op in progress (%d)\n", nodeid);
        // do nothing, outside of callback call
        return;
    }

        if(retcode != SDO_FINISHED) {
        DS302_DEBUG("_sm_BootSlave_getDeviceType SDO error (%d) = %x\n", nodeid, retcode);
                SM_ERROR(nodeid, SM_ErrB);
        }
        /* Finalise last SDO transfer with this node */
        closeSDOtransfer(d, nodeid, SDO_CLIENT);

    // if we're not done YET proceed
    if (!SM_IS_FINISHED(nodeid)) {

        DS302_DEBUG("_sm_BootSlave_getDeviceType no issues, logic proceeding (%d)\n", nodeid);

        // we have the data in Index1000
        // compare against the expected value in 0x1F84
        // dummy check for now
        if (0) {
            SM_ERROR(nodeid, SM_ErrC);
        } else {
            // everything OK, see if we need to check IDs or not
            // dummy check for now
            if (1) {
                // switch to pulling the IDs
                SM_SWITCH_STATE(SM_BOOTSLAVE_GET_ID1,d,nodeid)
            } else {
                // skip pulling the IDs chain
                SM_SWITCH_STATE(SM_BOOTSLAVE_DECIDE_BC,d,nodeid)
            }
        }
    } // else finished and we're done here
}

void _sm_BootSlave_getIdentification_1(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_getIdentification_1\n");

    if (SM_INITIAL(nodeid)) {
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

    UNS32   size = sizeof(SM_DATA(nodeid,Index1018_1));
    UNS8    retcode = getReadResultNetworkDict (d, nodeid, &SM_DATA(nodeid,Index1018_1), &size, &SM_DATA(nodeid,error_code));

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS)
            // do nothing, outside of callback call
            return;

    if(retcode != SDO_FINISHED) {
            SM_ERROR(nodeid, SM_ErrD);
    }
    /* Finalise last SDO transfer with this node */
    closeSDOtransfer(d, nodeid, SDO_CLIENT);

    // if we're not done YET proceed
    if (!SM_IS_FINISHED(nodeid)) {
        // verify against required data in 0x1F85
        if (0) {
            // mismatch, stop process
            SM_ERROR(nodeid, SM_ErrD);
        } else {
            // go to the next one
            SM_SWITCH_STATE(SM_BOOTSLAVE_GET_ID2,d,nodeid)
        }
    } // else finished and we're done here
}

void _sm_BootSlave_getIdentification_2(CO_Data* d, UNS8 nodeid)
{
    if (SM_INITIAL(nodeid)) {      
        // code for the first run only  
        // read 0x1018 0x02                       
        // self callback      
        readNetworkDictCallbackAI (d, nodeid, 0x1018, 0x02, 0, _sm_BootSlave_getIdentification_2, 0);
        // do nothing else on the first run
        return;                   
    }

    // we end here on callback

    UNS32   size = sizeof(SM_DATA(nodeid,Index1018_2));
    UNS8    retcode = getReadResultNetworkDict (d, nodeid, &SM_DATA(nodeid,Index1018_2), &size, &SM_DATA(nodeid,error_code));

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS)
        // do nothing, outside of callback call
        return;

    if(retcode != SDO_FINISHED) {
        SM_ERROR(nodeid, SM_ErrM);
    }
    /* Finalise last SDO transfer with this node */
    closeSDOtransfer(d, nodeid, SDO_CLIENT);  

    // if we're not done YET proceed
    if (!SM_IS_FINISHED(nodeid)) {
        // verify against required data in 0x1F86
        if (0) {                  
            // mismatch, stop process
            SM_ERROR(nodeid, SM_ErrM);
        } else {                               
            // go to the next one               
            SM_SWITCH_STATE(SM_BOOTSLAVE_GET_ID3,d,nodeid)
        }
    } // else finished and we're done here
}

void _sm_BootSlave_getIdentification_3(CO_Data* d, UNS8 nodeid)
{
    if (SM_INITIAL(nodeid)) {      
        // code for the first run only  
        // read 0x1018 0x03                       
        // self callback      
        readNetworkDictCallbackAI (d, nodeid, 0x1018, 0x03, 0, _sm_BootSlave_getIdentification_3, 0);
        // do nothing else on the first run
        return;                   
    }

    // we end here on callback

    UNS32   size = sizeof(SM_DATA(nodeid,Index1018_3));
    UNS8    retcode = getReadResultNetworkDict (d, nodeid, &SM_DATA(nodeid,Index1018_3), &size, &SM_DATA(nodeid,error_code));

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS)
        // do nothing, outside of callback call
        return;

    if(retcode != SDO_FINISHED) {
        SM_ERROR(nodeid, SM_ErrN);
    }
    /* Finalise last SDO transfer with this node */
    closeSDOtransfer(d, nodeid, SDO_CLIENT);  

    // if we're not done YET proceed
    if (!SM_IS_FINISHED(nodeid)) {
        // verify against required data in 0x1F87
        if (0) {                  
            // mismatch, stop process
            SM_ERROR(nodeid, SM_ErrN);
        } else {                               
            // go to the next one               
            SM_SWITCH_STATE(SM_BOOTSLAVE_GET_ID4,d,nodeid)
        }
    } // else finished and we're done here
}

void _sm_BootSlave_getIdentification_4(CO_Data* d, UNS8 nodeid)
{
    if (SM_INITIAL(nodeid)) {      
        // code for the first run only  
        // read 0x1018 0x04                       
        // self callback      
        readNetworkDictCallbackAI (d, nodeid, 0x1018, 0x04, 0, _sm_BootSlave_getIdentification_4, 0);
        // do nothing else on the first run
        return;                   
    }

    // we end here on callback

    UNS32   size = sizeof(SM_DATA(nodeid,Index1018_4));
    UNS8    retcode = getReadResultNetworkDict (d, nodeid, &SM_DATA(nodeid,Index1018_4), &size, &SM_DATA(nodeid,error_code));

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS)
        // do nothing, outside of callback call
        return;

    if(retcode != SDO_FINISHED) {
        SM_ERROR(nodeid, SM_ErrO);
    }
    /* Finalise last SDO transfer with this node */
    closeSDOtransfer(d, nodeid, SDO_CLIENT);  

    // if we're not done YET proceed
    if (!SM_IS_FINISHED(nodeid)) {
        // verify against required data in 0x1F85
        if (0) {                  
            // mismatch, stop process
            SM_ERROR(nodeid, SM_ErrO);
        } else {                               
            // go to the next one               
            SM_SWITCH_STATE(SM_BOOTSLAVE_DECIDE_BC,d,nodeid)
        }
    } // else finished and we're done here
}

void _sm_BootSlave_decideBCPath(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_decideBCPath\n");
    if (SM_INITIAL(nodeid)) {      
        // code for the first run only
        // switch to path C, path B is not implemented yet
        SM_SWITCH_STATE(SM_BOOTSLAVE_DO_CONFVER_CHECK,d,nodeid)
    }	
}

void _sm_BootSlave_doConfigurationVersionChecks(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_doConfigurationVersionChecks\n");
    if (SM_INITIAL(nodeid)) {      
        // code for the first run only  
       
        // see if we have 0x1F26 and 0x1F27 defined in the config for the node
        // if so, get the device info, if not download config

        if (0) {
            // we don't have the values, go straight to download
            SM_SWITCH_STATE(SM_BOOTSLAVE_DOWNLOAD_CONFIG,d,nodeid)
        } else {
            // we have the values, get them and compare them
            SM_SWITCH_STATE(SM_BOOTSLAVE_VERIFY_CONFVER_1,d,nodeid)
        }
    }  
}

void _sm_BootSlave_verifyConfigurationVersion_1(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_verifyConfigurationVersion_1\n");
    if (SM_INITIAL(nodeid)) {      
        // code for the first run only  
        // read 0x1020 0x01           
        // self callback      
        readNetworkDictCallbackAI (d, nodeid, 0x1020, 0x01, 0, _sm_BootSlave_verifyConfigurationVersion_1, 0);
        // do nothing else on the first run
        return;                
    }  

    // we end here on callback

    UNS32   size = sizeof(SM_DATA(nodeid,Index1020_1));
    UNS8    retcode = getReadResultNetworkDict (d, nodeid, &SM_DATA(nodeid,Index1020_1), &size, &SM_DATA(nodeid,error_code));

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS)
        // do nothing, outside of callback call
        return;

    if(retcode != SDO_FINISHED) {
        // here we have an error, but it is NOT fatal!!!
        // SM_ERROR(nodeid, SM_ErrO);
        // set the data to -1 to indicate failure to retrieve
        SM_DATA(nodeid,Index1020_1) = -1;
    }
    /* Finalise last SDO transfer with this node */
    closeSDOtransfer(d, nodeid, SDO_CLIENT);  

    // if we're not done YET proceed
    if (!SM_IS_FINISHED(nodeid)) {
        // verify against required data in 0x1F26
        if (0) {                  
            // mismatch, go directly to download
            SM_SWITCH_STATE(SM_BOOTSLAVE_DOWNLOAD_CONFIG,d,nodeid)                        
        } else {                               
            // go to the next one               
            SM_SWITCH_STATE(SM_BOOTSLAVE_VERIFY_CONFVER_2,d,nodeid)
        }
    } // else finished and we're done here
}

void _sm_BootSlave_verifyConfigurationVersion_2(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_verifyConfigurationVersion_2\n");
    if (SM_INITIAL(nodeid)) {      
        // code for the first run only         
        // read 0x1020 0x02 
        // self callback      
        readNetworkDictCallbackAI (d, nodeid, 0x1020, 0x02, 0, _sm_BootSlave_verifyConfigurationVersion_2, 0);
        // do nothing else on the first run
        return;                                  
    }  

    // we end here on callback

    UNS32   size = sizeof(SM_DATA(nodeid,Index1020_2)); 
    UNS8    retcode = getReadResultNetworkDict (d, nodeid, &SM_DATA(nodeid,Index1020_2), &size, &SM_DATA(nodeid,error_code));

    if (retcode == SDO_UPLOAD_IN_PROGRESS || retcode == SDO_DOWNLOAD_IN_PROGRESS)
        // do nothing, outside of callback call
        return;

    if(retcode != SDO_FINISHED) {
        // here we have an error, but it is NOT fatal!!!
        // SM_ERROR(nodeid, SM_ErrO);     
        // set the data to -1 to indicate failure to retrieve
        SM_DATA(nodeid,Index1020_2) = -1;
    }
    /* Finalise last SDO transfer with this node */
    closeSDOtransfer(d, nodeid, SDO_CLIENT);  

    // if we're not done YET proceed
    if (!SM_IS_FINISHED(nodeid)) {
        // verify against required data in 0x1F27
        if (0) {                  
            // mismatch, go directly to download
            SM_SWITCH_STATE(SM_BOOTSLAVE_DOWNLOAD_CONFIG,d,nodeid)                        
        } else {                               
            // configuration is at the exepected levels, skip configuration download
            // go to start error control services   
            SM_SWITCH_STATE(SM_BOOTSLAVE_START_ERRCTL,d,nodeid)
        }
    } // else finished and we're done here
}

void _sm_BootSlave_downloadConfiguration(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_downloadConfiguration\n");
    //// this is dummy for now. Doesn't do anything other than keep the SM functioning correctly
    if (SM_INITIAL(nodeid)) {      
        // code for the first run only         
    }

    if (0) {
        // we had an error in the configuration download
        SM_ERROR(nodeid, SM_ErrJ);
    } else {
        // configuration downloaded OK, go to error control
        SM_SWITCH_STATE(SM_BOOTSLAVE_START_ERRCTL,d,nodeid)
    }
}

void _sm_BootSlave_startErrorControlService(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_startErrorControlService\n");
    if (SM_INITIAL(nodeid)) {      
        // code for the first run only

        // is the heartbeat consumer non-zero for this node
        if (0) {
            // non-zero consumer, use HB
            SM_SWITCH_STATE(SM_BOOTSLAVE_WAIT_HB,d,nodeid)
        } else {
            // zero consumer, use NodeGuard
            // is the node still on the Network list (0x1F81 bit 0)
            if (0) {
                // if yes, start node guard for it
                // verify node guard for it, 0x1F81 bits 2,3
                if (0) {
                    //node guard time non-zero
                    // start Node Guard for the node
                    SM_SWITCH_STATE(SM_BOOTSLAVE_START_NODEGUARD,d,nodeid)
                } else {
                    //node guard time zero
                    // go to error control started
                    SM_SWITCH_STATE(SM_BOOTSLAVE_ERRCTL_STARTED,d,nodeid)
                }
            } else {
                // if no, error control completed OK
                // go to error control started
                SM_SWITCH_STATE(SM_BOOTSLAVE_ERRCTL_STARTED,d,nodeid)
            }
        }
    }
// nothing happens for subsequent runs, should not end here
}

void _sm_BootSlave_waitHeartbeat(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_waitHeartbeat\n");
    // dummy for now
    if (SM_INITIAL(nodeid)) {
        SM_SWITCH_STATE(SM_BOOTSLAVE_ERRCTL_STARTED,d,nodeid)
    }
}

void _sm_BootSlave_startNodeGuard(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_startNodeGuard\n");
    // dummy for now
    if (SM_INITIAL(nodeid)) {
        SM_SWITCH_STATE(SM_BOOTSLAVE_ERRCTL_STARTED,d,nodeid)
    }
}

void _sm_BootSlave_errorControlStarted(CO_Data* d, UNS8 nodeid)  // here we need to check if we went D/noconfig and exit
{
    DS302_DEBUG("_sm_BootSlave_errorControlStarted\n");
    if (SM_INITIAL(nodeid)) {
        // simple decisional node, have we went via D path or not
        if (SM_DATA(nodeid,ViaDPath)) {
            SM_ERROR(nodeid, SM_ErrL);			
        } else {
            SM_SWITCH_STATE(SM_BOOTSLAVE_START_SLAVE,d,nodeid)
        }
    }
}

void _sm_BootSlave_startSlave(CO_Data* d, UNS8 nodeid)
{
    DS302_DEBUG("_sm_BootSlave_startSlave\n");
    if (SM_INITIAL(nodeid)) {
        
        // am I allowed to start the nodes? 0x1F80 bit 3
        if (0) {
            // yes, I am
            // do I have to start each one individually? 0x1F80 bit 1
            if (1) {
                // yes, I have
                // well, then start the node

                // ...
                masterSendNMTstateChange (d, nodeid, NMT_Start_Node);

                // we're done here
                SM_ERROR(nodeid, SM_OK);
            } else {
                // no I don't
                // is my state Operational?
                if (getState(d) == Operational) {
                    // yes it is
                    // start the node
                    
                    // ...
                    masterSendNMTstateChange (d, nodeid, NMT_Start_Node);
        
                    // we're done here
                    SM_ERROR(nodeid, SM_OK);
                } else {
                    // nope, we're done here
                    SM_ERROR(nodeid, SM_OK);
                }
            }
        } else {
            // nope
            // we're done here
            SM_ERROR(nodeid, SM_OK);
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
                if (ds302_boot_data[nodeid] != BootCompleted) {
                    DS302_DEBUG("Mandatory slave not booted\n");
                    eprintf ("Slave ID is %d, state is %d\n", nodeid, ds302_boot_data[nodeid]);
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

    if (INITIAL_SM(_masterBoot)) {

            /* initialize data structures */
            int     i;

            // initialize the boot state machines   
            for (i = 0; i < NMT_MAX_NODE_ID; i++)
                    SM_INIT (i);
        
            // initialize the boot signals
            for (i = 0; i < NMT_MAX_NODE_ID; i++)
                    ds302_boot_data[i] = BootInitialised;

            if (ds302_bitcheck_32(d, 0x1F80, 0x00, DS302_DEVICE_NMT_MASTER) != 1) {
                    // enter slave state
                    DS302_DEBUG("I am not a master, so not booting\n");
                    // not needed, stop the boot machine
            STOP_SM(_masterBoot);
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

    // go to the boot process
    SWITCH_SM(_masterBoot, MB_BOOTPROC, d, idx);
}

void _sm_BootMaster_bootproc (CO_Data* d, UNS32 idx)
{
    int	i;

    eprintf ("_sm_BootMaster_bootproc ENTRY (%d)\n", idx);

    // this is called multiple times until all mandatory slaves are done
    // there is no initial at this point, it's lower in the process

    // verify if we are done here
    if (ds302_all_mandatory_booted(d)) {
        // switch to the next step
        SWITCH_SM(_masterBoot, MB_OPERWAIT, d, idx);
        return;
    }

    // if we are not yet done, process the state machines
    if (INITIAL_SM(_masterBoot)) {

        // this is the first run, so start all the state machines for the slaves in the network list
        eprintf ("_sm_BootMaster_bootproc INITIAL\n");

        for (i=1; i<NMT_MAX_NODE_ID; i++)
            if (ds302_nl_node_in_list(d, i)) {
                // it's a slave, so start it's machine
                // mark the start time
                SM_DATA(i,start_time) = rtuClock();
                //EnterMutex();
                SM_RUN_MACHINE(d, i);
                //LeaveMutex();
            } else {
                // zero out the start time to indicate unused
                SM_DATA(i,start_time) = 0;
            }
        // at this point we're done here
    } 
        

    // additional calls
    eprintf ("_sm_BootMaster_bootproc N+1\n");

    // process the entire state machine list, until all are done
    // if all are done, re-do the mandatory check and switch, or RAISE HELL (all machines done but NO JOY on mandatory)
    int	alldone = 1;
    for (i=1; i<NMT_MAX_NODE_ID; i++) {
        if(!SM_IS_FINISHED(i)) {
            // machine still runs, or it's not used
            // check if used. Rely on the start time
            if (SM_DATA(i,start_time) == 0) {
                // unused machine, skip
                continue;
            } else {
                DS302_DEBUG("BM: %d is still running\n", i);
                // machine is in use and it is not finished yet
                // however, machines run independently at this point, so we just mark that NOT all done
                alldone = 0;
            }
        } else {

            DS302_DEBUG("BM: %d is finished\n", i);
            // SM finished
            // do the process
            _sm_BootSlave_Codes     result = SM_DATA(i,machine_state);

            //if not mandatory slave, signal boot attempted
            if (!ds302_nl_mandatory_node(d, i)) {
                ds302_boot_data[i] = BootAttempted;
            }


                    if (result == SM_ErrB) {
                uint64_t		elapsedTime = rtuClock() - SM_DATA(i,start_time);

                DS302_DEBUG ("Got status B for SM %d, elapsed time %d\n", i, elapsedTime);
                // if it's mandatory, check 
                            if (ds302_nl_mandatory_node(d, i))
                    if (elapsedTime > 10*1000000) {
                                        // timed out, inform application
                                        eprintf ("Can't start node %d and it's mandatory. Elapsed %d trying to start it\n", i, elapsedTime);
                                        DS302_DEBUG("Boot expired for mandatory %d\n", i);
                                } else {
                                        DS302_DEBUG("Rescheduling %d (I should wait 1 second here)\n", i);
                                        // sleep 1 second
                        // restart machine
                                        SM_INIT(i);
                        //EnterMutex();
                                        SM_RUN_MACHINE(d, i);
                                        //LeaveMutex();
                        
                        // we are not yet done
                        alldone = 0;
                                }

                    } else if (result != SM_OK) {
                            DS302_DEBUG("Result is NOT OK, BootTimedOut for %d <|==\n", i);
                            // inform application
                            eprintf ("Node start for %d returned %d\n", i, result);
                            // signal boot failed
                            ds302_boot_data[i] = BootTimedOut;
                    } else {
                            DS302_DEBUG("Result is OK, BootCompleted for %d <---\n", i);
                            // everything is fine, exit loop
                            // signal completed ok
                            ds302_boot_data[i] = BootCompleted;
                    }
            
        }
    }


    if (alldone) {
        // re-verify if we are done here
            if (ds302_all_mandatory_booted(d)) {
                    // switch to the next step
                    SWITCH_SM(_masterBoot, MB_OPERWAIT, d, idx);
            return;
            } else {

            DS302_DEBUG ("ALL DONE BUT NOT FINISHED!!!\n");
            STOP_SM(_masterBoot);
            return;
        }
    } else {

        DS302_DEBUG("BM: not all done, waiting 100ms\n");
        // set alarm for 100ms for this function

        DS302_DEBUG("_sm_BootMaster_bootproc ALARM SET to %d\n", ++idx);
        SetAlarm (d, idx, _sm_BootMaster_bootproc, MS_TO_TIMEVAL(100), 0);
    }
}

void _sm_BootMaster_operwait (CO_Data* d, UNS32 idx)
{
    DS302_DEBUG ("_sm_BootMaster_operwait ENTRY (%d)\n", idx);

    if (INITIAL_SM(_masterBoot)) {
        if (!ds302_all_mandatory_booted(d)) {
            eprintf ("We should not be here, but here we are! Not ALL mandatory booted OK\n");
            STOP_SM(_masterBoot);
            return;
        }

        // shall I go Operational automagically?
        if (ds302_bitcheck_32(d, 0x1F80, 0x00, DS302_DEVICE_MANUAL_OPERATIONAL) == 0) {
            // automatic go to operational

            DS302_DEBUG("Entering operational\n");
            setState(d, Operational);

            // switch to start slaves
            SWITCH_SM(_masterBoot, MB_SLAVESTART, d, idx);
            return;
        }
    }

    DS302_DEBUG ("_sm_BootMaster_operwait N+1 (timer based I hope)\n");

    // verify if we're operational yet
    if (getState (d) == Operational) {
        DS302_DEBUG("Was put externally into operational, switching to MB_SLAVESTART\n");
        
        // switch to start slaves
        SWITCH_SM(_masterBoot, MB_SLAVESTART, d, idx);
        return;
    }

    DS302_DEBUG("_sm_BootMaster_bootproc ALARM SET to %d\n", ++idx);
    SetAlarm (d, idx, _sm_BootMaster_bootproc, MS_TO_TIMEVAL(100), 0);
}

void _sm_BootMaster_slavestart (CO_Data* d, UNS32 idx)
{
    DS302_DEBUG ("_sm_BootMaster_slavestart ENTRY (%d)\n", idx);

        if (INITIAL_SM(_masterBoot)) {

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

    DS302_DEBUG("MasterBoot - STOP STATE MACHINE (%d)\n", idx);
    // should not end up here, but just in case
    STOP_SM(_masterBoot);	
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