/*

epos.c

Main routines for the EPOS drives
*/

#include "EPOScontrol.h"
#include "epos.h"
#include "dcf.h"

#define EPOS_PDO_MAX     4

#define eprintf(...) rt_printf(__VA_ARGS__)

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

// master EPOS drive structure
EPOS_drive_t        EPOS_drive;

static UNS32 _statusWordCB (CO_Data * d, const indextable *idx, UNS8 bSubindex);

/*
 * Name         : epos_add_slave
 *
 * Synopsis     : int    epos_add_slave (UNS8 slaveid)
 *
 * Arguments    : UNS8  slaveid : slave id to add
 *
 * Description  : Adds a new slave to the drive object
 * 
 * Returns      : int    0 if error, 1 if success
 */
int    epos_add_slave (UNS8 slaveid) {
    
   if (slaveid == getNodeId(EPOS_drive.d) || slaveid < 1)
        return 0;
    
    if (EPOS_drive.epos_slave_count >= MAX_EPOS_DRIVES)
        return 0;
    
    // add the node to the list
    EPOS_drive.epos_slaves[EPOS_drive.epos_slave_count] = slaveid;
    
    // setup the SDOs
    if (!epos_setup_sdo (slaveid, EPOS_drive.epos_slave_count))
        return 0;
    
    // setup the PDO block for the node
    if (!epos_setup_rx_pdo (slaveid, EPOS_drive.epos_slave_count))
        return 0;
    if (!epos_setup_tx_pdo (slaveid, EPOS_drive.epos_slave_count))
        return 0;
    
    // add the DCF data to the node
    UNS32               errorCode;
    const indextable    *Object1F22;
    
    Object1F22 = (*EPOS_drive.d->scanIndexOD)(EPOS_drive.d, 0x1F22, &errorCode);
    if (errorCode != OD_SUCCESSFUL)
        return 0;
    
    if (slaveid >= Object1F22->bSubCount)
        return 0;

    dcfstream_t *nodedcf;
    if (!get_dcf_node (&EPOS_drive.dcf_data, slaveid, &nodedcf))
        return 0;
    
    Object1F22->pSubindex[slaveid].pObject = nodedcf->dcf;
    Object1F22->pSubindex[slaveid].size = nodedcf->size;
    
    // add the slave to the Network List (1F81)???
    // add the slave to the heartbeat???
    // setup the DCF PDO mappings???
    
    // node was setup
    EPOS_drive.epos_slave_count++;
    
    return 1;
}

/*
 * Name         : epos_setup_sdo
 *
 * Synopsis     : int     epos_setup_sdo (UNS8 slaveid, int idx)
 *
 * Arguments    : UNS8  slaveid : Slave ID
 *                int  idx : index of the slave in the slave table (for determining the location)
 *
 * Description  : sets up the SDOs for the slave
 * 
 * Returns      : int     
 */

int     epos_setup_sdo (UNS8 slaveid, int idx) {

    UNS32   result;
    UNS32   size;
    UNS32   COB_ID;
    // setup the client SDO for the node
    
    // transmit SDO
    COB_ID = 0x600 + slaveid;
    size = sizeof(COB_ID);
    result = writeLocalDict (EPOS_drive.d,
        0x1280 + idx, 0x01, &COB_ID, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;

    // receive SDO
    COB_ID = 0x580 + slaveid;
    result = writeLocalDict (EPOS_drive.d,
        0x1280 + idx, 0x02, &COB_ID, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;
    
    // node ID
    size = sizeof(slaveid);
    result = writeLocalDict (EPOS_drive.d,
        0x1280 + idx, 0x03, &slaveid, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;
    
    return 1;
}

/*
 * Name         : epos_setup_rx_pdo
 *
 * Synopsis     : int     epos_setup_rx_pdo (UNS8 slaveid, int idx)
 *
 * Arguments    : UNS8  slaveid : slave ID
 *                int  idx : index of the slave in the slave table (for determining the location)
 *
 * Description  : sets up the RX PDOs for the slave, and disables them
 * 
 * Returns      : int     
 */
int     epos_setup_rx_pdo (UNS8 slaveid, int idx) {
    
    int     pdonr;

    UNS32   cobs[EPOS_PDO_MAX] = {0x180, 0x280, 0x380, 0x480};
    
    UNS32   result;
    UNS32   COB_ID;
    UNS32   size;
    UNS8    trans_type = 0xFF;
    UNS8    map_count = 0x00;
    
    for (pdonr = 0; pdonr < EPOS_PDO_MAX; pdonr++) {

        // the PDO params
        COB_ID = 0x80000000 + cobs[pdonr] + slaveid;
        size = sizeof(COB_ID);
        result = writeLocalDict (EPOS_drive.d,
            0x1400 + pdonr + (idx * EPOS_PDO_MAX), 0x01, &COB_ID, &size, 0);
        if (result != OD_SUCCESSFUL)
            return 0;

        size = sizeof(trans_type);
        result = writeLocalDict (EPOS_drive.d,
            0x1400 + pdonr + (idx * EPOS_PDO_MAX), 0x02, &trans_type, &size, 0);
        if (result != OD_SUCCESSFUL)
            return 0;
        
        // setup the PDO mapping
        size = sizeof(map_count);
        result = writeLocalDict (EPOS_drive.d,
            0x1600 + pdonr + (idx * EPOS_PDO_MAX), 0x00, &map_count, &size, 0);
        if (result != OD_SUCCESSFUL)
            return 0;        
    }

// setup mapping for the client

// Rx PDO 1:
// StatusWord (2B) 0x5041 / idx + 1
// ModesOfOperationDisplay (1B) 0x5061 / idx + 1
// DigitalInput (2B) 0x4071 / idx + 1
    
    UNS32   PDO_map;
    
    size = sizeof (PDO_map);
    
    PDO_map = 0x5041 << 16 | (idx + 1) << 8 | 0x10; // IDX / SubIDX / Len (bits)
    result = writeLocalDict (EPOS_drive.d,
        0x1600 + 0x00 + (idx * EPOS_PDO_MAX), 0x01, &PDO_map, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;        

    PDO_map = 0x5061 << 16 | (idx + 1) << 8 | 0x08; // IDX / SubIDX / Len (bits)
    result = writeLocalDict (EPOS_drive.d,
        0x1600 + 0x00 + (idx * EPOS_PDO_MAX), 0x02, &PDO_map, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;

    PDO_map = 0x4071 << 16 | (idx + 1) << 8 | 0x10; // IDX / SubIDX / Len (bits)
    result = writeLocalDict (EPOS_drive.d,
        0x1600 + 0x00 + (idx * EPOS_PDO_MAX), 0x03, &PDO_map, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;

    map_count = 0x03;
    size = sizeof(map_count);

    result = writeLocalDict (EPOS_drive.d,
        0x1600 + 0x00 + (idx * EPOS_PDO_MAX), 0x00, &map_count, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;        

// Rx PDO 2: (mode dependant)
// Position Actual Value (4B) 0x5064 / idx + 1
// Velocity Actual Value (4B) 0x506C / idx + 1
    
    size = sizeof (PDO_map);
    
    PDO_map = 0x5064 << 16 | (idx + 1) << 8 | 0x20; // IDX / SubIDX / Len (bits)
    result = writeLocalDict (EPOS_drive.d,
        0x1600 + 0x01 + (idx * EPOS_PDO_MAX), 0x01, &PDO_map, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;        

    PDO_map = 0x506C << 16 | (idx + 1) << 8 | 0x08; // IDX / SubIDX / Len (bits)
    result = writeLocalDict (EPOS_drive.d,
        0x1600 + 0x01 + (idx * EPOS_PDO_MAX), 0x02, &PDO_map, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;

    map_count = 0x02;
    size = sizeof(map_count);

    result = writeLocalDict (EPOS_drive.d,
        0x1600 + 0x01 + (idx * EPOS_PDO_MAX), 0x00, &map_count, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;        
    
    return 1;
}


/*
 * Name         : epos_setup_tx_pdo
 *
 * Synopsis     : int     epos_setup_tx_pdo (UNS8 slaveid, int idx)
 *
 * Arguments    : UNS8  slaveid : slave ID
 *                int  idx : index of the slave in the slave table (for determining the location)
 *
 * Description  : sets up the TX PDOs for the slave, and disables them
 * 
 * Returns      : int     
 */

int     epos_setup_tx_pdo (UNS8 slaveid, int idx) {
    
    int     pdonr;

    UNS32   cobs[EPOS_PDO_MAX] = {0x200, 0x300, 0x400, 0x500};
    
    UNS32   result;
    UNS32   COB_ID;
    UNS32   size;
    UNS8    trans_type = 0xFF;
    UNS8    map_count = 0x00;
    UNS16   inhibit_time = 0; //10; //(it's in 100us, 10 = 1ms)
    
    for (pdonr = 0; pdonr < EPOS_PDO_MAX; pdonr++) {

        // the PDO params
        if (pdonr < 2)
            COB_ID = 0x00000000 + cobs[pdonr] + slaveid;
        else
            COB_ID = 0x80000000 + cobs[pdonr] + slaveid;
        size = sizeof(COB_ID);
        result = writeLocalDict (EPOS_drive.d,
            0x1800 + pdonr + (idx * EPOS_PDO_MAX), 0x01, &COB_ID, &size, 0);
        if (result != OD_SUCCESSFUL)
            return 0;

        size = sizeof(trans_type);
        result = writeLocalDict (EPOS_drive.d,
            0x1800 + pdonr + (idx * EPOS_PDO_MAX), 0x02, &trans_type, &size, 0);
        if (result != OD_SUCCESSFUL)
            return 0;
        
        size = sizeof(inhibit_time);
        result = writeLocalDict (EPOS_drive.d,
            0x1800 + pdonr + (idx * EPOS_PDO_MAX), 0x03, &inhibit_time, &size, 0);
        if (result != OD_SUCCESSFUL)
            return 0;
                
        // setup the PDO mapping
        size = sizeof(map_count);
        result = writeLocalDict (EPOS_drive.d,
            0x1A00 + pdonr + (idx * EPOS_PDO_MAX), 0x00, &map_count, &size, 0);
        if (result != OD_SUCCESSFUL)
            return 0;        
    }
    
// setup mapping for the client

// Tx PDO 1:
// ControlWord (2B) 0x5040 / idx + 1
// ModesOfOperation (1B) 0x5060 / idx + 1
// DigitalOutput (2B) 0x4078 / idx + 1

    UNS32   PDO_map;
    
    size = sizeof (PDO_map);

    PDO_map = 0x5040 << 16 | (idx + 1) << 8 | 0x10; // IDX / SubIDX / Len (bits)
    result = writeLocalDict (EPOS_drive.d,
        0x1A00 + 0x00 + (idx * EPOS_PDO_MAX), 0x01, &PDO_map, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;        
    
    PDO_map = 0x5060 << 16 | (idx + 1) << 8 | 0x08; // IDX / SubIDX / Len (bits)
    result = writeLocalDict (EPOS_drive.d,
        0x1A00 + 0x00 + (idx * EPOS_PDO_MAX), 0x02, &PDO_map, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;        

    PDO_map = 0x4078 << 16 | (idx + 1) << 8 | 0x10; // IDX / SubIDX / Len (bits)
    result = writeLocalDict (EPOS_drive.d,
        0x1A00 + 0x00 + (idx * EPOS_PDO_MAX), 0x03, &PDO_map, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;        

    map_count = 0x03;
    size = sizeof(map_count);
    result = writeLocalDict (EPOS_drive.d,
        0x1A00 + 0x00 + (idx * EPOS_PDO_MAX), 0x00, &map_count, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;        

// Tx PDO 2: (mode dependant)
// PPM: ControlWord + Position Demand
// ControlWord (2B) 0x5040 / idx + 1
// Position Demand Value (4B) 0x4062 / idx + 1
// VPM: ControlWord + Velocity Demand

    size = sizeof (PDO_map);

    PDO_map = 0x5040 << 16 | (idx + 1) << 8 | 0x10; // IDX / SubIDX / Len (bits)
    result = writeLocalDict (EPOS_drive.d,
        0x1A00 + 0x01 + (idx * EPOS_PDO_MAX), 0x01, &PDO_map, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;        

    PDO_map = 0x4062 << 16 | (idx + 1) << 8 | 0x20; // IDX / SubIDX / Len (bits)
    result = writeLocalDict (EPOS_drive.d,
        0x1A00 + 0x01 + (idx * EPOS_PDO_MAX), 0x02, &PDO_map, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;        

    map_count = 0x02;
    size = sizeof(map_count);
    result = writeLocalDict (EPOS_drive.d,
        0x1A00 + 0x01 + (idx * EPOS_PDO_MAX), 0x00, &map_count, &size, 0);
    if (result != OD_SUCCESSFUL)
        return 0;        
    
    return 1;
}

/*
 * Name         : epos_setup_master
 *
 * Synopsis     : int     epos_setup_master ()
 *
 * Description  : set ups the master
 * 
 * Returns      : int     
 */

int     epos_initialize_master (CO_Data * d, const char * dcf_file) {
    
    int idx;
    
    EPOS_drive.epos_slave_count = 0;
    EPOS_drive.d = d;
    
    clear_dcf_set (&EPOS_drive.dcf_data);
    
    for (idx = 0; idx < MAX_EPOS_DRIVES; idx++) {
        // clean the slaves
        EPOS_drive.epos_slaves[idx] = 0x00;
        
        // clean the error data
        EPOS_drive.slave_err[idx][0] = 0x00;

        // set the callbacks
        
        // callback for drive status word
        RegisterSetODentryCallBack (EPOS_drive.d, 0x5041, 0x01 + idx, _statusWordCB);
    }
    
    load_dcf_set (&EPOS_drive.dcf_data, dcf_file);
    
    /* stupid fix for the PDO not being disabled by the objdictedit generator */
    
    UNS32   COB_ID = 0x80000000;
    UNS32   size = sizeof (COB_ID);

    for (idx = 0; idx < 20; idx++) {
        writeLocalDict (EPOS_drive.d,
            0x1400 + idx, 0x01, &COB_ID, &size, 0);
        
        writeLocalDict (EPOS_drive.d,
            0x1800 + idx, 0x01, &COB_ID, &size, 0);
    }

    return 1;
}

/*
 * Name         : epos_get_slave_index
 *
 * Synopsis     : int     epos_get_slave_index (UNS8 slaveid)
 *
 * Arguments    : UNS8  slaveid : Slave ID
 *
 * Description  : returns the slave index for the provided ID
 * 
 * Returns      : int     
 */

int     epos_get_slave_index (UNS8 slaveid) {
    
    int idx;
    for (idx = 0; idx < EPOS_drive.epos_slave_count; idx++)
        if (EPOS_drive.epos_slaves[idx] == slaveid)
            return idx;
        
    return -1;
}



/*
 * the status word callback does the state machines for the DS402
 *
 *
 *
 *
 */

static int debug = 1;
static UNS32 _statusWordCB (CO_Data * d, const indextable *idxtbl, UNS8 bSubindex) {
    
    // idx is the OD entry, bSubindex is the array item in it (eq. drive idx + 1)
    int     idx = bSubindex - 1;

    // update the state for the corresponding drive based on the status word
    EPOS_drive.EPOS_State[idx] = (*(UNS16 *)(idxtbl->pSubindex[bSubindex].pObject)) & 0x417F;
    
    /*
        Possible external commands:
        - Shutdown (2, 6, 8) : all shutdown targets go to RSO
        - Switch On (3) 
        - Disable operation (5) / Enable operation (4,16) - power crossing (enabled/disabled)
        - QuickStop (7 in PD, 10 in PD, 11 in PE)
        - Disable Voltage (7 in PD, 9 in PE, 10 in PD, 12 in PE)
        - Fault Reset (15)
        
        Everything is controlled by 4 bits in Controlword + fault reset (15)
        - bit 0 : Switch On / Switch Off
        - bit 1 : Enable Voltage / Disable Voltage
        - bit 2 : Quick Stop
        - bit 3 : Enable Operation / Disable operation
        - bit 7 : Fault reset
        
        In short:
        - drive is ENABLED or DISABLED (should be edge triggered, not level triggered)
            It means we'll have a EnableDrive and DisableDrive external function without a stable signal
            EnableDrive will work from SOD only via 2->3->4
            DisableDrive will work from OPEN/SD only via 9/12
        - QuickStop is ENABLED or DISABLED (can be level triggered)
            a QuickStop pin will control 11/16 transitions
        - if fault active, try to reset it and go to disabled? (edge triggered)
            It means we'll have a FaultReset function that will work in the FAULT state only
    */
    
    /***** NOTE: callback from PDO, NO MUTEXES! ****/
    switch (EPOS_drive.EPOS_State[idx]) {
        case EPOS_START:
            // state: bootup
            // possible transitions:
            // 0 -> NOTREADY (AUTO)
            if (debug) eprintf("Start\n");
            break;
        case EPOS_NOTREADY:
            // state: measure current offset
            // state: drive function disabled
            // possible transitions (entry from 19, Node Reset)
            // 1 -> SOD (switch on disabled) (AUTO)
            if (debug) eprintf("Not Ready to Switch On\n");
            break;
        case EPOS_SOD:
            // state: drive init complete
            // state: drive function disabled
            // state: DRIVE PARAMS CAN BE CHANGED
            // THIS IS THE POWER DISABLED DEFAULT STATE
            // possible transitions
            // 2 -> RSO (ready to switch on) (manual) (user, function)
            if (debug) eprintf("Switch On Disabled\n");
            /* should be done if we REQUESTED to turn on */
            //EnterMutex();
        
            // clear the Fault Reset bit here, fault was reset
            CLEAR_BIT (ControlWord[idx], 7);
            
            /*SET_BIT(ControlWord[idx], 2);
            SET_BIT(ControlWord[idx], 1);
            CLEAR_BIT(ControlWord[idx], 0);*/
            
            //LeaveMutex();
            break;
        case EPOS_RSO:
            // state: drive function disabled
            // state: drive params can be changed
            // this is a transitory state
            // possible transitions
            // 3 -> SWO (switched on) (automatic) 
            // 7 -> SOD (switch on disabled) (manual, not needed)
            if (debug) eprintf("Ready to Switch On\n");
            /* 3 if requested to turn on, 7 if requested to shut down */
            // this is #3
            //EnterMutex();
            SET_BIT(ControlWord[idx], 2);
            SET_BIT(ControlWord[idx], 1);
            SET_BIT(ControlWord[idx], 0);
            //LeaveMutex();
            break;
        case EPOS_SWO:
            // state: drive function disabled
            // this is a transitory state
            // possible transitions
            // 4 -> REFRESH (automatic)
            // 6 -> RSO (manual, not needed)
            // 10 ->SOD (manual, not needed)
            if (debug) eprintf("Switched on\n");
            /* 4 if requested to turn on, 6 or 10 if requested to shut down */
            // this is #4
            //EnterMutex();
            SET_BIT(ControlWord[idx], 3);
            SET_BIT(ControlWord[idx], 2);
            SET_BIT(ControlWord[idx], 1);
            SET_BIT(ControlWord[idx], 0);
            //LeaveMutex();
            break;
        case EPOS_REFRESH:
            // state: refresh power stage
            // possible transitions
            // 20 -> MEASURE (AUTO)
            if (debug) eprintf("Refresh\n");
            // transition 20 to measure
            // this should be automatic
            break;
        case EPOS_MEASURE:
            // state: power applied, electrical measurements
            // possible transitions
            // 21 -> OPEN (AUTO)
            if (debug) eprintf("Measure Init\n");
            // transition 21 to operation enable
            // this should be automatic
            break;
        case EPOS_OPEN:
            // state: no faults detected, power applied
            // THIS IS THE DEFAULT POWER ENABLED STATE
            // possible transitions
            // 5 -> SWO (manual, not needed)
            // 8 -> RSO (manual, not needed)
            // 9 -> SOD (manual, via function)
            // 11 -> QUICKS (manual, via QuickStop pin)
            if (debug) eprintf("Operation enable\n");
            // transition 5 to switched on
            // transition 8 to readdy to switch on
            // transition 9 to switch on disabled
            // transition 11 to quick stop active
            
            /* implement QuickStop pin handling */
            
            break;
        case EPOS_QUICKS:
            // state: QuickStop is active, power applied
            // this is the QuickStop state. Flip the QS bit to get back into operation
            // possible transitions
            // 16 -> OPEN (manual, via QuickStop pin)
            // 12 -> SOD (manual, via function)
            if (debug) eprintf("Quick Stop Active\n");
            // transition 16 to operation enable
            // transition 12 to switch on disabled
            
            /* implement QuickStop pin handling */
            
            break;
        case EPOS_FRAD:
            // state: fault detected, drive function disabled
            // possible transitions (entry from 13, fault during power disabled)
            // 14 -> FAULT (AUTO)
            if (debug) eprintf("Fault Reaction Active (disabled)\n");
            break;
        case EPOS_FRAE:
            // state: fault detected, quick stop is being executed, drive function enabled and power active
            // after FRAE completes, transition to fault automatically via 18
            // possible transitions (entry from 17, fault during power enabled)
            // 18 -> FAULT (AUTO)
            if (debug) eprintf("Fault Reaction Active (enabled)\n");
            break;
        case EPOS_FAULT:
            // possible transitions
            // 15 -> SOD (manual, via function)
            if (debug) eprintf("Fault\n");
            break;
        default:
            eprintf("Bored to input codes. Unknown code %04x\n", EPOS_drive.EPOS_State[idx]);
    }

    // Do the PPM state machine
    // first, update the current state
    update_PPM (idx);
    
    // now, see what transition is needed. The only transition at this point is from ACK to RUN

    if (EPOS_drive.EPOS_PPMState[idx] == PPM_Acknowledged) {
        
        // transition to Running by clearing the ControlWord bit
        CLEAR_BIT(ControlWord[idx], 4);
    }
        
    // send the updates (observing the mapping)
    sendPDOevent(d);
    //sendOnePDOevent(EPOS_drive.d, 0 + (idx * EPOS_PDO_MAX));

    return OD_SUCCESSFUL;
}

void    update_PPM (int idx) {
    
    // update the current state
    if (BIT_IS_SET(ControlWord[idx],4))
        if (BIT_IS_SET(StatusWord[idx], 12))
            EPOS_drive.EPOS_PPMState[idx] = PPM_Acknowledged;
        else
            EPOS_drive.EPOS_PPMState[idx] = PPM_Sent;
    else if (BIT_IS_SET(StatusWord[idx], 12))
            EPOS_drive.EPOS_PPMState[idx] = PPM_Running;
        else
            EPOS_drive.EPOS_PPMState[idx] = PPM_Ready;    

#ifdef __DEBUG__        
    switch (EPOS_drive.EPOS_PPMState[idx]) {
        case PPM_Acknowledged:
            eprintf ("PPM acknowledged\n");
            break;
        case PPM_Sent:
            eprintf ("PPM sent\n");
            break;
        case PPM_Running:
            eprintf ("PPM running\n");
            break;
        case PPM_Ready:
            eprintf ("PPM ready\n");
            break;
    }
#endif
}

/*
    Verifies if the PPM is ready to do a move
*/
int     epos_can_do_PPM (int idx) {
    
    // update the PPM state
    update_PPM (idx);
    
    // ensure we're in the proper PPM and also the drive is OPERATIONAL
    return EPOS_drive.EPOS_PPMState[idx] == PPM_Ready && epos_drive_operational(idx);
}

int     epos_do_move_PPM (int idx, INTEGER32 position) {
    
    if (epos_can_do_PPM(idx)) {
        // load the position into 0x4062[idx+1]
        PositionDemandValue[idx] = position;
        // set the bit for the control word
        SET_BIT(ControlWord[idx],4);
        // send the PDO for the MOVE (observing the mapping)
        sendPDOevent(EPOS_drive.d);
        //sendOnePDOevent(EPOS_drive.d, 1 + (idx * EPOS_PDO_MAX));
        return 1;
    }
    
    return 0;
}

/*
*/
int     epos_in_position (int idx) {
    
    return BIT_IS_SET (StatusWord[idx], 10);
}

void    epos_set_absolute (int idx) {
    
    CLEAR_BIT(ControlWord[idx], 6);
}

void    epos_set_relative (int idx) {
    
    SET_BIT(ControlWord[idx], 6);
}

void    epos_set_continuous (int idx) {
    
    SET_BIT(ControlWord[idx], 5);
}

void    epos_set_segmented (int idx) {
    
    CLEAR_BIT(ControlWord[idx], 5);
}

void    epos_halt (int idx) {
    
    SET_BIT(ControlWord[idx], 8);
}

void    epos_execute (int idx) {
    
    CLEAR_BIT(ControlWord[idx], 8);
}


void    epos_enable_drive (int idx) {
    
    /* enables the drive function. Drive must be in SOD */
    
    if (EPOS_drive.EPOS_State[idx] == EPOS_SOD) {
        
        /* do transition 2 */
        SET_BIT (ControlWord[idx], 2);
        SET_BIT (ControlWord[idx], 1);
        CLEAR_BIT (ControlWord[idx], 0);
    }
}

void    epos_disable_drive (int idx) {
    
    /* disables the drive function. Drive must be either OPEN or QUICKS */

    if (EPOS_drive.EPOS_State[idx] == EPOS_OPEN || EPOS_drive.EPOS_State[idx] == EPOS_QUICKS) {
        
        /* do transition 9/12 via Voltage Disable */
        CLEAR_BIT (ControlWord[idx], 1);
    }    
}

void    epos_fault_reset (int idx) {
    
    if (EPOS_drive.EPOS_State[idx] == EPOS_FAULT) {
        
        /* do transition 15 via Fault Reset */
        SET_BIT (ControlWord[idx], 7);
    }
}

int     epos_drive_operational (int idx) {
    
    if (EPOS_drive.EPOS_State[idx] == EPOS_OPEN || EPOS_drive.EPOS_State[idx] == EPOS_QUICKS)
        return 1;
    
    return 0;
}

int     epos_drive_faulted (int idx) {
    
    if (EPOS_drive.EPOS_State[idx] == EPOS_FAULT)
        return 1;
    
    return 0;
}

int     epos_drive_disabled (int idx) {
    
    if (EPOS_drive.EPOS_State[idx] != EPOS_OPEN && EPOS_drive.EPOS_State[idx] != EPOS_QUICKS && EPOS_drive.EPOS_State[idx] != EPOS_FAULT)
        return 1;
    
    return 0;
}
