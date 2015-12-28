

/*
This file is part of CanFestival, a library implementing CanOpen Stack. 

Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#if defined(WIN32) && !defined(__CYGWIN__)
#include <windows.h>
#include "getopt.h"
void pause(void)
{
    system("PAUSE");
}
#else
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#endif

#include "canfestival.h"
#include "master.h"
#include "gendcf.h"
#include "EPOScontrol.h"
#include "data.h"

#include "ds302.h"

//#include "epos_api.h"

unsigned int slavenodeid = 0x01;
unsigned int bootup = 1;

unsigned int debug = 0;

UNS8 dcfdatas[NMT_MAX_NODE_ID][DCF_MAX_SIZE];

#define SET_BIT(val, bitIndex) val |= (1 << bitIndex)
#define CLEAR_BIT(val, bitIndex) val &= ~(1 << bitIndex)
#define TOGGLE_BIT(val, bitIndex) val ^= (1 << bitIndex)
#define BIT_IS_SET(val, bitIndex) (val & (1 << bitIndex))



//#define USE_HEARTBEAT
#define HB_INTERVAL_MS		1000
//#define USE_SYNC
#define SYNC_INTERVAL_MS 	10
#define MAN_PDO


#define PROFILE_VELOCITY	1500
#define PROFILE_ACCELERATION	50000

static UNS16	EPOS_State;

typedef struct {
	// EPOS_State is the slave state
	UNS16	EPOS_State;

	// enabled or disabled
	char	drive_enabled;
	// initialised or not
	char	drive_initialised;
	// state of the drive
	char	drive_state;

	// CNC interface
	char	enable;		// true - drive enabled, false - drive disabled
	int	control_type;	// 0 - position control, 1 - velocity control
	int	maxvel;		// maximal drive velocity
	int	maxaccel;	// maximal drive acceleration

	int	counts;		// raw encoder / position from the drive
	float	position_scale;	// scale for positioning. position = counts / position-scale

	float	position_cmd;	// position command (for position control)
	float	velocity_cmd;	// velocity command (for velocity control)

	float	position_fb;	// position feedback
	float	velocity_fb;	// velocity feedback

	char	fault;		// drive is faulted

	// extra stuff

	char	home;		// set to high to initialise homing
	char	homing_done;	// set to high when drive is homed

	char	lock;		// engage the lock brake
	char	is_locked;	// lock brake is engaged and axis is confirmed locked

	// GPIO pins
	char	home_sw_pin;	// home switch pin
	char	enable_pin;	// enable pin
	char	in_a_pin;	// IN A
	char	in_b_pin;	// IN B
	char	in_c_pin;	// IN C
	char	in_d_pin;	// IN D
	char	in_e_pin;	// IN E
	char	in_f_pin;	// IN F

	float	in_ana1;	// Analog 1 in
	float	in_ana2;	// Analog 2 in

	char	out_a_pin;	// OUT A
	char	out_b_pin;	// OUT B
	char	out_c_pin;	// OUT C
	char	out_d_pin;	// OUT D
	
} EPOS_drive;


int sleep_ms ( unsigned long ms )
{
        if ( ms >= 1000 ) ms = 1000;
        return usleep ( ms * 1000 );
}

/*****************************************************************************/
void TestMaster_heartbeatError(CO_Data* d, UNS8 heartbeatID)
{
	eprintf("TestMaster_heartbeatError %d\n", heartbeatID);
}

void setup_local_params(UNS8);

/********************************************************
 * ConfigureSlaveNode is responsible to
 *  - setup master RPDO 1 to receive TPDO 1 from id 0x40
 *  - setup master TPDO 1 to send RPDO 1 to id 0x40
 ********************************************************/
void TestMaster_initialisation(CO_Data* d)
{
	setup_local_params (slavenodeid);
}

static int init_step = 0;

/*Froward declaration*/
static void ConfigureSlaveNode(CO_Data* d, UNS8 nodeId);

/**/
static void CheckSDOAndContinue(CO_Data* d, UNS8 nodeId)
{
	UNS32 abortCode;

	if(getWriteResultNetworkDict (d, nodeId, &abortCode) != SDO_FINISHED)
		eprintf("Master : Failed in initializing slave %2.2x, step %d, AbortCode :%4.4x \n", nodeId, init_step, abortCode);

	/* Finalise last SDO transfer with this node */
	closeSDOtransfer(&EPOScontrol_Data, nodeId, SDO_CLIENT);

	ConfigureSlaveNode(d, nodeId);
}




void WiteSDOValue (CO_Data* d, UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS32 count, UNS8 dataType, void *data)
{
        UNS8 res = writeNetworkDict (d, /*CO_Data* d*/
        	nodeId, /*UNS8 nodeId*/
                index, /*UNS16 index*/
                subIndex, /*UNS8 subindex*/
                count, /*UNS8 count*/
                0, /*UNS8 dataType*/
                data,/*void *data*/
        0); /* use block mode */
}


/********************************************************
 * ConfigureSlaveNode is responsible to
 *  - setup slave TPDO 1 transmit time
 *  - setup slave TPDO 2 transmit time
 *  - setup slave Heartbeat Producer time
 *  - switch to operational mode
 *  - send NMT to slave
 ********************************************************
 * This an example of :
 * Network Dictionary Access (SDO) with Callback 
 * Slave node state change request (NMT) 
 ********************************************************
 * This is called first by TestMaster_preOperational
 * then it called again each time a SDO exchange is
 * finished.
 ********************************************************/
static void ConfigureSlaveNode(CO_Data* d, UNS8 nodeId)
{
	UNS8 res;
	eprintf("Master : ConfigureSlaveNode %2.2x\n", nodeId);
	printf("nodeid slave=%x\n",nodeId);
        printf("init step=%d\n", init_step);
	switch(++init_step){
		case 1:	
		{
#ifdef USE_HEARTBEAT			
			UNS16 Heartbeat_Producer_Time = HB_INTERVAL_MS; 
			eprintf("Master : set slave %2.2x heartbeat producer time to %d\n", nodeId, Heartbeat_Producer_Time);
			res = writeNetworkDictCallBack (d, /*CO_Data* d*/
					/**TestSlave_Data.bDeviceNodeId, UNS8 nodeId*/
					nodeId, /*UNS8 nodeId*/
					0x1017, /*UNS16 index*/
					0x00, /*UNS8 subindex*/
					2, /*UNS8 count*/
					0, /*UNS8 dataType*/
					&Heartbeat_Producer_Time,/*void *data*/
					CheckSDOAndContinue, /*SDOCallback_t Callback*/
                    	0); /* use block mode */
#else
			// disable heartbeat
                        UNS16 Heartbeat_Producer_Time = 0x0; 
                        eprintf("Master : disable slave %2.2x heartbeat producer time \n", nodeId);
                        res = writeNetworkDictCallBack (d, /*CO_Data* d*/
                                        /**TestSlave_Data.bDeviceNodeId, UNS8 nodeId*/
                                        nodeId, /*UNS8 nodeId*/
                                        0x1017, /*UNS16 index*/
                                        0x00, /*UNS8 subindex*/
                                        2, /*UNS8 count*/
                                        0, /*UNS8 dataType*/
                                        &Heartbeat_Producer_Time,/*void *data*/
                                        CheckSDOAndContinue, /*SDOCallback_t Callback*/
                        0); /* use block mode */   
#endif
		}			
		break;

		case 2:
		{
                        Profile_Velocity = PROFILE_VELOCITY;
                        eprintf("Master : set profile velocity\n");
                        res = writeNetworkDictCallBack (d, /*CO_Data* d*/
                                        /**TestSlave_Data.bDeviceNodeId, UNS8 nodeId*/
                                        nodeId, /*UNS8 nodeId*/
                                        0x6081, /*UNS16 index*/
                                        0x00, /*UNS8 subindex*/
                                        4, /*UNS8 count*/
                                        0, /*UNS8 dataType*/
                                        &Profile_Velocity,/*void *data*/
                                        CheckSDOAndContinue, /*SDOCallback_t Callback*/
                        0); /* use block mode */
		}
		break;

                case 3: 
                {
                        Profile_Acceleration = PROFILE_ACCELERATION;      
                        eprintf("Master : set profile acceleration\n");     
                        res = writeNetworkDictCallBack (d, /*CO_Data* d*/
                                        /**TestSlave_Data.bDeviceNodeId, UNS8 nodeId*/
                                        nodeId, /*UNS8 nodeId*/
                                        0x6083, /*UNS16 index*/
                                        0x00, /*UNS8 subindex*/
                                        4, /*UNS8 count*/
                                        0, /*UNS8 dataType*/
                                        &Profile_Acceleration,/*void *data*/
                                        CheckSDOAndContinue, /*SDOCallback_t Callback*/
                        0); /* use block mode */
                }
                break;

                case 4:
                {
                        Profile_Deceleration = PROFILE_ACCELERATION;      
                        eprintf("Master : set profile deceleration\n");     
                        res = writeNetworkDictCallBack (d, /*CO_Data* d*/
                                        /**TestSlave_Data.bDeviceNodeId, UNS8 nodeId*/
                                        nodeId, /*UNS8 nodeId*/
                                        0x6084, /*UNS16 index*/
                                        0x00, /*UNS8 subindex*/
                                        4, /*UNS8 count*/
                                        0, /*UNS8 dataType*/
                                        &Profile_Deceleration,/*void *data*/
                                        CheckSDOAndContinue, /*SDOCallback_t Callback*/
                        0); /* use block mode */
                }
                break;

                case 5:
                {     
                        Motion_ProfileType = 1;
                        eprintf("Master : set profile type\n");     
                        res = writeNetworkDictCallBack (d, /*CO_Data* d*/
                                        /**TestSlave_Data.bDeviceNodeId, UNS8 nodeId*/
                                        nodeId, /*UNS8 nodeId*/         
                                        0x6086, /*UNS16 index*/          
                                        0x00, /*UNS8 subindex*/
                                        2, /*UNS8 count*/      
                                        0, /*UNS8 dataType*/   
                                        &Motion_ProfileType,/*void *data*/
                                        CheckSDOAndContinue, /*SDOCallback_t Callback*/
                        0); /* use block mode */            
                }
                break;

		case 6:
			// um?
			eprintf ("WHY THE HELL AM I HERE?\n");
			/* Put the master in operational mode */
			setState(d, Operational);
			  
			eprintf ("BOOTING UP NODE %02x\n", nodeId);
			/* Ask slave node to go in operational mode */
			masterSendNMTstateChange (d, nodeId, NMT_Start_Node);

#ifdef USE_HEARTBEAT
			// Add the node to the heartbeat proto
			UNS32	hb_data = (nodeId << 16) + HB_INTERVAL_MS + HB_INTERVAL_MS;
			if (debug) eprintf ("Set heartbeat for index 1 to %04x\n", hb_data);
			eprintf ("Adding slave %d to HB consumers with a value of %08X\n", nodeId, hb_data);
			UNS32	size_hb = sizeof (hb_data);

			writeLocalDict( &EPOScontrol_Data, /*CO_Data* d*/
                        	0x1016, /*UNS16 index*/                
                        	0x01, /*UNS8 subind*/           
                        	&hb_data, /*void * pSourceData,*/ 
                        	&size_hb, /* UNS8 * pExpectedSize*/
                        	RW);  /* UNS8 checkAccess */
#endif
	}
			
}

void TestMaster_preOperational(CO_Data* d)
{

	eprintf("TestMaster_preOperational\n");
/*
	masterSendNMTstateChange (&EPOScontrol_Data, slavenodeid, NMT_Reset_Node);
*/

	
/*
	eprintf("Set slave to preOperational\n");
	masterSendNMTstateChange (&EPOScontrol_Data, slavenodeid, NMT_Enter_PreOperational);
	eprintf("Slave set to preOperational\n");
*/

#ifdef USE_SYNC
	UNS32 interval = SYNC_INTERVAL_MS * 1000;
	UNS32 size = sizeof (interval);

        writeLocalDict( &EPOScontrol_Data, /*CO_Data* d*/
                        0x1006, /*UNS16 index*/
                        0x00, /*UNS8 subind*/ 
                        &interval, /*void * pSourceData,*/ 
                        &size, /* UNS8 * pExpectedSize*/
                        RW);  /* UNS8 checkAccess */

	startSYNC(&EPOScontrol_Data);
#endif
}

void TestMaster_operational(CO_Data* d)
{
	eprintf("NMT: TestMaster_operational\n");
	// dummy operational forewah!
	//while (1);
}

void TestMaster_stopped(CO_Data* d)
{
	eprintf("NMT: TestMaster_stopped\n");
}

void TestMaster_post_SlaveBootup(CO_Data* d, UNS8 SlaveID) {
	eprintf ("NMT: TestMaster_post_SlaveBootup : %02x\n", SlaveID);

/*
	if (bootup) {
		eprintf("Configuring the slave %d\n", SlaveID);
                //ConfigureSlaveNode(&EPOScontrol_Data, SlaveID);

		check_and_start_node(d, SlaveID);
	}
*/

    if (ds302_status(d) == BootCompleted) {
        // a post-boot slave came up. Well, configure it if able?
        
        ds302_boot_slave (d, SlaveID);
    }
}

void TestMaster_post_SlaveStateChange(CO_Data* d, UNS8 nodeId, e_nodeState newNodeState)
{
	eprintf ("NMT: TestMaster_post_SlaveStateChange : %02x -> %04x\n", nodeId, newNodeState);
}

void TestMaster_post_sync(CO_Data* d)
{
	if (debug) eprintf("TestMaster_post_SYNC\n");
}

void TestMaster_post_TPDO(CO_Data* d)
{
	if (debug) eprintf("TestMaster_post_TPDO\n");	
}

void TestMaster_post_emcy (CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg, const UNS8 errSpec[5])
{
	int	i;

	eprintf ("NMT: TestMaster_post_EMCY\n");
	eprintf ("\t%04x : %04x / %04x\n", nodeID, errCode, errReg);
	for (i=0; i < 5; i++) {
		eprintf ("\tERRTBL[%d] = %04x\n", i, errSpec[i]);
	}

    eprintf ("Error register: ");
    
    if (errReg & 0x80)
        eprintf ("M ");
    else
        eprintf ("  ");
    
    if (errReg & 0x20)
        eprintf ("D ");
    else
        eprintf ("  ");
    
    if (errReg & 0x10)
        eprintf ("C ");
    else
        eprintf ("  ");
    
    if (errReg & 0x08)
        eprintf ("T ");
    else
        eprintf ("  ");

    if (errReg & 0x04)
        eprintf ("V ");
    else
        eprintf ("  ");
    
    if (errReg & 0x02)
        eprintf ("A ");
    else
        eprintf ("  ");

    if (errReg & 0x01)
        eprintf ("G ");
    else
        eprintf ("  ");
    
    eprintf ("\n");
}

/*
	Implements the state machine for StatusWord

	States are based on bitmap of StatusWord x1xx xxx1 x111 1111
*/

#define EPOS_START      0x0000
#define EPOS_NOTREADY   0x0100
#define EPOS_SOD        0x0140
#define EPOS_RSO        0x0121
#define EPOS_SWO        0x0123
#define EPOS_REFRESH    0x4123
#define EPOS_MEASURE    0x4133
#define EPOS_OPEN       0x0137
#define EPOS_QUICKS     0x0117
#define EPOS_FRAD       0x010F
#define EPOS_FRAE       0x011F
#define EPOS_FAULT      0x0108

UNS32 StatusWordCallback (CO_Data* d, const indextable *idx, UNS8 bSubindex)
{
	if (debug) eprintf ("StWoCa!!!\n");
	EPOS_State = StatusWord & 0x417F;
	/***** NOTE: callback from PDO, NO MUTEXES! ****/
        switch (EPOS_State) {
                case EPOS_START:
                        if (debug) eprintf("Start\n");
                        break;
                case EPOS_NOTREADY:
                        if (debug) eprintf("Not Ready to Switch On\n");
                        // transition 1 to switch on disabled
                        // AUTOMATIC
                        break;
                case EPOS_SOD:
                        if (debug) eprintf("Switch On Disabled\n");
                        // transition 2 to ready to switch on
                        /* should be done if we REQUESTED to turn on */
                        //EnterMutex();
                        SET_BIT(ControlWord, 2);
                        SET_BIT(ControlWord, 1);
                        CLEAR_BIT(ControlWord, 0);              
                        //LeaveMutex();
                        break;
                case EPOS_RSO:
                        if (debug) eprintf("Ready to Switch On\n");
                        // transition 3 to switched on
                        // transition 7 to switch on disabled
                        /* 3 if requested to turn on, 7 if requested to shut down */
                        // this is #3
                        //EnterMutex();
                        SET_BIT(ControlWord, 2);
                        SET_BIT(ControlWord, 1);
                        SET_BIT(ControlWord, 0);
                        //LeaveMutex();
			break;
                case EPOS_SWO:
                        if (debug) eprintf("Switched on\n");
                        // transition 4 to refresh -> measure
                        // transition 6 to ready to switch on
                        // transition 10 to switch on disabled
                        /* 4 if requested to turn on, 6 or 10 if requested to shut down */
                        // this is #4
                        //EnterMutex();
                        SET_BIT(ControlWord, 3);
                        SET_BIT(ControlWord, 2);
                        SET_BIT(ControlWord, 1);
                        SET_BIT(ControlWord, 0);
                        //LeaveMutex();
                        break;
                case EPOS_REFRESH:
                        if (debug) eprintf("Refresh\n");
                        // transition 20 to measure
			// this should be automatic
                        break;
                case EPOS_MEASURE:
                        if (debug) eprintf("Measure Init\n");
                        // transition 21 to operation enable
			// this should be automatic
                        break;
                case EPOS_OPEN:
                        if (debug) eprintf("Operation enable\n");
                        // transition 5 to switched on
                        // transition 8 to readdy to switch on
                        // transition 9 to switch on disabled
                        // transition 11 to quick stop active
                        break;
                case EPOS_QUICKS:
                        if (debug) eprintf("Quick Stop Active\n");
                        // transition 16 to operation enable
                        // transition 12 to switch on disabled
                        break;
                case EPOS_FRAD:
                        if (debug) eprintf("Fault Reaction Active (disabled)\n");
                        // transition 18 to fault
                        break;
                case EPOS_FRAE:
                        if (debug) eprintf("Fault Reaction Active (enabled)\n");
                        // transition 14 to fault
                        break;
                case EPOS_FAULT:
                        if (debug) eprintf("Fault\n");
                        // transition 15 to switch on disabled
                        break;
                default:
                        eprintf("Bored to input codes. Unknown code %04x\n", EPOS_State);
        }

	// do the stupid assuming of position, and execution
	// if the set point ack is SET, then CLEAR the new set point flag
	if(BIT_IS_SET(StatusWord, 12)) {
		if (BIT_IS_SET(ControlWord, 4)) {
			CLEAR_BIT(ControlWord, 4);
			if (debug) eprintf ("New move ACK!\n");
		} else {
			eprintf ("What the FUCKING hell???\n");
		}
	}
#ifdef MAN_PDO
	sendPDOevent(&EPOScontrol_Data);
#endif
}

//s_BOARD SlaveBoard = {"0", "500K"};
s_BOARD MasterBoard = {(char *)"0", (char *)"1M"};

#if !defined(WIN32) || defined(__CYGWIN__)
void catch_signal(int sig)
{
  signal(SIGTERM, catch_signal);
  signal(SIGINT, catch_signal);
  
  eprintf("Got Signal %d\n",sig);
}
#endif

void help(void)
{
  printf("**************************************************************\n");
  printf("*  TestMasterMicroMod                                        *\n");
  printf("*                                                            *\n");
  printf("*  A simple example for PC.                                  *\n");
  printf("*  A CanOpen master that control a MicroMod module:          *\n");
  printf("*  - setup module TPDO 1 transmit type                       *\n");
  printf("*  - setup module RPDO 1 transmit type                       *\n");
  printf("*  - setup module hearbeatbeat period                        *\n");
  printf("*  - disable others TPDOs                                    *\n");
  printf("*  - set state to operational                                *\n");
  printf("*  - send periodic SYNC                                      *\n");
  printf("*  - send periodic RPDO 1 to Micromod (digital output)       *\n");
  printf("*  - listen Micromod's TPDO 1 (digital input)                *\n");
  printf("*  - Mapping RPDO 1 bit per bit (digital input)              *\n");
  printf("*                                                            *\n");
  printf("*   Usage:                                                   *\n");
  printf("*   ./TestMasterMicroMod  [OPTIONS]                          *\n");
  printf("*                                                            *\n");
  printf("*   OPTIONS:                                                 *\n");
  printf("*     -l : Can library [\"libcanfestival_can_virtual.so\"]     *\n");
  printf("*                                                            *\n");
  printf("*    Slave:                                                  *\n");
  printf("*     -i : Slave Node id format [0x01 , 0x7F]                *\n");
  printf("*                                                            *\n");
  printf("*    Master:                                                 *\n");
  printf("*     -m : bus name [\"1\"]                                    *\n");
  printf("*     -M : 1M,500K,250K,125K,100K,50K,20K,10K                *\n");
  printf("*                                                            *\n");
  printf("**************************************************************\n");
}

/***************************  INIT  *****************************************/
void InitNodes(CO_Data* d, UNS32 id)
{
	/****************************** INITIALISATION MASTER *******************************/
	if(MasterBoard.baudrate){
		/* Defining the node Id */
		// don't set node ID in masters
		//setNodeId(&EPOScontrol_Data, 0x7F);

		/* init */
		setState(&EPOScontrol_Data, Initialisation);
	}
}

/***************************  EXIT  *****************************************/
void Exit(CO_Data* d, UNS32 id)
{
	masterSendNMTstateChange(&EPOScontrol_Data, 0x7F, NMT_Reset_Node);

    	//Stop master
	setState(&EPOScontrol_Data, Stopped);
}


void printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;

    for (i=size-1;i>=0;i--)
    {
        for (j=7;j>=0;j--)
        {
            byte = b[i] & (1<<j);
            byte >>= j;
            eprintf("%u", byte);
        }
    }
}

void	printStatusword () {

	UNS16	sw = StatusWord;
	
	eprintf("\nStatusWord: ");printBits (sizeof (sw), &sw);eprintf("\n");
	UNS16 swstate = sw & 0b0100000101111111;
	eprintf("State:      ");printBits (sizeof (swstate), &swstate);eprintf("\n");

	eprintf ("Status mach END\n\n");
}

extern subindex EPOScontrol_Index1F22[];

void setup_dcf(void)
{
    uint8_t subidx;
    uint8_t nbr_subidx = *(uint8_t *)EPOScontrol_Index1F22[0].pObject;
    printf("setup_dcf : %u sub indexes to set\n", nbr_subidx);

    for (subidx = 0; subidx < nbr_subidx; subidx++) {
        // zero out the data
        dcfdatas[subidx][0] = 0x00;
        dcfdatas[subidx][1] = 0x00;
        dcfdatas[subidx][2] = 0x00;
        dcfdatas[subidx][3] = 0x00;
    }

    dcf_read_in_file(DEVICE_DICT_NAME, dcfdatas);
    dcf_data_display(dcfdatas);
    for(subidx = 0 ; subidx < nbr_subidx ; subidx++){
        EPOScontrol_Index1F22[subidx + 1].pObject = dcfdatas[subidx];
        EPOScontrol_Index1F22[subidx + 1].size = DCF_MAX_SIZE;
    }
}


/*
 Sets the local dict for slaveID
*/
void setup_local_params (UNS8 slaveID)
{

	int	idx = slaveID - 1;
/*
 Set the client SDO values on the proper index
*/
	UNS32	sdo_rx = 0x580 + slaveID;
	UNS32	sdo_tx = 0x600 + slaveID;
	UNS32	size = sizeof (UNS32);

	// write TX SDO
	writeLocalDict( &EPOScontrol_Data, /*CO_Data* d*/
		0x1280 + idx, /*UNS16 index*/
		0x01, /*UNS8 subind*/
		&sdo_tx, /*void * pSourceData,*/       
		&size, /* UNS8 * pExpectedSize*/      
		RW);  /* UNS8 checkAccess */

	// write RX SDO
        writeLocalDict( &EPOScontrol_Data, /*CO_Data* d*/
                0x1280 + idx, /*UNS16 index*/        
                0x02, /*UNS8 subind*/        
                &sdo_rx, /*void * pSourceData,*/             
                &size, /* UNS8 * pExpectedSize*/      
                RW);  /* UNS8 checkAccess */

	// write slave ID
	size = sizeof (slaveID);
        writeLocalDict( &EPOScontrol_Data, /*CO_Data* d*/
                0x1280 + idx, /*UNS16 index*/        
                0x03, /*UNS8 subind*/        
                &slaveID, /*void * pSourceData,*/             
                &size, /* UNS8 * pExpectedSize*/      
                RW);  /* UNS8 checkAccess */	

/*
 Set the RxPDOs and TxPDOs on the proper index
*/
	// we reserve 4 PDOs for each slave for EPOS, that's the max supported by device
	int	pdo_idx = idx * 4;

	// receive PDOs

	UNS32	rxpdo_id = 0x180 + slaveID;
	size = sizeof (UNS32);
        writeLocalDict( &EPOScontrol_Data, /*CO_Data* d*/
                0x1400 + pdo_idx + 0, /*UNS16 index*/
                0x01, /*UNS8 subind*/        
                &rxpdo_id, /*void * pSourceData,*/             
                &size, /* UNS8 * pExpectedSize*/      
                RW);  /* UNS8 checkAccess */    

        rxpdo_id = 0x280 + slaveID;
        size = sizeof (UNS32);
        writeLocalDict( &EPOScontrol_Data, /*CO_Data* d*/
                0x1400 + pdo_idx + 1, /*UNS16 index*/          
                0x01, /*UNS8 subind*/        
                &rxpdo_id, /*void * pSourceData,*/             
                &size, /* UNS8 * pExpectedSize*/      
                RW);  /* UNS8 checkAccess */    

        rxpdo_id = 0x380 + slaveID;
        size = sizeof (UNS32);
        writeLocalDict( &EPOScontrol_Data, /*CO_Data* d*/
                0x1400 + pdo_idx + 2, /*UNS16 index*/          
                0x01, /*UNS8 subind*/        
                &rxpdo_id, /*void * pSourceData,*/             
                &size, /* UNS8 * pExpectedSize*/      
                RW);  /* UNS8 checkAccess */    

        rxpdo_id = 0x480 + slaveID;
        size = sizeof (UNS32);
        writeLocalDict( &EPOScontrol_Data, /*CO_Data* d*/
                0x1400 + pdo_idx + 3, /*UNS16 index*/          
                0x01, /*UNS8 subind*/        
                &rxpdo_id, /*void * pSourceData,*/             
                &size, /* UNS8 * pExpectedSize*/      
                RW);  /* UNS8 checkAccess */    

	// transmit PDOs

        UNS32   txpdo_id = 0x200 + slaveID;
	UNS8	trans_type = 0xFF;
        size = sizeof (UNS32);               
        writeLocalDict( &EPOScontrol_Data, /*CO_Data* d*/    
                0x1800 + pdo_idx + 0, /*UNS16 index*/ 
                0x01, /*UNS8 subind*/        
                &txpdo_id, /*void * pSourceData,*/             
                &size, /* UNS8 * pExpectedSize*/      
                RW);  /* UNS8 checkAccess */
	size = sizeof (UNS8);
        writeLocalDict( &EPOScontrol_Data, /*CO_Data* d*/    
                0x1800 + pdo_idx + 0, /*UNS16 index*/ 
                0x02, /*UNS8 subind*/        
                &trans_type, /*void * pSourceData,*/             
                &size, /* UNS8 * pExpectedSize*/      
                RW);  /* UNS8 checkAccess */    	


        txpdo_id = 0x300 + slaveID;
        size = sizeof (UNS32);               
        writeLocalDict( &EPOScontrol_Data, /*CO_Data* d*/    
                0x1800 + pdo_idx + 1, /*UNS16 index*/ 
                0x01, /*UNS8 subind*/        
                &txpdo_id, /*void * pSourceData,*/             
                &size, /* UNS8 * pExpectedSize*/      
                RW);  /* UNS8 checkAccess */    
        size = sizeof (UNS8);
        writeLocalDict( &EPOScontrol_Data, /*CO_Data* d*/    
                0x1800 + pdo_idx + 1, /*UNS16 index*/ 
                0x02, /*UNS8 subind*/        
                &trans_type, /*void * pSourceData,*/             
                &size, /* UNS8 * pExpectedSize*/      
                RW);  /* UNS8 checkAccess */     
}

void timer_play(CO_Data* d, UNS32 id)
{
	eprintf ("TIMER: play with ID %x, %d\n", id, getElapsedTime());
}

int     ds302_nl_keepalive_nodes_present(CO_Data* d);

/****************************************************************************/
/***************************  MAIN  *****************************************/
/****************************************************************************/
int main(int argc,char **argv)
{

  int c;
  extern char *optarg;
  char* LibraryPath= (char *)"libcanfestival_can_socket.so";
  char *snodeid;
  while ((c = getopt(argc, argv, "-m:s:M:S:l:i:")) != EOF)
  {
    switch(c)
    {
      case 'm' :
        if (optarg[0] == 0)
        {
          help();
          exit(1);
        }
        MasterBoard.busname = optarg;
        break;
      case 'M' :
        if (optarg[0] == 0)
        {
          help();
          exit(1);
        }
        MasterBoard.baudrate = optarg;
        break;
      case 'l' :
        if (optarg[0] == 0)
        {
          help();
          exit(1);
        }
        LibraryPath = optarg;
        break;
      case 'i' :
        if (optarg[0] == 0)
        {
          help();
          exit(1);
        }
        snodeid = optarg;
		sscanf(snodeid,"%x",&slavenodeid);
        break;
      default:
        help();
        exit(1);
    }
  }

	setup_dcf ();

	// Set the master ID
	// THE STUPID MASTER ID SET WILL REWRITE THE SDOs/PDOs
	setNodeId(&EPOScontrol_Data, 0x7F);

    // load the DCF configuration for the master node before starting the timers and such
    ds302_load_dcf_local (&EPOScontrol_Data);

#if !defined(WIN32) || defined(__CYGWIN__)
  /* install signal handler for manual break */
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);
	TimerInit();
#endif

#ifndef NOT_USE_DYNAMIC_LOADING
	LoadCanDriver(LibraryPath);
#endif		

	//ds302_nl_keepalive_nodes_present(&EPOScontrol_Data);

	//sleep(60);

	EPOScontrol_Data.heartbeatError = TestMaster_heartbeatError;
	EPOScontrol_Data.initialisation = TestMaster_initialisation;
	EPOScontrol_Data.preOperational = TestMaster_preOperational;
	EPOScontrol_Data.operational = TestMaster_operational;
	EPOScontrol_Data.stopped = TestMaster_stopped;
	EPOScontrol_Data.post_TPDO = TestMaster_post_TPDO;
	// SYNC
	EPOScontrol_Data.post_sync = TestMaster_post_sync;
    
    // The error control routines only get into action after the ds302 boot
	EPOScontrol_Data.post_SlaveBootup = TestMaster_post_SlaveBootup;
	EPOScontrol_Data.post_SlaveStateChange = TestMaster_post_SlaveStateChange;
	// EMCY
	EPOScontrol_Data.post_emcy = TestMaster_post_emcy;

	// Register Call My Backs
	// Call My Back 1 : do the state machine for StatusWord
	//RegisterSetODentryCallBack (&EPOScontrol_Data, 0x4001, 0x00, StatusWordCallback);
	
	UNS16 cw = 0x01;

	if(!canOpen(&MasterBoard,&EPOScontrol_Data)){
		eprintf("Cannot open Master Board\n");
		goto fail_master;
	}
	
	// Start timer thread
	StartTimerLoop(&InitNodes);
	
	/* doing the boot process in the MAIN loop */
	
        // Move to DS 302 procedures

        //ds302_preOperational_preBoot (&EPOScontrol_Data);
        //ds302_slaveBootprocess (&EPOScontrol_Data);
        //ds302_preOperational_postBoot (&EPOScontrol_Data);

	// use the new state machine
	//INIT_SM(BOOTMASTER,_masterBoot,MB_INITIAL);

    ds302_init (&EPOScontrol_Data);
    ds302_setHeartbeat (&EPOScontrol_Data, 0x01, 1200);

	EnterMutex();
	ds302_start (&EPOScontrol_Data);
	LeaveMutex();

	// timer play
	// MS_TO_TIMEVAL(ms) or US_TO_TIMEVAL(us) to create the timevalentries

	//SetAlarm (&EPOScontrol_Data, 0x12334, timer_play, MS_TO_TIMEVAL(1000), MS_TO_TIMEVAL(1000));

	//printStatusword ();

	eprintf ("EPOS loop started, waiting for enabled!\n");

	while (EPOS_State != EPOS_OPEN) sleep(1);

	eprintf ("EPOS ready for operation!\n");
	eprintf ("Setting PPM params\n");
	EnterMutex();
	SET_BIT(ControlWord, 5); // 1 start immmediately, interrupt in progress if any. 0 finish previous first
	CLEAR_BIT(ControlWord, 6); // 0 absolute, 1 relative
	// to make the stupid thing move, you put it to 1, and then to 0 to start
	//SET_BIT(ControlWord, 4); // 1 assumes target, 0 does not assume target
	CLEAR_BIT(ControlWord, 8); // 0 execute, 1 halt
#ifdef MAN_PDO
        sendPDOevent(&EPOScontrol_Data);
#endif
	LeaveMutex();
	sleep(2);

#define CYCLES		0
#define STEP_SIZE	20000
#define SLEEP_TIME	250

	int	i;
	for (i=0; i<CYCLES; i++) {
		eprintf ("Moving to %d\n", i);
		int newposition = i * STEP_SIZE;
		EnterMutex();
		//Target_Position = newposition;
		SET_BIT(ControlWord, 4); // 1 means NEW setpoint, it's cleared in callback once ACK
		Target_Position = newposition;
#ifdef MAN_PDO
                sendOnePDOevent(&EPOScontrol_Data, 1);
#endif
		LeaveMutex();
		eprintf ("Executing move...\n");
		sleep_ms(SLEEP_TIME);
	}

	for (i=CYCLES; i>=0; i--) {
                eprintf ("Moving to %d\n", i);
                int newposition = i * STEP_SIZE;
                EnterMutex();
                //Target_Position = newposition;
                SET_BIT(ControlWord, 4); // 1 means NEW setpoint, it's cleared in callback once ACK
                Target_Position = newposition;
#ifdef MAN_PDO
		sendOnePDOevent(&EPOScontrol_Data, 1);
#endif
                LeaveMutex();
                eprintf ("Executing move...\n");      
                sleep_ms(SLEEP_TIME);

	}
	eprintf ("Moves finished\n");

	// wait Ctrl-C
	pause();
	eprintf("Finishing.\n");

	init_step = 0;
	bootup = 0;

	EnterMutex();
        masterSendNMTstateChange (&EPOScontrol_Data, slavenodeid, NMT_Enter_PreOperational);
	LeaveMutex();
	eprintf ("Slave set to pre-op\n");
	sleep(1);

	setState (&EPOScontrol_Data, Pre_operational);
	eprintf ("Master set to pre-op\n");
	sleep(1);

	// Stop master
	setState(&EPOScontrol_Data, Stopped);
	eprintf ("Master stopped\n");
	
	// Stop timer thread
	StopTimerLoop(&Exit);
	
	eprintf ("Timer loop stopped\n");

fail_master:
	if(MasterBoard.baudrate) canClose(&EPOScontrol_Data);	
	eprintf ("CAN closed\n");

	TimerCleanup();
  	return 0;
}


