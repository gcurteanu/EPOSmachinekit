
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

#include "epos.h"

#include <pthread.h>


//#include "epos_api.h"

unsigned int slavenodeid = 0x01;
unsigned int bootup = 1;

unsigned int debug = 0;

uint64_t    almClock = 0;
uint64_t    almClock2 = 0;                 

//#define USE_HEARTBEAT
#define HB_INTERVAL_MS		1000
//#define USE_SYNC
#define SYNC_INTERVAL_MS 	10
#define MAN_PDO


#define PROFILE_VELOCITY	1500
#define PROFILE_ACCELERATION	50000

static void sleep_ms ( unsigned long ms )
{
        struct timespec sleep, left;

        sleep.tv_sec = ms / 1000;
        sleep.tv_nsec = (ms % 1000) * 1000 * 1000;
        nanosleep (&sleep, &left);

        //rt_timer_spin (ms * 1000 * 1000);
        //rt_task_sleep (ms * 1000 * 1000);
}


static void sleep_us ( unsigned long us )
{
        struct timespec sleep, left;

        sleep.tv_sec = us / 1000 * 1000;
        sleep.tv_nsec = (us % (1000 * 1000)) * 1000;
        nanosleep (&sleep, &left);

        //rt_timer_spin (ms * 1000 * 1000);
        //rt_task_sleep (ms * 1000 * 1000);
}

/*****************************************************************************/
void TestMaster_heartbeatError(CO_Data* d, UNS8 heartbeatID)
{
    eprintf("TestMaster_heartbeatError %d\n", heartbeatID);
}

/********************************************************
 * ConfigureSlaveNode is responsible to
 *  - setup master RPDO 1 to receive TPDO 1 from id 0x40
 *  - setup master TPDO 1 to send RPDO 1 to id 0x40
 ********************************************************/
void TestMaster_initialisation(CO_Data* d)
{
}

static int init_step = 0;

#ifdef _NOT_USED_
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
#endif

void TestMaster_preOperational(CO_Data* d)
{
    eprintf("TestMaster_preOperational\n");
}

void TestMaster_operational(CO_Data* d)
{
    eprintf("NMT: TestMaster_operational\n");

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
    eprintf("Starting SYNC at a %d ms interval\n", SYNC_INTERVAL_MS);
#endif
}

void TestMaster_stopped(CO_Data* d)
{
    eprintf("NMT: TestMaster_stopped\n");
}

void TestMaster_post_SlaveBootup(CO_Data* d, UNS8 SlaveID) {
    eprintf ("NMT: TestMaster_post_SlaveBootup : %02x\n", SlaveID);

    /*if (ds302_status(d) == BootCompleted) {
        // a post-boot slave came up. Well, configure it if able?
        
        ds302_boot_slave (d, SlaveID);
    }*/
}

void TestMaster_post_SlaveStateChange(CO_Data* d, UNS8 nodeId, e_nodeState newNodeState)
{
    /* this will be called if a NMT message / state change (timeout error, etc) happens */
    eprintf ("NMT: TestMaster_post_SlaveStateChange : %02x -> %04x\n", nodeId, newNodeState);
}

void TestMaster_post_sync(CO_Data* d)
{
    /* called after a SYNC is generated */
    if (debug) eprintf("TestMaster_post_SYNC\n");
}

void TestMaster_post_TPDO(CO_Data* d)
{
    /* called at the TPDO transmission */
    if (debug) eprintf("TestMaster_post_TPDO\n");	
}

void TestMaster_post_emcy (CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg, const UNS8 errSpec[5])
{
    /* called for EMCY receive */
    /* this is the only way to manage received errors*/

    eprintf ("NMT: TestMaster_post_EMCY\n");
    eprintf ("\t%04x : %04x / %04x\n", nodeID, errCode, errReg);
/*
    // this displays the remaining 5 bytes of the EMCY
    // those are unused
    int	i;
    for (i=0; i < 5; i++) {
        eprintf ("\tERRTBL[%d] = %04x\n", i, errSpec[i]);
    }
*/

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
    
    eprintf ("Error detail: %04x -> %s\n", errCode, epos_error_text (errCode));
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


/***************************  INIT  *****************************************/
void InitNodes(CO_Data* d, UNS32 id)
{
    /* set to init */
    setState(&EPOScontrol_Data, Initialisation);
}

/***************************  EXIT  *****************************************/
void Exit(CO_Data* d, UNS32 id)
{
    masterSendNMTstateChange(&EPOScontrol_Data, 0x7F, NMT_Stop_Node);

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

void    printStatusword () {

    UNS16	sw = StatusWord[0];

    eprintf("\nStatusWord: ");printBits (sizeof (sw), &sw);eprintf("\n");
    UNS16 swstate = sw & 0b0100000101111111;
    eprintf("State:      ");printBits (sizeof (swstate), &swstate);eprintf("\n");

    eprintf ("Status mach END\n\n");
}

#define TIMER_USEC  100000

void timer_play(CO_Data* d, UNS32 id)
{
    uint64_t    crtclock = rtuClock(); //rt_timer_read();
    uint64_t    cdiff = crtclock - almClock;
    int diff = cdiff - TIMER_USEC;

    if (diff > 100 || diff < -100) eprintf ("1 : elapsed %6lld %4d\n", cdiff, diff);
    almClock = crtclock;
}

#define TIMER2_USEC  50000
void timer_play2(CO_Data* d, UNS32 id)
{
    uint64_t    crtclock = rtuClock(); //rt_timer_read();
    uint64_t    cdiff = crtclock - almClock2;
    int diff = cdiff - TIMER2_USEC;

    if (diff > 100 || diff < -100) eprintf ("2 : elapsed %6lld %4d\n", cdiff, diff);
    almClock2 = crtclock; 
}

int     ds302_nl_keepalive_nodes_present(CO_Data* d);

/****************************************************************************/
/***************************  MAIN  *****************************************/
/****************************************************************************/
int main(int argc,char **argv)
{

  char* LibraryPath= (char *)"libcanfestival_can_socket.so";

    //setup_dcf ();
    
    // Set the master ID
    // THE STUPID MASTER ID SET WILL REWRITE THE SDOs/PDOs
    setNodeId(&EPOScontrol_Data, 0x7F);

    epos_initialize_master (&EPOScontrol_Data, "dcfdata.txt");
    display_dcf_set (&EPOS_drive.dcf_data);
    epos_add_slave (0x01);
    
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

	EPOScontrol_Data.heartbeatError = TestMaster_heartbeatError;
	EPOScontrol_Data.initialisation = TestMaster_initialisation;
	EPOScontrol_Data.preOperational = TestMaster_preOperational;
	EPOScontrol_Data.operational = TestMaster_operational;
	EPOScontrol_Data.stopped = TestMaster_stopped;
	//EPOScontrol_Data.post_TPDO = TestMaster_post_TPDO;
	// SYNC
	//EPOScontrol_Data.post_sync = TestMaster_post_sync;
    
    // The error control routines only get into action after the ds302 boot
	EPOScontrol_Data.post_SlaveBootup = TestMaster_post_SlaveBootup;
	EPOScontrol_Data.post_SlaveStateChange = TestMaster_post_SlaveStateChange;
	// EMCY
	EPOScontrol_Data.post_emcy = TestMaster_post_emcy;
	
	if(!canOpen(&MasterBoard,&EPOScontrol_Data)){
		eprintf("Cannot open Master Board\n");
		goto fail_master;
	}
	
	// Start timer thread
	StartTimerLoop(&InitNodes);


    // BECOME REALTIME
    struct sched_param  param = { .sched_priority = 80 };       
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);     


    //EnterMutex();
    //SetAlarm (&EPOScontrol_Data, 0x12334, timer_play, US_TO_TIMEVAL(TIMER_USEC), US_TO_TIMEVAL(TIMER_USEC));
    //SetAlarm (&EPOScontrol_Data, 0x56789, timer_play2, US_TO_TIMEVAL(TIMER2_USEC), US_TO_TIMEVAL(TIMER2_USEC));    
    //LeaveMutex();


	/* doing the boot process in the MAIN loop */
	
    ds302_init (&EPOScontrol_Data);
    // profile position mode
    OperationMode[0] = 1;

	EnterMutex();
	ds302_start (&EPOScontrol_Data);
	LeaveMutex();

    ds302_setHeartbeat (&EPOScontrol_Data, 0x01, 60);


	eprintf ("EPOS loop started, waiting for boot to complete!\n");

	while (ds302_status(&EPOScontrol_Data) != BootCompleted) sleep_ms(100);

	eprintf ("EPOS ready for operation!\n");
	eprintf ("Setting PPM params and enable drive\n");

	EnterMutex();

    epos_set_continuous (0);
    //epos_set_segmented(0);
    epos_set_absolute (0);
    epos_execute (0);

	///SET_BIT(ControlWord[0], 5); // 1 start immmediately, interrupt in progress if any. 0 finish previous first
	///CLEAR_BIT(ControlWord[0], 6); // 0 absolute, 1 relative
	///CLEAR_BIT(ControlWord[0], 8); // 0 execute, 1 halt

    VelocityDemandValue[0] = 1000;

    epos_enable_drive (0);
    epos_set_mode (0, EPOS_MODE_PPM);
    // load values to the drive
    sendPDOevent(&EPOScontrol_Data);

	LeaveMutex();

    eprintf ("PPM parameters done, ensuring drive is enabled\n");

#define CYCLES		400
#define STEP_SIZE	1000
#define SLEEP_TIME	10
#define SETTLE_TIME 100

    int     faulted = 0;
    // wait to become operational
    while (!epos_drive_operational(0)) {
        if (epos_drive_faulted(0)) {
            // try to clear fault
            eprintf ("Faulted, try to clear fault ()\n");
            EnterMutex();
            epos_fault_reset(0);
            sendPDOevent(&EPOScontrol_Data);
            LeaveMutex();
            faulted++;
        } else if (epos_drive_disabled(0) && faulted) {
            // we end up here after a fault reset
            printf ("Fault cleared, try to start ()\n");
            EnterMutex();
            epos_enable_drive(0);
            sendPDOevent(&EPOScontrol_Data);
            LeaveMutex();        
        }
        sleep_ms (SLEEP_TIME);
    }

    eprintf ("Drive is ready for operation\n");

    int     cycle = 0;

    INTEGER32   position = 0;

    // run a cycle with the specified periodicity
    while (1) {

        // we do + 1 since we move from 0 to CYCLES.
        if (cycle>CYCLES)
            break;

        if (epos_drive_faulted(0) || epos_drive_disabled(0))    // bail out on error
            break;

        EnterMutex();
        if (epos_do_move_PPM(0, position)) {
            // increment target
            position += STEP_SIZE;
            // increment cycle
            cycle++;
            //eprintf ("Completed cycle %d\n", cycle);
        }
        LeaveMutex();
        sleep_ms (SLEEP_TIME);
    };

    // decrement the position to account for the overshoot
    position -= STEP_SIZE;
    eprintf ("Forward done, final position executed %ld, cycle=%d\n", position, cycle);
    eprintf ("Waiting for moves to complete\n");

    while (!epos_in_position(0))
        sleep_ms (SLEEP_TIME);
    eprintf ("Servo in position\n");

    sleep_ms(SETTLE_TIME);
    eprintf ("Reversing...\n");

    // run a cycle with the specified periodicity
    while (1) {

        if (cycle <= 0)
            break;

        if (epos_drive_faulted(0) || epos_drive_disabled(0))    // bail out on error
            break;

        EnterMutex();
        if (epos_do_move_PPM(0, position)) {
            // move was OK, increment the value
            position -= STEP_SIZE;
            // decrement cycle
            cycle--;
            //eprintf ("Completed cycle %d\n", cycle);
        }
        LeaveMutex();
        sleep_ms (SLEEP_TIME);
    };
    // increment the position to account for the overshoot
    position += STEP_SIZE;
    eprintf ("All moves done, final position is %ld\n", position);

    eprintf ("Waiting for moves to complete\n");

    while (!epos_in_position(0))
        sleep_ms (SLEEP_TIME);
    eprintf ("Servo in position\n");

    sleep_ms(SETTLE_TIME);

	//pause();
	eprintf("Finishing.\n");

    EnterMutex();    
    epos_disable_drive (0);
    sendPDOevent(&EPOScontrol_Data);
    LeaveMutex();
    sleep_ms(SETTLE_TIME);

    pause();

	init_step = 0;
	bootup = 0;

	EnterMutex();
    masterSendNMTstateChange (&EPOScontrol_Data, slavenodeid, NMT_Enter_PreOperational);
	LeaveMutex();
	eprintf ("Slave set to pre-op\n");
	sleep_ms(SETTLE_TIME);

	setState (&EPOScontrol_Data, Pre_operational);
	eprintf ("Master set to pre-op\n");
	sleep_ms(SETTLE_TIME);

	// Stop master
	setState(&EPOScontrol_Data, Stopped);
	eprintf ("Master stopped\n");
	
	// Stop timer thread
	StopTimerLoop(&Exit);
	eprintf ("Timer loop stopped\n");

fail_master:
	//if(MasterBoard.baudrate) canClose(&EPOScontrol_Data);	
	//eprintf ("CAN closed\n");

	TimerCleanup();
  	return 0;
}
