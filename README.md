# EPOSmachinekit
CANopen EPOS controller for Machinekit
...

##Main goal for this project:

- Develop a Machinekit/LinuxCNC component able to control CANopen Maxon EPOS positioning controllers (or with minimal changes other DS-402 drives)

Component should be able to control multiple drives, and do so with a simple configuration. Due to the fact there can be only one NMT master, and only one Configuration Manager, 
the 2 are combined into a CAN manager and only ONE instance can be present. This should be handled via the LinuxCNC/MK component.
(note: There is a possibility to have multiple CAN buses, and each can have it's own. Will probably require minor component changes to support multi-instance)

Currently implements: CiA 301 (CanFestival mostly), CiA 302 (own code) as Configuration Manager / Master boot, CiA 402 drives
Not Implemented: CiA 302 software download, SDO manager, NMT requests. Not planned, not useful for the proposed scope

##Targets:
- create the required CanFestival infrastrucuture to support the CANopen operations
    * current status: _*mostly complete*_
- support for the DS-302 boot process and automatic configuration updates for the CANopen devices via concise DCF (0x1F22). The CanFestival existing DCF seems to be lacking a lot of required features
    * current status: _*CiA 302 done, works, needs various updates*_
- support for the DS-402 state machine allowing the drive operation
    * current status: _*CiA 402 state machines present, needs further features*_
- support for profile position / speed as well as direct position / velocity modes as a minimum. Probably the profile modes will not be useful directly and we'll end up using the built in profile generator in Machinekit/LinuxCNC
    * current status: _*PPM completed along with the Command/Acknowledge state machine for multiple profile moves (not used in LinuxCNC/MK)*_ . This was tested on real hardware running under LinuxCNC/Machinekit as axis A and provides a VERY good and stable response with a 1kHz servo loop.

##Current status:

* Jan 2015:
  - LinuxCNC/Machinekit component created, and tested. Works better than expected. Additional code, cleanup in progress. Support for multi-axis operation ongoing
  - next target is to integrate the ConciseDCF into a larger configuration method, yet to be determined. ConciseDCF is VERY good for hardware operation/upload, but writing it is problematic/error prone
  - update the CiA 302 to the latest version I could find

* Dec 2015:
  - initial (messy) implementation done to learn the CanFestival stack capabilities and modes to control the drive. Along with constraints due to the RT drivers.
It is functional, and able to position the drive and get position data in realtime with minimal external coding.
Moves from external application (MK/LinuxCNC) should be done by simply working to the values in the OD, the TPDOs/RPDOs taking over independently from there.
  - initial (messy) implementation of the DS-302 code, moving to a more standard state machine setup (ongoing). Still lacking the DCF upload
  - initial (very messy) DS-402 state machine support (just goes to Operation Enabled if able and keeps it there)


##Hardware:
- Peak PCI dual port (opto isolated) CAN card
- Xenomai Machinekit kernel running on Debian
- Using the RT-CAN socket interfaces (not the regular ones) included in the kernel
- No need for the Peak Linux driver. It is not used / useful from what can be seen
- The RT-CAN in the kernel seems stable, but can not cope with Linux interrupt sharing when used in the RT model. Peak card MUST NOT share it's IRQ with other devices... Or else kernel panics.

As far as positioning controller, Maxon EPOS 70/10. Looks like a very standard implementation of DS 402 is used, so if it works for this, it will work for others as well
testing the CAN with a 25mtr CAN cable with 120ohm termination at both ends, point to point

##Items needing attention:
- serious bug in CanFestival timer scheduling. Due to a TIMEVAL to UNS32 conversion, in case of negative values, leads to the timers to misfire BADLY. Redefined the overrun as timeval, and removed the conversion.
Not even sure why that was there since all operations are on TIMEVAL , not UNS32. Quick fix but was a pain to figure out
```
In file src/timer.c, start of TimeDispatch function, change as follows:
        /* Get time since timer signal */
        TIMEVAL overrun = getElapsedTime();
```

- there is no IOCT for loopback in the Xenomai API as shipped with the Debian/MK packages.
This is easy to fix, the code block in the drivers/can_socket/can_socket.c
```
/*
  {
    int loopback = 1;
    err = CAN_SETSOCKOPT(*(int *)fd0, SOL_CAN_RAW, CAN_RAW_LOOPBACK,
               &loopback, sizeof(loopback));
    if (err) {
        fprintf(stderr, "rt_dev_setsockopt: %s\n", strerror (CAN_ERRNO (err)));
        goto error_close;
    }
  }
*/
```
In fact, using rtcanconfig this can be set on the card itself (ex: rtcanconfig rtcan0 -c loopback -b 1000000 start)
This is ONLY needed in case of using SYNC telegrams, since those are generated over the wire and otherwise are NOT received/processed by the local node
SYNC is NOT USED at this moment (planned, but on a 1ms cycle it will stop a random amount of time. Not sure why yet, planned feature in order to support SYNC based PDOs)

# DCF file format

The format for initializing slaves is the DCF format, but not sure if a full implementation is required.
A full DCF parser is quite a lot of work and will not add much compared to binary ConciseDCF streams.

Standard DCF format is a bit complex for our needs, and also requires some application logic to apply in case of the PDO mappings
(per 301 : disable PDO, set mapping count to zero, load mapping, configure PDO, enable mapping and PDO)

"Subindex 0 determines the valid number of objects that have been mapped. For changing the PDO
mapping first the PDO has to be deleted, the sub-index 0 must be set to 0 (mapping is deactivated).
Then the objects can be remapped. When a new object is mapped by wrinting a subindex between 1
and 64, the device may check whether the object specified by index / sub-index exists. If the object
does not exist or the object cannot be mapped, the SDO transfer must be aborted with the Abort SDO
Transfer Service with one of the abort codes 0602 0000h or 0604 0041h.
After all objects are mapped subindex 0 is set to the valid number of mapped objects. Finally the PDO
will be created by writing to its communication parameter COB-ID. When subindex 0 is set to a value
>0 the device may validate the new PDO mapping before transmitting the response of the SDO
service. If an error is detected the device has to transmit the Abort SDO Transfer Service with one of
the abort codes 0602 0000h, 0604 0041h or 0604 0042h."

Our format should directly describe the steps and should be a 1:1 mapping to SDO actions. One problem to cope with is describing the data size of the objects.
There is a ConciseDCF implementation in CanFestival but that is rather rudimentary and not CiA 302 compliant.

Proposed format (pretty much the same as the one used in the dcf example from CanFestival):
[slave id]
Index Subindex Size Value
...

slave id = decimal/hex *16 bits used)
index / subindex = decimal/hex (8 bits used)
size = decimal/hex in BYTES (full value used, but it's pointless to be more than 4)
value = decimal/hex (max 32 bits used) .

Example:
[1]
0x1400 0x01 4 0x10000000
...

~~For initial testing, we will use the existing DCF example code from CanFestival~~
Updated ConciseDCF code was implemented and there's also a capability to configure the master via ConciseDCF 
outside the normal CiA 302 boot process (prior to CiA 302)



# Machinekit/LinuxCNC interface

## Module options

- master_can_id='<CAN ID>'
Sets the CAN ID of the master node (the component). It is by default 0x7F

- slaveid=<slave1>,<slave2>,...,slave<EPOS_MAX_DRIVES>
Defines the slaves that are being managed. Those are added as MANDATORY slaves in the CiA 302 process, module will NOT start if one of the slaves is not responding or has an error.

- heartbeat=<slave1>,<slave2>,...,slave<EPOS_MAX_DRIVES>
Defines the heartbeat for each slave (as a CONSUMER). The producer side (on the drive itself) needs to be either configured manually or via CDCF and MUST match the value set in order to prevent 
drives being detected as disconnected. Set the hearbeat time for the slave at least as <hb producer time>*1.5 for small values, take into account some amount of jitter will be present

_*NOTE: if a heartbeat is set it WILL be used during the boot process. Boot will stop waiting to receive a heartbeat from the slave. A zero values disables heartbeat checking*_

- dcf=<filename>
The DCF file name containing the data for configuring the slaves at boot-up time. THIS IS MANDATORY (for now, due to code not being 100% right). 
Each defined slaveid must have at least one entry in the file, for example setting the heartbeat producer time (ex for a 50ms heartbeat: 0x1017 0x00 2 0x0032)

## Pins / parameters

- param slave-count
The number of slaves that are being managed

- param '<driveno>'.slave-id
The CAN ID of the slave for that particular drive

- pin '<driveno>'.enable
The pin enables / disables the drive.
When enable goes high, drive seeks to get to the enabled state, clearing all the drive errors in the process
When enable goes low, drive is disabled (does a full stop via quickstop automatically?)

- pin '<driveno>'.enabled
The drive is enabled (?) Not implemented. Maybe a "ready" signal to indicate presence? 

- pin '<driveno>'.faulted
At least one of the drives is faulted. This happens at heartbeat loss OR EMCY frame.
Only way to clear it is via enable to high transition

- pin '<driveno>'.control_type
When 0 drive is in position mode (position command into effect)
When 1 drive is in velocity mode (velocity command into effect)

- maxvel
- maxaccel
Macimal velocity / acceleration values
These are NOT implemented yet (no support for SDOs after startup)

- pin '<driveno>'.counts
raw encoder / position from the drive

- param '<driveno>'.position_scale
scale for positioning. position = counts / position-scale

- pin '<driveno>'.position_cmd
position command (for position control)

- pin '<driveno>'.velocity_cmd
velocity command (for velocity control) (not implemented yet)

- pin '<driveno>'.position_fb
position feedback

- pin '<driveno>'.velocity_fb
velocity feedback (not implemented yet)

# Internal CANopen objects

The module uses a set of internal CAN objects in the OD for drive control
The mapping is as follows (rule is moving from 0x6000 to the 0x5000 manufacturer specific area):

* 0x5040 (UInt16) - DS 402 control word array[numdrives] (DS 402 at 0x6040)
* 0x5041 (UInt16) - DS 402 status word array[numdrives] (DS 402 at 0x6041)
* 0x5060 (Int8) - DS 402 modes of operation
* 0x5061 (Int8) - DS 402 modes of operation display


# PDO mapping structure

In order for communication to function properly, PDOs need to be defined on the controllers and the master. At this moment, this is handled via different methods (ConciseDCF for the slaves, default hardcoded PDOs in the master).
However, there is support for using ConciseDCF for configuring the master node too
ConciseDCF can configure ANY object in the object dictionary as long as it's writable and present.

*Special note*
For updating PDOs, the following rules must be observed (per CiA 301)
- Update PDO parameters
  1. Disable the PDO by setting bit 31 to on (PDO COB-ID | 0x80000000) (some drives allow updates without disable in PreOperational)
  2. Update the PDO settings (inhibit time, transmission type, etc)
  3. Re-enable the PDO if that's all (or do this at the end if mapping changes are needed too
- Update PDO mapping
  1. Disable mapping by writing zero as the map count (@ subindex 0x00)
  2. Update the subindexes 0x01 - end with the PDO mapping (UNS32 = 2 bytes index, 1 byte subindex, 1 byte lenght in bits (8/16/32/...)
  3. Write the proper mapping count at subindex 0x00
  4. Re-enable the PDO


