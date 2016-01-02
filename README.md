# EPOSmachinekit
CANopen EPOS controller for Machinekit
...

Main goal for this project:

- Develop a Machinekit/LinuxCNC component able to control CANopen Maxon EPOS positioning controllers (or with minimal changes other DS-402 drives)

Component should be able to control multiple drives, and do so with a simple configuration


Targets:
- create the required CanFestival infrastrucuture to support the CANopen operations
- support for the DS-302 boot process and automatic configuration updates for the CANopen devices via concise DCF (0x1F22).
The CanFestival existing DCF seems to be lacking a lot of required features
- support for the DS-402 state machine allowing the drive operation
- support for profile position / speed as well as direct position / velocity modes as a minimum.
Probably the profile modes will not be useful directly and we'll end up using the built in profile generator in Machinekit/LinuxCNC



Current status:
- initial (messy) implementation done to learn the CanFestival stack capabilities and modes to control the drive. Along with constraints due to the RT drivers.
It is functional, and able to position the drive and get position data in realtime with minimal external coding.
Moves from external application (MK/LinuxCNC) should be done by simply working to the values in the OD, the TPDOs/RPDOs taking over independently from there.
- initial (messy) implementation of the DS-302 code, moving to a more standard state machine setup (ongoing). Still lacking the DCF upload
- initial (very messy) DS-402 state machine support (just goes to Operation Enabled if able and keeps it there)


Hardware:
- Peak PCI dual port (opto isolated) CAN card
- Xenomai Machinekit kernel
- Using the RT-CAN socket interfaces (not the regular ones) included in the kernel
- No need for the Peak Linux driver. It is not used / useful from what can be seen
- The RT-CAN in the kernel seems stable, but can not cope with Linux interrupt sharing when used in the RT model. Peak card MUST NOT share it's IRQ with other devices... Or else kernel panics.

As far as positioning controller, Maxon EPOS 70/10. Looks like a very standard implementation of DS 402 is used, so if it works for this, it will work for others as well
testing the CAN with a 25mtr CAN cable with 120ohm termination at both ends, point to point


# DCF file format

The format for initializing slaves is the DCF format, but not sure if a full implementation is required

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

Our format should directly describe the steps and should be a 1:1 mapping to SDO actions. One problem to cope with is describing the data size of the objects

Proposed format:
[slave id]
Index.Subindex = Size Value
...

slave id = decimal
index / subindex = HEX without specifier?
size = decimal
value = either hex or decimal notation.

Example:
[1]
1400.1 = 32 0x10000000
...

The standard DCF-like format might come, but since we're now doing Concise DCF the file format is up to us

For initial testing, we will use the existing DCF example code from CanFestival




# Machinekit/LinuxCNC interface

- pin enable
The pin enables / disables the drive.
When enable high, drive seeks to get to the enabled state, clearing all the drive errors in the process
When enable low, drive does a full stop via quickstop

- pin enabled
The drives are enabled

- pin faulted
At least one of the drives is faulted

- control_type
When 0 drive is in position mode (position command into effect)
When 1 drive is in velocity mode (velocity command into effect)

- maxvel
- maxaccel
Macimal velocity / acceleration values

- counts
raw encoder / position from the drive

- position_scale
scale for positioning. position = counts / position-scale

- position_cmd
position command (for position control)

- velocity_cmd
velocity command (for velocity control)

- position_fb
position feedback

- velocity_fb
velocity feedback

The drive instance pins are arrays[]
enable / fault are general pins, a fault on one will fault the component. enable will enable/disable all


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


