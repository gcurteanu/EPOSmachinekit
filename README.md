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







# DCF file format

The format for initializing slaves is the DCF format, but not sure if a full implementation is required

Standard DCF format is a bit complex for our needs, and also requires some application logic to apply in case of the PDO mappings
(per 301 : disable PDO, set mapping count to zero, load mapping, configure PDO, enable mapping and PDO)

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