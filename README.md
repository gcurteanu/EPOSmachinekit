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
