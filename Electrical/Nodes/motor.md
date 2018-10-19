# Motor Node

<!-- ![Iris](files/iris.jpg) -->

![Iris Render](files/motor.jpg)

Designed by [Nick McComb](www.nickmccomb.net) for OSURC Mars Rover.


## Summary

Made for MR1718 as part of the OSU Robotics Club.


This board drives one of the Mars Rover’s wheels. It’s designed to be a breakout board for one of Pololu’s 21A motor driver boards, allowing it to be controlled over RS485 from the IRIS Node.

Note, if you’re using the design, the traces going to the motor driver are only rated for 10A continuous, so be careful with the actual amount of current that you run through the motors. If you need to, bodge a wire in parallel with the XT30 connector to add more current capability. This design limitation was forced because of the space constraint that this board was designed into.

This design is completely open-source, the design files can be found at CircuitMaker, see the link below.

### Bill of Materials

[Bill of Materials V2](https://docs.google.com/spreadsheets/d/1zQtJtcauIfV7cGKvJbZdHLx3p09I3BvlQvNzqRRZR7c/edit?usp=sharing)

[Bill of Materials V1](
https://docs.google.com/spreadsheets/d/1CobSEg-5mzBy_F1_ASbbnYLLLra0shLwDUG4rKD09mE/edit?usp=sharing
)

#### Design files

[MR1718 Motor Node on CircuitMaker](https://workspace.circuitmaker.com/Projects/Details/Nick-McComb/MR1718-Motor-Node)

### Downloads

[Schematic V2](files/motor-v2-schematic.pdf)

[Schematic V1](files/motor-v1-schematic.pdf)

[3D Model V1 (STEP)](files/motor.step)

### Known Issues

#### Version 1
- Switching regulator that was purchased is not compatible with the design. A pin-compatible new one was specc'd and ordered, it is included in the BOM as "alternative" parts. Only U1, L1, C2, and R1 are changed. An 1N4148 diode also needs to be added per LT1933's datasheet. See below schematic:
<img src="files/motor_v1_bodge_1.jpg" width="600px">
- Missing 3v3 rail connection to VBAT on Teensy. Will program without it, but will not boot without it. Connect VBat (pin 21) to the 3v3 side of C12 (closest to the XTAL)
- MCU mistakenly connected to the 5v_SYS, needs to be connected to the 3v3 net (as there is no 5v net on this board)
- RGB led missing 3v3 anode connection. Currently connected to non-present 5V_SYS net.

#### Version 2 
- R1 and R2 are labeled backwards in the schematic, R1 should be 14k and R2 should be 8.45k (the math is 1.245(1+14000/8450))
