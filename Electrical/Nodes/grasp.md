# Grasp Node

![Grasp Node](https://github.com/OSURoboticsClub/Rover_2017_2018/blob/master/electrical/documents/nodes/files/grasp.JPG)

Designed by [Dylan Thrush](www.dylanthrush.com).

## Summary

Made for MR1718 as part of the OSU Robotics Club.

Controls 4 DC motors with quadrature encoder and current feedback. Has four analog inputs intended to be used with force resistors.

### Bill of Materials

[Bill of Materials](https://docs.google.com/spreadsheets/d/1zSJRJATg0B-pW4RrPtu3-lWw_H9taLzccoVNc3EoKm8/edit?usp=sharing)

#### Design files

[MR1718 GRASP on CircuitMaker](https://workspace.circuitmaker.com/Projects/Details/Dylan-Thrush-2/MR1718-GRASP)

### Downloads

[Schematic V1](files/grasp-v1-schematic.pdf)

[3D Model V1 (STEP)](files/grasp.step)

### Firmware Info

#### Pinouts

| Pin Name | Arduino Pin Number |
| -------- | :----------------: |
| PWM 1.1 | 25 |
| PWM 1.2 | 22 |
| PWM 2.1 | 23 |
| PWM 2.2 | 9 |
| PWM 3.1 | 10 |
| PWM 3.2 | 20 |
| PWM 4.1 | 5 |
| PWM 4.2 | 21 |
| Motor Current 1 | 16/A2 |
| Motor Current 2 | 17/A3 |
| Motor Current 3 | 19/A5 |
| Motor Current 4 | 18/A4 |
| Encoder 1A | 2 |
| Encoder 1B | 30 |
| Encoder 2A | 29 |
| Encoder 2B | 27 |
| Encoder 3A | 28 |
| Encoder 3B | 12 |
| Encoder 4A | 11 |
| Encoder 4B | 13 |
| Force 1 | A14 |
| Force 2 | A13 |
| Force 3 | A12 |
| Force 4 | A1| 
| Red LED | 6 |
| Green LED | 32 |
| Blue LED | 1 |

### Known Issues

#### Version 1
- Missing 3v3 rail connection to VBAT pin on Teensy. Will program without it, but will not boot without it.
- 12V switching supply GND is not connected to board GND.
- Motor controller needs a thermal pad in the footprint, will not work otherwise because thermal pad is connected to GND. Scrape away silkscreen below package before soldering.
- Motors 2-4 are still not working...
- LED needs to be rotated 90 deg CCW for power pin to line up.

#### Version 2
- LSS for motor needs to be a star ground return to motor pad, short LSS to GND for normal operation
