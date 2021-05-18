# EPFL Rocket Team - <em>Bella Lui GSE Software</em>

## Table of Contents
1. [Abstract](#abstract)
2. [File Structure](#file-structure)
3. [Thread Architecture](#thread-architecture)
4. [Building the Software](#building-the-software)
5. [Flashing the Hostboards](#flashing-the-hostboards)
6. [Miscellaneous](#miscellaneous)

## Abstract
### Project purpose 
The EPFL Rocket Team association will be participating in the 10’000 feet 
SRAD Hybrid Engine category of the 2021 Virtual Spaceport America Cup, which will be held in June 2021. 
For more information, visit https://epflrocketteam.ch/ :rocket:

### Description
This repository contains the software of the <b>Ground Segment subsystem</b> dedicated to the <b>Ground Support Equipment</b>, or <b>GSE</b>.\
The main goal of this software is to run the operations in the Launch Area. Which, for safety reasons, are all done remotely from the <b>Ground Station</b>.\
To acheieve this, this software comports different features, mainly dedicated to the control of actuators, the aquisition of sensors, safety features, as well as the critical management of the radio communication with other subsystems.\
This piece of software is meant to be run on EPFL Rocket Teams "Hostboards", our custom PCBs housing a STM32F446RET6 microcontroller. And for a completely functional setup, 3 Hostboards need to be used, all of them connected to each other through a CAN bus.\
This software is being developped in C on STM32CubeIDE, using FreeRTOS, the CMSIS abstraction layer, and with heavy inpiration from the 2020 AV Code located at: https://github.com/EPFLRocketTeam/BellaLui for many of the core functionnality.\
For the Radio communication, this is established using XBee RF modems available on external custom PCBs from the team (868 MHz in CH and 915 MHz in USA).

## File Structure
```
BellaLuiGSE
│   README.md
└───Application
│   └───Src                     User-defined source files
│   │   └───debug               Contains all the debugging source files
│   │   └───GSE                 Contains all GSE Specific source files
│   │   └───telemetry           Contains all the Telemetry related source files
│   │   can_reception.c         All functions related to receiving messages on the CAN bus
│   │   can_transmission.c      All functions related to transmitting messages on the CAN bus
│   │   sync.c                  Synchronisation logic used by some debugging functions
│   │   thread_init.c           Thread initialisations
│   |
│   └───Inc                     User-defined header files
│   │   └───debug               Contains all the debugging header files
│   │   └───GSE                 Contains all GSE Specific header files
│   │   └───misc                Contains common enum/structure definitions
│   │   └───telemetry           Contains all the Telemetry related header files
│   │   can_reception.h         All defintions related to receiving messages on the CAN bus
│   │   can_transmission.h      All definitions to transmitting messages on the CAN bus
│   │   sync.h                  Synchronisation logic used by some debugging functions
│   │   thread_init.h           Thread initialisations, choice of Hostboard code to flash
│
└───Core                        Core functionnality
│   └───Src                     Core source files
│   │   main.c                  Main file, program starting point
│   │   other system source files
│   |
│   └───Inc                     Core header files
│   │   main.h                  Main file header
|   |   FreeRTOSConfig.h        FreeRTOS main config file
│   │   other system header files
│
└───Drivers
│   └───CMSIS                   CMSIS HAL Core
│   └───STM32F4xx_HAL_Driver    CMSIS HAL Drivers
│
└───Middlewares
│   └───Third_Party             Contains all Thrid Party software used
│   │   └───FreeRTOS            FreeRTOS source code
|
|   BellaLuiGSE.ioc             STM32CubeIDE main configuration file
|   other auto-generated files
```

## Thread Architecture
This software is heavily using the multi-threading functionnality of FreeRTOS, several threads have thus been implemented to manage all required functionnalities of the GSE.

<!-- INSERT PICTURE THREADS -->

And as mentionned also above, a complete setup using this software requires 3 Hostboards to be complete.\
Each of these need to be flashed by a different version of the software, all of them containing different running threads when running.\
This thread definitions are pre-configured in the threads_init.h file, the main Hostboard define only needs to be properly selected to select the part of the code for each Hostboard.

## Building the Software

<!-- TO FILL -->

## Flashing the Hostboards

<!-- TO FILL -->

## Miscellaneous

### Authors
* [Lucas Pallez](https://www.linkedin.com/in/lucas-pallez-37b47b1a4/)

### Acknowledgments
* [ERT Avionics Software](https://github.com/EPFLRocketTeam/BellaLui)
* [EPFL Rocket Team](https://epflrocketteam.ch/fr/)

### Useful links
<!-- TO FILL -->
