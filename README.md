# ErgoMod
SJSU Computer Engineering Department Capstone Project 

[Github Link](https://github.com/vincebreezy/ergomod)

Project Members: Andrew Chau, Jayson Mercurio, Vince Dionisio

ErgoMod is an ergonomic and mutli-platform controller focused on providing the best feeling of comfort to enhance performance for gamers for long quality play sessions.

*Recommended IDE software to use: [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)*

## Features Implemented
* USB support
* Joysticks
* D-Pad
* Buttons
* Bluetooth
* Rumble

## Setup and Build
Due to flashing a physical microcontroller, this code cannot be run without our hardware.

### Requirements
- ErgoMod PCB
- [ST-Link](https://www.st.com/en/development-tools/st-link-v2.html) or [Nucleo 64 Dev Board](https://www.st.com/en/evaluation-tools/nucleo-wb55rg.html) for flashing. Both are debuggers

### Flashing the Microcontroller
1. Download and install `STM32CubeIDE`
2. Open our project in the IDE
3. `Build` the project (either in debug or release)
4. Ensure debugger (ST-Link or Nucleo 64) is connected to PC
5. Connect the debugger to the debug pins at the top of the ErgoMod PCB
6. Click either `Run` or `Debug` to flash onto microcontroller

Other option for flashing is the [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)

### Variable Viewer
To see debug information and variables of the program flashed onto the microcontroller, you can use [STMStudio](https://www.st.com/en/development-tools/stm-studio-stm32.html). As shown in the demo, you can view joystick values using this program. The following are steps to do so.

1. In `STM32CubeIDE`, open the project
2. In the tabs above, go to `Projects->Properties`
3. Under `C/C++ Build->Settings`, there are three sections with a command field. Change command to `gcc -gdwarf-4` under these sections:  
* `MCU GCC Assembler`
* `MCU GCC Compiler`
* `MCU GCC Linker`
4. Build the project
5. In `STMStudio`, click `Import variables from executable` and find the `.elf` file in the debug folder of the project.
6. Find wanted variables in list and import.
7. Exit menu and right click variables to add to variable viewer
8. Ensure debugger and board are connected and click `Start recording session`
