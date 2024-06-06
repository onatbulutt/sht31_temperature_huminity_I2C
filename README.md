# sht31_temperature_huminity_I2C
 This project involves reading temperature and humidity data from a sensor using I2C communication and displaying the values on a 7-segment display using an STM32 microcontroller.

# Overview
This project reads temperature and humidity data from a sensor via I2C and displays the results on a 7-segment display. It is designed for use with STM32 microcontrollers and utilizes the HAL (Hardware Abstraction Layer) library.

# Features
I2C communication with a temperature and humidity sensor.
Display of temperature and humidity on a 7-segment display.
Simple and modular code structure.

# Hardware Requirements
STM32 microcontroller (e.g., STM32F103)
Temperature and humidity sensor (SHT31)
7-segment display
Breadboard and jumper wires for prototyping

# Software Requirements
STM32CubeIDE or any compatible IDE for STM32 development

# Installation
Download project as zip.
Extract the zip.

# Open the Project
Open STM32CubeIDE.
Select File > Open Projects from File System....
Navigate to the cloned repository folder and click Finish.

# Build the Project
Click on the project in the Project Explorer.
Select Project > Build All.

# Upload to STM32
Connect your STM32 microcontroller to your computer using a USB cable.
Click the Run button in STM32CubeIDE to upload the binary file to your STM32 microcontroller using a compatible programmer/debugger (ST-Link).

# Usage
## Connect the Hardware
Connect the I2C sensor to the STM32 I2C pins.
Connect the 7-segment display to the appropriate GPIO pins.
## Power the System
Power up the STM32 and ensure all connections are correct.
## Monitor the Display
The temperature and humidity values will be displayed on the 7-segment display.
