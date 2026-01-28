# Sharkbyte Debugger Board

## Purpose of Project

The purpose of this project is to develop a budget-friendly electronics debugging station that includes various PCBs (Printed Circuit Boards) that incorporate different functionalities. 
A self-designed main board will be used to handle the data processing to display the outputs of the various peripheral boards. The peripheral boards will each have their own functionality and use such 
as an oscilloscope, multimeter, function generator, logic analyzer, variable DC power supply, and CAN Bus debugger. Each board will have a data connector utilizing various GPIO and communication protocols such as
**UART, SPI, and PSSI** as well as a power connector that is handled by the power management system on the main board. 

## Main Board

The central board that all peripheral boards connects to and handles data processing pipeline to a computer. This board has a power management system uses a central 5V rail to provide to a 3.3V LDO for microcontroller logic, and +-12V for operational amplifiers. 

# Notes for Board Redesign

Make sure boot capacitor for 5V is before the inductor
