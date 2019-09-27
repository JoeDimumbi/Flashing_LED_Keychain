# **Lumkani Keychain**

This firmware is intended for use on the keychain hardware.

This repository contains the source code for fun project that blinks the LEDs on a keychain when the button is pressed

Keychain blinking LED project based on Microchip PIC16LF18326

## **Versions**

1. Install MPLAB-X IDE [v5.10](https://www.microchip.com/mplab/mplab-x-ide)
2. Install XC8 v[2.00](https://www.microchip.com/mplab/compilers)

## **Hardware Requirements**

    * PIC16LF18326 
    * PCB
	* Pushbutton
    * 3.3v CR2032 Battery
    * RED LED 0603
    * 17x GREEN LED 0603
    * 20x 1kΩ Resistor
	* 1x 1000PF 0603
	* 2x 0.1UF 0603
	
	
## **Description**

Used 1 PIC16LF18326 for controling the LEDs patterns. There are 1 Button allowing the circuit to be powered from the battery. 
When the button is pressed, the LEDs go on from the outside ring to the inside one.

![Keychain_1](https://github.com/Lumkani/Lumkani_Keychain/blob/master/Lumkani_Keychain_Images/Keychain_1.gif)![Keychain_2](https://github.com/Lumkani/Lumkani_Keychain/blob/master/Lumkani_Keychain_Images/Keychain_2.gif)
