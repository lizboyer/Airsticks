# Airsticks-Advanced-Digital-Systems-Lab-Project
Airsticks was a project by myself (Elizabeth Boyer), Christian Tchilikov, and Dearborn Plys. It uses accelerometers, a speaker, an STM32 MCU, and a board we created. The user puts the velcro cuffs onto a pair of drum sticks and - based off of the orientation and acceleration of the sticks- can play through drumset samples.


# How To Use:
The project can be cloned onto the users local computer by running:
 “git clone https://github.com/lizboyer/Airsticks-Advanced-Digital-Systems-Lab-Project.git [name of new directory]” 
The “main” branch contains a directory called “10-6-sine,” which should be run via STM32CubeIDE on the STM32F407 Discovery Kit to playback the audio files of the drum noises.
The other directory, “firmware_with_directions_sound” should be run on our PCB.

If a serial device is used to read directional output, Picocom or Putty can be used to print those outputs via UART, at a baud rate of 19200. 
