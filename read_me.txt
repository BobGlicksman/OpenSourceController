The Open Source Controller (Controller) is a software programmable controller that has been designed to control 
temperature based processes, using Maxim DS18B20 temperature sensors.  The Controller is based on the Arduino Uno 
microcontroller board (http://arduino.cc/en/Main/ArduinoBoardUno).  The Controller is supplied with a test software package only.
The test software is useful for testing the assembled Controller as well as for use as a template for custom software applications.  
One such software application is the "Feedstock Processing Controller" which is documented in its own repository on this site.

NOTE: This project requires Arduino IDE version 022.  The included libraries do not work with Arduino 1.x!

This repository contains files for the Open Source Controller, release package 1.1.
The repository contains the following folders and their contents:

- Documentation:  
--  Open Source Controller Manual release 1.1: pdf version of the user manual with instructions to build and test the Controller.
--  Instructions for Ordering the Shield Board: step by step instructions for placing an order for the Shield Board, which is 
	a small, custom designed printed circuit board that is part of the Controller's electronics module.

- Hardware:  
-- ControllerCaseLayout_to scale: pdf file containing the layout for the controller cover at 1:1 scale when printed.
-- Shield Board Eagle Files:  folder containing the Eagle CAD files for the Shield Board.
-- Still Contoller Parts List - unit and vendor sorted:  Excel file with the complete parts list, sorted by unit and by vendor.

- Libraries:
-- DallasTemperature:  folder to copy into the Arduino022 "libraries" folder
-- LiquidCrystal_I2C: folder to copy into the Arduino022 "libraries" folder
-- LiquidTWI: folder to copy into the Arduino022 "libraries" folder
-- OneWire: folder to copy into the Arduino022 "libraries" folder

- Software:
-- still_control_test_11: folder to copy to the Arduino022 "sketch" folder.

- Drivers:
-- Arduino_UNO_REV3.inf:  Driver for the Uno R3 board, from the Arduino Release 1.0 package.  Copy to the Arduino022 "Drivers" folder.


Complete instructions for using these files and folders are in the Open Source Controller Manual document.

The Open Source Controller, including all items in this archive, is released under a Creative Commons license Attribution-ShareAlike 3.0.
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/ or send a letter to Creative Commons, 
444 Castro Street, Suite 900, Mountain View, California, 94041, USA.

The Open Source Controller is intended for hobbyist applications and for “do it yourself” assembly and use.  
The user assumes all responsibility and liability for any damages, injuries or consequences resulting from the construction, 
implementation and use of this product or any part thereof.

Copyright © 2012 Bob Glicksman, Curbie. All rights reserved.

The material contained within this docuement and its archive may not be copied, duplicated, or reproduced in whole or in part using any method, 
for any reason, or under any circumstances without expressed written permission by the authors. Individuals may download and 
keep one unaltered copy for non-profit, personal use of the contents.
