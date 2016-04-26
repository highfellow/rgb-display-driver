EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:controls-components
LIBS:controls-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Controls Daughter board for RGB LED Controller"
Date "19 apr 2015"
Rev ""
Comp "highfellow.org"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONTROLS_HEADER J1
U 1 1 5529C30F
P 4700 3850
F 0 "J1" H 4700 3400 60  0000 C CNN
F 1 "CONTROLS_HEADER" H 4700 4500 60  0000 C CNN
F 2 "" H 4650 3850 60  0000 C CNN
F 3 "" H 4650 3850 60  0000 C CNN
	1    4700 3850
	-1   0    0    -1  
$EndComp
$Comp
L ENCODER J2
U 1 1 5529C488
P 6650 3550
F 0 "J2" H 6600 3200 60  0000 C CNN
F 1 "ENCODER" H 6600 3900 60  0000 C CNN
F 2 "" H 6450 3450 60  0000 C CNN
F 3 "" H 6450 3450 60  0000 C CNN
	1    6650 3550
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW2
U 1 1 5529C4A4
P 6100 4150
F 0 "SW2" H 6250 4260 50  0000 C CNN
F 1 "MODE BUTTON" H 6100 4070 50  0000 C CNN
F 2 "~" H 6100 4150 60  0000 C CNN
F 3 "~" H 6100 4150 60  0000 C CNN
	1    6100 4150
	0    -1   -1   0   
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 5529C4B3
P 5700 4300
F 0 "SW1" H 5850 4410 50  0000 C CNN
F 1 "PARAM BUTTON" H 5700 4220 50  0000 C CNN
F 2 "~" H 5700 4300 60  0000 C CNN
F 3 "~" H 5700 4300 60  0000 C CNN
	1    5700 4300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6100 3400 5200 3400
Wire Wire Line
	5350 3400 5350 4600
Wire Wire Line
	5350 4600 6100 4600
Connection ~ 5350 3400
Wire Wire Line
	6100 4600 6100 4450
Connection ~ 5700 4600
Wire Wire Line
	5700 4000 5200 4000
Wire Wire Line
	6100 3850 5200 3850
Wire Wire Line
	5200 3700 6100 3700
Wire Wire Line
	6100 3550 5200 3550
$EndSCHEMATC
