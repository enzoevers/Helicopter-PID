EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
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
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L R R?
U 1 1 5A36DF46
P 4200 1800
F 0 "R?" V 4280 1800 50  0000 C CNN
F 1 "680" V 4200 1800 50  0000 C CNN
F 2 "" V 4130 1800 50  0001 C CNN
F 3 "" H 4200 1800 50  0001 C CNN
	1    4200 1800
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A36DFAF
P 4750 1800
F 0 "R?" V 4830 1800 50  0000 C CNN
F 1 "680" V 4750 1800 50  0000 C CNN
F 2 "" V 4680 1800 50  0001 C CNN
F 3 "" H 4750 1800 50  0001 C CNN
	1    4750 1800
	1    0    0    -1  
$EndComp
$Comp
L S8050 S8050
U 1 1 5A36DFD6
P 4100 2850
F 0 "S8050" H 4300 2900 50  0000 L CNN
F 1 "Q_NPN_BCE" H 4300 2800 50  0000 L CNN
F 2 "S8050" H 4300 2950 50  0001 C CNN
F 3 "" H 4100 2850 50  0001 C CNN
	1    4100 2850
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_BCE S8050
U 1 1 5A36E029
P 4650 2550
F 0 "S8050" H 4850 2600 50  0000 L CNN
F 1 "Q_NPN_BCE" H 4850 2500 50  0000 L CNN
F 2 "" H 4850 2650 50  0001 C CNN
F 3 "" H 4650 2550 50  0001 C CNN
	1    4650 2550
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_BCE S8050
U 1 1 5A36E054
P 5500 2150
F 0 "S8050" H 5700 2200 50  0000 L CNN
F 1 "Q_NPN_BCE" H 5700 2100 50  0000 L CNN
F 2 "" H 5700 2250 50  0001 C CNN
F 3 "" H 5500 2150 50  0001 C CNN
	1    5500 2150
	1    0    0    -1  
$EndComp
$Comp
L Q_PNP_BCE S8550
U 1 1 5A36E097
P 5500 2850
F 0 "S8550" H 5700 2900 50  0000 L CNN
F 1 "Q_PNP_BCE" H 5700 2800 50  0000 L CNN
F 2 "" H 5700 2950 50  0001 C CNN
F 3 "" H 5500 2850 50  0001 C CNN
	1    5500 2850
	1    0    0    -1  
$EndComp
$Comp
L +9V #PWR?
U 1 1 5A36E118
P 4200 1600
F 0 "#PWR?" H 4200 1450 50  0001 C CNN
F 1 "+9V" H 4200 1740 50  0000 C CNN
F 2 "" H 4200 1600 50  0001 C CNN
F 3 "" H 4200 1600 50  0001 C CNN
	1    4200 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A36E138
P 4200 3150
F 0 "#PWR?" H 4200 2900 50  0001 C CNN
F 1 "GND" H 4200 3000 50  0000 C CNN
F 2 "" H 4200 3150 50  0001 C CNN
F 3 "" H 4200 3150 50  0001 C CNN
	1    4200 3150
	1    0    0    -1  
$EndComp
$Comp
L +9V #PWR?
U 1 1 5A36E1B1
P 4750 1600
F 0 "#PWR?" H 4750 1450 50  0001 C CNN
F 1 "+9V" H 4750 1740 50  0000 C CNN
F 2 "" H 4750 1600 50  0001 C CNN
F 3 "" H 4750 1600 50  0001 C CNN
	1    4750 1600
	1    0    0    -1  
$EndComp
$Comp
L +9V #PWR?
U 1 1 5A36E744
P 5600 1600
F 0 "#PWR?" H 5600 1450 50  0001 C CNN
F 1 "+9V" H 5600 1740 50  0000 C CNN
F 2 "" H 5600 1600 50  0001 C CNN
F 3 "" H 5600 1600 50  0001 C CNN
	1    5600 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A36E9FC
P 5600 3150
F 0 "#PWR?" H 5600 2900 50  0001 C CNN
F 1 "GND" H 5600 3000 50  0000 C CNN
F 2 "" H 5600 3150 50  0001 C CNN
F 3 "" H 5600 3150 50  0001 C CNN
	1    5600 3150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A36EA19
P 4750 3150
F 0 "#PWR?" H 4750 2900 50  0001 C CNN
F 1 "GND" H 4750 3000 50  0000 C CNN
F 2 "" H 4750 3150 50  0001 C CNN
F 3 "" H 4750 3150 50  0001 C CNN
	1    4750 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 1950 4200 2550
Wire Wire Line
	4200 2550 4200 2650
Wire Wire Line
	4200 1650 4200 1600
Wire Wire Line
	4750 1650 4750 1600
Wire Wire Line
	4200 2550 4450 2550
Connection ~ 4200 2550
Wire Wire Line
	5600 2350 5600 2500
Wire Wire Line
	5600 2500 5600 2650
Wire Wire Line
	5300 2150 5300 2850
Wire Wire Line
	4750 2150 5300 2150
Wire Wire Line
	4750 1950 4750 2150
Wire Wire Line
	4750 2150 4750 2350
Wire Wire Line
	4200 3150 4200 3050
Wire Wire Line
	4750 3150 4750 2750
Wire Wire Line
	5600 3150 5600 3050
Wire Wire Line
	5600 1950 5600 1600
Connection ~ 4750 2150
Connection ~ 5300 2150
Connection ~ 5300 2850
Connection ~ 5600 2650
Connection ~ 5600 2350
Connection ~ 5600 1950
Connection ~ 5600 3050
Connection ~ 4750 2750
Connection ~ 4200 3050
Connection ~ 4750 1600
Connection ~ 4750 1650
Connection ~ 4200 1600
Connection ~ 4200 1650
Connection ~ 4750 1950
Connection ~ 4200 1950
Connection ~ 5600 1600
Text Label 3100 2850 0    60   ~ 0
5V_Arduino_PWM
Wire Wire Line
	3100 2850 3900 2850
Connection ~ 3900 2850
Text Label 6250 2500 2    60   ~ 0
Output_PWM
Wire Wire Line
	5600 2500 6250 2500
Connection ~ 5600 2500
$EndSCHEMATC