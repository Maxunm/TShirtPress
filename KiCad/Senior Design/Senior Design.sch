EESchema Schematic File Version 4
EELAYER 30 0
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
L arduino_micro_shield:ARDUINO_MICRO_SHIELD U1
U 1 1 5FB18976
P 4650 3500
F 0 "U1" V 4600 3500 60  0000 L CNN
F 1 "ARDUINO_MICRO_SHIELD" V 6400 3450 60  0000 L CNN
F 2 "" H 5050 3450 60  0000 C CNN
F 3 "" H 5050 3450 60  0000 C CNN
	1    4650 3500
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x04_Female J1
U 1 1 5FB1CA21
P 9450 1700
F 0 "J1" H 9478 1676 50  0000 L CNN
F 1 "7SegDisplay" H 9478 1585 50  0000 L CNN
F 2 "" H 9450 1700 50  0001 C CNN
F 3 "~" H 9450 1700 50  0001 C CNN
	1    9450 1700
	1    0    0    -1  
$EndComp
$Comp
L Connector:Raspberry_Pi_2_3 J2
U 1 1 5FB1DAFD
P 7250 2400
F 0 "J2" H 7250 3881 50  0000 C CNN
F 1 "Raspberry_Pi_2_3" H 7250 3790 50  0000 C CNN
F 2 "" H 7250 2400 50  0001 C CNN
F 3 "https://www.raspberrypi.org/documentation/hardware/raspberrypi/schematics/rpi_SCH_3bplus_1p0_reduced.pdf" H 7250 2400 50  0001 C CNN
	1    7250 2400
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM7805_TO220 U2
U 1 1 5FB2B0A8
P 5950 4450
F 0 "U2" H 5950 4692 50  0000 C CNN
F 1 "AP1086TL-U" H 5950 4601 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 5950 4675 50  0001 C CIN
F 3 "http://www.fairchildsemi.com/ds/LM/LM7805.pdf" H 5950 4400 50  0001 C CNN
	1    5950 4450
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C1
U 1 1 5FB2C79C
P 5400 4600
F 0 "C1" H 5515 4646 50  0000 L CNN
F 1 "100u" H 5515 4555 50  0000 L CNN
F 2 "" H 5400 4600 50  0001 C CNN
F 3 "~" H 5400 4600 50  0001 C CNN
	1    5400 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C2
U 1 1 5FB2D1FA
P 6450 4600
F 0 "C2" H 6565 4646 50  0000 L CNN
F 1 "100u" H 6565 4555 50  0000 L CNN
F 2 "" H 6450 4600 50  0001 C CNN
F 3 "~" H 6450 4600 50  0001 C CNN
	1    6450 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 4450 5650 4450
Wire Wire Line
	6250 4450 6450 4450
$Comp
L power:GND #PWR02
U 1 1 5FB2EE1E
P 5400 4750
F 0 "#PWR02" H 5400 4500 50  0001 C CNN
F 1 "GND" H 5405 4577 50  0000 C CNN
F 2 "" H 5400 4750 50  0001 C CNN
F 3 "" H 5400 4750 50  0001 C CNN
	1    5400 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5FB2F264
P 6450 4750
F 0 "#PWR04" H 6450 4500 50  0001 C CNN
F 1 "GND" H 6455 4577 50  0000 C CNN
F 2 "" H 6450 4750 50  0001 C CNN
F 3 "" H 6450 4750 50  0001 C CNN
	1    6450 4750
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR01
U 1 1 5FB2F7CF
P 6700 4450
F 0 "#PWR01" H 6700 4300 50  0001 C CNN
F 1 "+5V" H 6715 4623 50  0000 C CNN
F 2 "" H 6700 4450 50  0001 C CNN
F 3 "" H 6700 4450 50  0001 C CNN
	1    6700 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 4450 6700 4450
Connection ~ 6450 4450
$Comp
L power:GND #PWR03
U 1 1 5FB3048B
P 5950 4750
F 0 "#PWR03" H 5950 4500 50  0001 C CNN
F 1 "GND" H 5955 4577 50  0000 C CNN
F 2 "" H 5950 4750 50  0001 C CNN
F 3 "" H 5950 4750 50  0001 C CNN
	1    5950 4750
	1    0    0    -1  
$EndComp
$Comp
L Connector:Barrel_Jack J5
U 1 1 5FB31159
P 7750 4650
F 0 "J5" H 7807 4975 50  0000 C CNN
F 1 "Barrel_Jack" H 7807 4884 50  0000 C CNN
F 2 "" H 7800 4610 50  0001 C CNN
F 3 "~" H 7800 4610 50  0001 C CNN
	1    7750 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 5FB34B15
P 8050 4750
F 0 "#PWR030" H 8050 4500 50  0001 C CNN
F 1 "GND" H 8055 4577 50  0000 C CNN
F 2 "" H 8050 4750 50  0001 C CNN
F 3 "" H 8050 4750 50  0001 C CNN
	1    8050 4750
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR029
U 1 1 5FB3502D
P 8050 4550
F 0 "#PWR029" H 8050 4400 50  0001 C CNN
F 1 "+12V" H 8065 4723 50  0000 C CNN
F 2 "" H 8050 4550 50  0001 C CNN
F 3 "" H 8050 4550 50  0001 C CNN
	1    8050 4550
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR027
U 1 1 5FB3568A
P 5150 4450
F 0 "#PWR027" H 5150 4300 50  0001 C CNN
F 1 "+12V" H 5165 4623 50  0000 C CNN
F 2 "" H 5150 4450 50  0001 C CNN
F 3 "" H 5150 4450 50  0001 C CNN
	1    5150 4450
	1    0    0    -1  
$EndComp
Connection ~ 5400 4450
Wire Wire Line
	5150 4450 5400 4450
$Comp
L Connector:Conn_01x12_Female J3
U 1 1 5FB37FE1
P 9450 2950
F 0 "J3" H 9478 2926 50  0000 L CNN
F 1 "MotorDriverConnection" H 9478 2835 50  0000 L CNN
F 2 "" H 9450 2950 50  0001 C CNN
F 3 "~" H 9450 2950 50  0001 C CNN
	1    9450 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5FB39B2C
P 9250 2550
F 0 "#PWR011" H 9250 2300 50  0001 C CNN
F 1 "GND" V 9255 2422 50  0000 R CNN
F 2 "" H 9250 2550 50  0001 C CNN
F 3 "" H 9250 2550 50  0001 C CNN
	1    9250 2550
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR010
U 1 1 5FB3A3D9
P 9250 2450
F 0 "#PWR010" H 9250 2300 50  0001 C CNN
F 1 "+5V" H 9265 2623 50  0000 C CNN
F 2 "" H 9250 2450 50  0001 C CNN
F 3 "" H 9250 2450 50  0001 C CNN
	1    9250 2450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR07
U 1 1 5FB41108
P 9250 1600
F 0 "#PWR07" H 9250 1450 50  0001 C CNN
F 1 "+5V" H 9265 1773 50  0000 C CNN
F 2 "" H 9250 1600 50  0001 C CNN
F 3 "" H 9250 1600 50  0001 C CNN
	1    9250 1600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5FB4152C
P 9250 1700
F 0 "#PWR08" H 9250 1450 50  0001 C CNN
F 1 "GND" V 9255 1572 50  0000 R CNN
F 2 "" H 9250 1700 50  0001 C CNN
F 3 "" H 9250 1700 50  0001 C CNN
	1    9250 1700
	0    1    1    0   
$EndComp
Text GLabel 4850 3450 2    50   Input ~ 0
MOSI
Text GLabel 4850 3350 2    50   Input ~ 0
SS
Text GLabel 3500 3350 0    50   Input ~ 0
MISO
Text GLabel 3500 3450 0    50   Input ~ 0
mCLK
Text GLabel 9250 2950 0    50   Input ~ 0
MOSI
Text GLabel 9250 2850 0    50   Input ~ 0
MISO
Text GLabel 9250 3050 0    50   Input ~ 0
mCLK
Text GLabel 9250 3150 0    50   Input ~ 0
SS
Text GLabel 4850 2650 2    50   Input ~ 0
DIR
Text GLabel 4850 2550 2    50   Input ~ 0
STEP
Text GLabel 9250 2650 0    50   Input ~ 0
STEP
Text GLabel 9250 2750 0    50   Input ~ 0
DIR
NoConn ~ 9250 3250
NoConn ~ 9250 3350
NoConn ~ 9250 3450
NoConn ~ 9250 3550
Text GLabel 4850 2850 2    50   Input ~ 0
SDA
Text GLabel 4850 2750 2    50   Input ~ 0
SCL
Text GLabel 8050 1900 2    50   Input ~ 0
SCL
Text GLabel 8050 1800 2    50   Input ~ 0
SDA
Text GLabel 9250 1800 0    50   Input ~ 0
SDA
Text GLabel 9250 1900 0    50   Input ~ 0
SCL
$Comp
L power:+5V #PWR05
U 1 1 5FB3261B
P 7050 1100
F 0 "#PWR05" H 7050 950 50  0001 C CNN
F 1 "+5V" H 7065 1273 50  0000 C CNN
F 2 "" H 7050 1100 50  0001 C CNN
F 3 "" H 7050 1100 50  0001 C CNN
	1    7050 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5FB335A1
P 6850 3700
F 0 "#PWR015" H 6850 3450 50  0001 C CNN
F 1 "GND" H 6855 3527 50  0000 C CNN
F 2 "" H 6850 3700 50  0001 C CNN
F 3 "" H 6850 3700 50  0001 C CNN
	1    6850 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5FB33B4A
P 6950 3700
F 0 "#PWR016" H 6950 3450 50  0001 C CNN
F 1 "GND" H 6955 3527 50  0000 C CNN
F 2 "" H 6950 3700 50  0001 C CNN
F 3 "" H 6950 3700 50  0001 C CNN
	1    6950 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5FB33CF1
P 7050 3700
F 0 "#PWR017" H 7050 3450 50  0001 C CNN
F 1 "GND" H 7055 3527 50  0000 C CNN
F 2 "" H 7050 3700 50  0001 C CNN
F 3 "" H 7050 3700 50  0001 C CNN
	1    7050 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 5FB33E6A
P 7150 3700
F 0 "#PWR018" H 7150 3450 50  0001 C CNN
F 1 "GND" H 7155 3527 50  0000 C CNN
F 2 "" H 7150 3700 50  0001 C CNN
F 3 "" H 7150 3700 50  0001 C CNN
	1    7150 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5FB3418B
P 7250 3700
F 0 "#PWR019" H 7250 3450 50  0001 C CNN
F 1 "GND" H 7255 3527 50  0000 C CNN
F 2 "" H 7250 3700 50  0001 C CNN
F 3 "" H 7250 3700 50  0001 C CNN
	1    7250 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5FB343B6
P 7350 3700
F 0 "#PWR020" H 7350 3450 50  0001 C CNN
F 1 "GND" H 7355 3527 50  0000 C CNN
F 2 "" H 7350 3700 50  0001 C CNN
F 3 "" H 7350 3700 50  0001 C CNN
	1    7350 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5FB345B7
P 7450 3700
F 0 "#PWR021" H 7450 3450 50  0001 C CNN
F 1 "GND" H 7455 3527 50  0000 C CNN
F 2 "" H 7450 3700 50  0001 C CNN
F 3 "" H 7450 3700 50  0001 C CNN
	1    7450 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5FB3475E
P 7550 3700
F 0 "#PWR022" H 7550 3450 50  0001 C CNN
F 1 "GND" H 7555 3527 50  0000 C CNN
F 2 "" H 7550 3700 50  0001 C CNN
F 3 "" H 7550 3700 50  0001 C CNN
	1    7550 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5FB348FE
P 3500 3150
F 0 "#PWR014" H 3500 2900 50  0001 C CNN
F 1 "GND" H 3505 2977 50  0000 C CNN
F 2 "" H 3500 3150 50  0001 C CNN
F 3 "" H 3500 3150 50  0001 C CNN
	1    3500 3150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5FB34DEA
P 4850 2950
F 0 "#PWR013" H 4850 2700 50  0001 C CNN
F 1 "GND" H 4855 2777 50  0000 C CNN
F 2 "" H 4850 2950 50  0001 C CNN
F 3 "" H 4850 2950 50  0001 C CNN
	1    4850 2950
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR012
U 1 1 5FB351CF
P 3500 2950
F 0 "#PWR012" H 3500 2800 50  0001 C CNN
F 1 "+5V" H 3515 3123 50  0000 C CNN
F 2 "" H 3500 2950 50  0001 C CNN
F 3 "" H 3500 2950 50  0001 C CNN
	1    3500 2950
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x02_Female J4
U 1 1 5FB36047
P 9450 3850
F 0 "J4" H 9478 3826 50  0000 L CNN
F 1 "StepperPWR" H 9478 3735 50  0000 L CNN
F 2 "" H 9450 3850 50  0001 C CNN
F 3 "~" H 9450 3850 50  0001 C CNN
	1    9450 3850
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR024
U 1 1 5FB36935
P 9250 3850
F 0 "#PWR024" H 9250 3700 50  0001 C CNN
F 1 "+12V" H 9265 4023 50  0000 C CNN
F 2 "" H 9250 3850 50  0001 C CNN
F 3 "" H 9250 3850 50  0001 C CNN
	1    9250 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 5FB36D57
P 9250 3950
F 0 "#PWR025" H 9250 3700 50  0001 C CNN
F 1 "GND" H 9255 3777 50  0000 C CNN
F 2 "" H 9250 3950 50  0001 C CNN
F 3 "" H 9250 3950 50  0001 C CNN
	1    9250 3950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J6
U 1 1 5FB3743D
P 9450 4350
F 0 "J6" H 9478 4326 50  0000 L CNN
F 1 "5V" H 9478 4235 50  0000 L CNN
F 2 "" H 9450 4350 50  0001 C CNN
F 3 "~" H 9450 4350 50  0001 C CNN
	1    9450 4350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR026
U 1 1 5FB37718
P 9250 4350
F 0 "#PWR026" H 9250 4200 50  0001 C CNN
F 1 "+5V" H 9265 4523 50  0000 C CNN
F 2 "" H 9250 4350 50  0001 C CNN
F 3 "" H 9250 4350 50  0001 C CNN
	1    9250 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR028
U 1 1 5FB37D4B
P 9250 4450
F 0 "#PWR028" H 9250 4200 50  0001 C CNN
F 1 "GND" H 9255 4277 50  0000 C CNN
F 2 "" H 9250 4450 50  0001 C CNN
F 3 "" H 9250 4450 50  0001 C CNN
	1    9250 4450
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5FB39278
P 6000 2600
F 0 "SW1" H 6000 2885 50  0000 C CNN
F 1 "SW_Push" H 6000 2794 50  0000 C CNN
F 2 "" H 6000 2800 50  0001 C CNN
F 3 "~" H 6000 2800 50  0001 C CNN
	1    6000 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 2600 6450 2600
$Comp
L Device:R_Small R1
U 1 1 5FB3ADF9
P 5550 2400
F 0 "R1" H 5609 2446 50  0000 L CNN
F 1 "220" H 5609 2355 50  0000 L CNN
F 2 "" H 5550 2400 50  0001 C CNN
F 3 "~" H 5550 2400 50  0001 C CNN
	1    5550 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 2600 5550 2600
Wire Wire Line
	5550 2600 5550 2500
$Comp
L power:+3.3V #PWR06
U 1 1 5FB3CCA9
P 7350 1100
F 0 "#PWR06" H 7350 950 50  0001 C CNN
F 1 "+3.3V" H 7365 1273 50  0000 C CNN
F 2 "" H 7350 1100 50  0001 C CNN
F 3 "" H 7350 1100 50  0001 C CNN
	1    7350 1100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR09
U 1 1 5FB3D6B4
P 5550 2300
F 0 "#PWR09" H 5550 2150 50  0001 C CNN
F 1 "+3.3V" H 5565 2473 50  0000 C CNN
F 2 "" H 5550 2300 50  0001 C CNN
F 3 "" H 5550 2300 50  0001 C CNN
	1    5550 2300
	1    0    0    -1  
$EndComp
$Comp
L MCU_Microchip_PIC12:PIC12F1822-IP U3
U 1 1 5FB4A2FB
P 2550 4400
F 0 "U3" H 2650 3850 50  0000 C CNN
F 1 "PIC12F1822-IP" H 2900 3950 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 3150 5050 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/41413B.pdf" H 2550 4400 50  0001 C CNN
	1    2550 4400
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR023
U 1 1 5FB4B746
P 2550 3800
F 0 "#PWR023" H 2550 3650 50  0001 C CNN
F 1 "+5V" H 2565 3973 50  0000 C CNN
F 2 "" H 2550 3800 50  0001 C CNN
F 3 "" H 2550 3800 50  0001 C CNN
	1    2550 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 5FB4BB31
P 2550 5000
F 0 "#PWR031" H 2550 4750 50  0001 C CNN
F 1 "GND" H 2555 4827 50  0000 C CNN
F 2 "" H 2550 5000 50  0001 C CNN
F 3 "" H 2550 5000 50  0001 C CNN
	1    2550 5000
	1    0    0    -1  
$EndComp
Text GLabel 3150 4500 2    50   Input ~ 0
A
Text GLabel 3150 4300 2    50   Input ~ 0
Z
$EndSCHEMATC
