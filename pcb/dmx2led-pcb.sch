EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
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
L MCU_Microchip_SAMD:ATSAMD21G18A-A U2
U 1 1 5FDD0125
P 8850 3600
F 0 "U2" H 8300 1750 50  0000 C CNN
F 1 "ATSAMD21G18A-A" H 9400 1700 50  0000 C CNN
F 2 "Package_QFP:TQFP-48_7x7mm_P0.5mm" H 9750 1750 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/SAM_D21_DA1_Family_Data%20Sheet_DS40001882E.pdf" H 8850 4600 50  0001 C CNN
F 4 "ATSAMD21G18A-AU-ND" H 8850 3600 50  0001 C CNN "DIGI_PN"
	1    8850 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 4400 7600 4400
Wire Wire Line
	8150 4500 7600 4500
Text Label 7600 4400 0    50   ~ 0
USB_DM
Text Label 7600 4500 0    50   ~ 0
USB_DP
Wire Wire Line
	8150 5100 7600 5100
Text Label 7600 5000 0    50   ~ 0
SWCLK
Text Label 7600 5100 0    50   ~ 0
SWDIO
Wire Wire Line
	8150 5300 7600 5300
Text Label 7600 5300 0    50   ~ 0
NRST
Wire Wire Line
	9550 4200 9800 4200
Text Label 9800 4200 2    50   ~ 0
TXD
Text Label 9800 4300 2    50   ~ 0
RXD
$Comp
L Connector:TestPoint TP1
U 1 1 5FDE6561
P 9800 4200
F 0 "TP1" V 9800 4388 50  0000 L CNN
F 1 "TestPoint" V 9845 4388 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 10000 4200 50  0001 C CNN
F 3 "~" H 10000 4200 50  0001 C CNN
	1    9800 4200
	0    1    1    0   
$EndComp
Wire Wire Line
	9550 4300 9800 4300
$Comp
L Connector:TestPoint TP2
U 1 1 5FDE7321
P 9800 4300
F 0 "TP2" V 9800 4488 50  0000 L CNN
F 1 "TestPoint" V 9845 4488 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 10000 4300 50  0001 C CNN
F 3 "~" H 10000 4300 50  0001 C CNN
	1    9800 4300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5FDEA1CE
P 8850 5700
F 0 "#PWR0101" H 8850 5450 50  0001 C CNN
F 1 "GND" H 8855 5527 50  0000 C CNN
F 2 "" H 8850 5700 50  0001 C CNN
F 3 "" H 8850 5700 50  0001 C CNN
	1    8850 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 5700 8850 5600
Wire Wire Line
	8950 5500 8950 5600
Wire Wire Line
	8950 5600 8850 5600
Connection ~ 8850 5600
Wire Wire Line
	8850 5600 8850 5500
$Comp
L Device:C_Small C8
U 1 1 5FDECF1B
P 8350 1600
F 0 "C8" V 8121 1600 50  0000 C CNN
F 1 "1uF" V 8212 1600 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 8350 1600 50  0001 C CNN
F 3 "~" H 8350 1600 50  0001 C CNN
F 4 "1276-6524-2-ND" H 8350 1600 50  0001 C CNN "DIGI_PN"
	1    8350 1600
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5FDEE8F1
P 6600 2350
F 0 "C6" H 6692 2396 50  0000 L CNN
F 1 "22pF" H 6692 2305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6600 2350 50  0001 C CNN
F 3 "~" H 6600 2350 50  0001 C CNN
F 4 "445-1239-1-ND" H 6600 2350 50  0001 C CNN "DIGI_PN"
	1    6600 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5FDEF2C4
P 6950 2350
F 0 "C7" H 7042 2396 50  0000 L CNN
F 1 "22pF" H 7042 2305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6950 2350 50  0001 C CNN
F 3 "~" H 6950 2350 50  0001 C CNN
F 4 "445-1239-1-ND" H 6950 2350 50  0001 C CNN "DIGI_PN"
	1    6950 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5FDF5C39
P 8100 1700
F 0 "#PWR0102" H 8100 1450 50  0001 C CNN
F 1 "GND" H 8105 1527 50  0000 C CNN
F 2 "" H 8100 1700 50  0001 C CNN
F 3 "" H 8100 1700 50  0001 C CNN
	1    8100 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 1700 8750 1450
Wire Wire Line
	8750 1450 8850 1450
Wire Wire Line
	9050 1450 9050 1300
Wire Wire Line
	8850 1700 8850 1450
Connection ~ 8850 1450
Wire Wire Line
	8850 1450 8950 1450
Wire Wire Line
	8950 1700 8950 1450
Connection ~ 8950 1450
Wire Wire Line
	8950 1450 9050 1450
Wire Wire Line
	9050 1450 9150 1450
Wire Wire Line
	9150 1450 9150 1700
Connection ~ 9050 1450
$Comp
L power:+5V #PWR0103
U 1 1 5FE0ABA4
P 3200 3000
F 0 "#PWR0103" H 3200 2850 50  0001 C CNN
F 1 "+5V" H 3215 3173 50  0000 C CNN
F 2 "" H 3200 3000 50  0001 C CNN
F 3 "" H 3200 3000 50  0001 C CNN
	1    3200 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 3150 3200 3150
Wire Wire Line
	3200 3150 3200 3000
Wire Wire Line
	2600 3350 3200 3350
Wire Wire Line
	2600 3450 3200 3450
Text Label 3200 3350 2    50   ~ 0
USB_DP
Text Label 3200 3450 2    50   ~ 0
USB_DM
$Comp
L power:GND #PWR0104
U 1 1 5FE1264C
P 2300 3950
F 0 "#PWR0104" H 2300 3700 50  0001 C CNN
F 1 "GND" H 2305 3777 50  0000 C CNN
F 2 "" H 2300 3950 50  0001 C CNN
F 3 "" H 2300 3950 50  0001 C CNN
	1    2300 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 3750 2200 3850
Wire Wire Line
	2200 3850 2300 3850
Wire Wire Line
	2300 3850 2300 3950
Wire Wire Line
	2300 3750 2300 3850
Connection ~ 2300 3850
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J6
U 1 1 5FE177EB
P 5050 4150
F 0 "J6" H 5100 4467 50  0000 C CNN
F 1 "TAG CONNECT" H 5100 4376 50  0000 C CNN
F 2 "Connector:Tag-Connect_TC2030-IDC-NL_2x03_P1.27mm_Vertical" H 5050 4150 50  0001 C CNN
F 3 "~" H 5050 4150 50  0001 C CNN
	1    5050 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 4050 4700 4050
Wire Wire Line
	4700 4050 4700 3850
Wire Wire Line
	4850 4150 4500 4150
Wire Wire Line
	4850 4250 4700 4250
Wire Wire Line
	4700 4250 4700 4450
Wire Wire Line
	5350 4150 5800 4150
$Comp
L power:GND #PWR0105
U 1 1 5FE1F81B
P 4700 4450
F 0 "#PWR0105" H 4700 4200 50  0001 C CNN
F 1 "GND" H 4705 4277 50  0000 C CNN
F 2 "" H 4700 4450 50  0001 C CNN
F 3 "" H 4700 4450 50  0001 C CNN
	1    4700 4450
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0106
U 1 1 5FE22063
P 4700 3850
F 0 "#PWR0106" H 4700 3700 50  0001 C CNN
F 1 "+3V3" H 4715 4023 50  0000 C CNN
F 2 "" H 4700 3850 50  0001 C CNN
F 3 "" H 4700 3850 50  0001 C CNN
	1    4700 3850
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0107
U 1 1 5FE23928
P 9050 1300
F 0 "#PWR0107" H 9050 1150 50  0001 C CNN
F 1 "+3V3" H 9065 1473 50  0000 C CNN
F 2 "" H 9050 1300 50  0001 C CNN
F 3 "" H 9050 1300 50  0001 C CNN
	1    9050 1300
	1    0    0    -1  
$EndComp
Text Label 4500 4150 0    50   ~ 0
NRST
Text Label 5800 4050 2    50   ~ 0
SWDIO
Text Label 5800 4150 2    50   ~ 0
SWCLK
$Comp
L Regulator_Linear:MIC5504-3.3YM5 U1
U 1 1 5FE87529
P 2300 1450
F 0 "U1" H 2300 1817 50  0000 C CNN
F 1 "MIC5504-3.3YM5" H 2300 1726 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 2300 1050 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/MIC550X.pdf" H 2050 1700 50  0001 C CNN
F 4 "MIC5504-3.3YM5-T5TR-ND" H 2300 1450 50  0001 C CNN "DIGI_PN"
	1    2300 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0108
U 1 1 5FE88988
P 1150 1250
F 0 "#PWR0108" H 1150 1100 50  0001 C CNN
F 1 "+5V" H 1165 1423 50  0000 C CNN
F 2 "" H 1150 1250 50  0001 C CNN
F 3 "" H 1150 1250 50  0001 C CNN
	1    1150 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1350 1150 1250
$Comp
L Device:C_Small C1
U 1 1 5FE8FE9C
P 1150 1550
F 0 "C1" H 1242 1596 50  0000 L CNN
F 1 "1uF" H 1242 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1150 1550 50  0001 C CNN
F 3 "~" H 1150 1550 50  0001 C CNN
F 4 "1276-6524-2-ND" H 1150 1550 50  0001 C CNN "DIGI_PN"
	1    1150 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5FE9209C
P 2800 1550
F 0 "C2" H 2892 1596 50  0000 L CNN
F 1 "1uF" H 2892 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2800 1550 50  0001 C CNN
F 3 "~" H 2800 1550 50  0001 C CNN
F 4 "1276-6524-2-ND" H 2800 1550 50  0001 C CNN "DIGI_PN"
	1    2800 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1450 1150 1350
Connection ~ 1150 1350
Wire Wire Line
	2700 1350 2800 1350
Wire Wire Line
	2800 1350 2800 1200
Wire Wire Line
	2800 1450 2800 1350
Connection ~ 2800 1350
$Comp
L power:GND #PWR0109
U 1 1 5FE9F836
P 2300 1850
F 0 "#PWR0109" H 2300 1600 50  0001 C CNN
F 1 "GND" H 2305 1677 50  0000 C CNN
F 2 "" H 2300 1850 50  0001 C CNN
F 3 "" H 2300 1850 50  0001 C CNN
	1    2300 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5FEA0252
P 2800 1750
F 0 "#PWR0110" H 2800 1500 50  0001 C CNN
F 1 "GND" H 2805 1577 50  0000 C CNN
F 2 "" H 2800 1750 50  0001 C CNN
F 3 "" H 2800 1750 50  0001 C CNN
	1    2800 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5FEA091C
P 1150 1750
F 0 "#PWR0111" H 1150 1500 50  0001 C CNN
F 1 "GND" H 1155 1577 50  0000 C CNN
F 2 "" H 1150 1750 50  0001 C CNN
F 3 "" H 1150 1750 50  0001 C CNN
	1    1150 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1750 1150 1650
Wire Wire Line
	2300 1850 2300 1750
Wire Wire Line
	2800 1750 2800 1650
$Comp
L power:+3V3 #PWR0112
U 1 1 5FED4030
P 2800 1200
F 0 "#PWR0112" H 2800 1050 50  0001 C CNN
F 1 "+3V3" H 2815 1373 50  0000 C CNN
F 2 "" H 2800 1200 50  0001 C CNN
F 3 "" H 2800 1200 50  0001 C CNN
	1    2800 1200
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5FED7117
P 5100 6600
F 0 "SW1" H 5100 6885 50  0000 C CNN
F 1 "RESET" H 5100 6794 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_EVQPE1" H 5100 6800 50  0001 C CNN
F 3 "~" H 5100 6800 50  0001 C CNN
F 4 "CKN12311-2-ND" H 5100 6600 50  0001 C CNN "DIGI_PN"
	1    5100 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 6600 4650 6600
Wire Wire Line
	5300 6600 5400 6600
Wire Wire Line
	5400 6600 5400 6700
$Comp
L power:GND #PWR0113
U 1 1 5FEDEA37
P 5400 6700
F 0 "#PWR0113" H 5400 6450 50  0001 C CNN
F 1 "GND" H 5405 6527 50  0000 C CNN
F 2 "" H 5400 6700 50  0001 C CNN
F 3 "" H 5400 6700 50  0001 C CNN
	1    5400 6700
	1    0    0    -1  
$EndComp
Text Label 4650 6600 0    50   ~ 0
NRST
$Comp
L Device:C_Small C5
U 1 1 5FEE64BD
P 5450 1500
F 0 "C5" H 5542 1546 50  0000 L CNN
F 1 "10uf" H 5542 1455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5450 1500 50  0001 C CNN
F 3 "~" H 5450 1500 50  0001 C CNN
F 4 "490-3298-1-ND" H 5450 1500 50  0001 C CNN "DIGI_PN"
	1    5450 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5FEE69AD
P 4500 1500
F 0 "C3" H 4592 1546 50  0000 L CNN
F 1 "100nF" H 4592 1455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4500 1500 50  0001 C CNN
F 3 "~" H 4500 1500 50  0001 C CNN
F 4 "311-2077-1-ND" H 4500 1500 50  0001 C CNN "DIGI_PN"
	1    4500 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5FEE7B73
P 5000 1500
F 0 "C4" H 5092 1546 50  0000 L CNN
F 1 "100nF" H 5092 1455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5000 1500 50  0001 C CNN
F 3 "~" H 5000 1500 50  0001 C CNN
F 4 "311-2077-1-ND" H 5000 1500 50  0001 C CNN "DIGI_PN"
	1    5000 1500
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0114
U 1 1 5FEE7FBD
P 4500 1300
F 0 "#PWR0114" H 4500 1150 50  0001 C CNN
F 1 "+3V3" H 4515 1473 50  0000 C CNN
F 2 "" H 4500 1300 50  0001 C CNN
F 3 "" H 4500 1300 50  0001 C CNN
	1    4500 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0115
U 1 1 5FEE86BF
P 5000 1300
F 0 "#PWR0115" H 5000 1150 50  0001 C CNN
F 1 "+3V3" H 5015 1473 50  0000 C CNN
F 2 "" H 5000 1300 50  0001 C CNN
F 3 "" H 5000 1300 50  0001 C CNN
	1    5000 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0116
U 1 1 5FEE8AD6
P 5450 1300
F 0 "#PWR0116" H 5450 1150 50  0001 C CNN
F 1 "+3V3" H 5465 1473 50  0000 C CNN
F 2 "" H 5450 1300 50  0001 C CNN
F 3 "" H 5450 1300 50  0001 C CNN
	1    5450 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 5FEE9476
P 4500 1700
F 0 "#PWR0117" H 4500 1450 50  0001 C CNN
F 1 "GND" H 4505 1527 50  0000 C CNN
F 2 "" H 4500 1700 50  0001 C CNN
F 3 "" H 4500 1700 50  0001 C CNN
	1    4500 1700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 5FEE9D05
P 5000 1700
F 0 "#PWR0118" H 5000 1450 50  0001 C CNN
F 1 "GND" H 5005 1527 50  0000 C CNN
F 2 "" H 5000 1700 50  0001 C CNN
F 3 "" H 5000 1700 50  0001 C CNN
	1    5000 1700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5FEEA161
P 5450 1700
F 0 "#PWR0119" H 5450 1450 50  0001 C CNN
F 1 "GND" H 5455 1527 50  0000 C CNN
F 2 "" H 5450 1700 50  0001 C CNN
F 3 "" H 5450 1700 50  0001 C CNN
	1    5450 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 1600 5450 1700
Wire Wire Line
	5000 1600 5000 1700
Wire Wire Line
	4500 1600 4500 1700
Wire Wire Line
	4500 1300 4500 1400
Wire Wire Line
	5000 1300 5000 1400
Wire Wire Line
	5450 1300 5450 1400
Wire Wire Line
	8150 5000 7600 5000
$Comp
L Device:R_Small R5
U 1 1 5FF0D146
P 5450 3800
F 0 "R5" H 5509 3846 50  0000 L CNN
F 1 "1K" H 5509 3755 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 5450 3800 50  0001 C CNN
F 3 "~" H 5450 3800 50  0001 C CNN
F 4 "311-1.00KLRTR-ND" H 5450 3800 50  0001 C CNN "DIGI_PN"
	1    5450 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 4050 5450 4050
Wire Wire Line
	5450 3900 5450 4050
Connection ~ 5450 4050
Wire Wire Line
	5450 4050 5800 4050
$Comp
L power:+3V3 #PWR0120
U 1 1 5FF1EF27
P 5450 3600
F 0 "#PWR0120" H 5450 3450 50  0001 C CNN
F 1 "+3V3" H 5465 3773 50  0000 C CNN
F 2 "" H 5450 3600 50  0001 C CNN
F 3 "" H 5450 3600 50  0001 C CNN
	1    5450 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 3700 5450 3600
$Comp
L Device:R_Small R1
U 1 1 5FF27EE3
P 1500 1550
F 0 "R1" H 1559 1596 50  0000 L CNN
F 1 "100K" H 1559 1505 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 1500 1550 50  0001 C CNN
F 3 "~" H 1500 1550 50  0001 C CNN
F 4 "YAG2307TR-ND" H 1500 1550 50  0001 C CNN "DIGI_PN"
	1    1500 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1350 1500 1350
Wire Wire Line
	1500 1450 1500 1350
Connection ~ 1500 1350
Wire Wire Line
	1500 1350 1900 1350
Wire Wire Line
	1900 1550 1800 1550
Wire Wire Line
	1800 1550 1800 1750
Wire Wire Line
	1800 1750 1500 1750
Wire Wire Line
	1500 1750 1500 1650
$Comp
L Device:LED_ABGR D1
U 1 1 5FF60D8C
P 5100 5500
F 0 "D1" H 5100 5997 50  0000 C CNN
F 1 "LED_ABGR" H 5100 5906 50  0000 C CNN
F 2 "LED_SMD:LED_Cree-PLCC4_2x2mm_CW" H 5100 5450 50  0001 C CNN
F 3 "~" H 5100 5450 50  0001 C CNN
F 4 "CLMVC-FKA-CL1D1L71BB7C3C3TR-ND" H 5100 5500 50  0001 C CNN "DIGI_PN"
	1    5100 5500
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R4
U 1 1 5FF622FC
P 4700 5700
F 0 "R4" V 4600 5750 50  0000 C CNN
F 1 "390" V 4600 5600 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 4700 5700 50  0001 C CNN
F 3 "~" H 4700 5700 50  0001 C CNN
F 4 "311-390LRTR-ND" H 4700 5700 50  0001 C CNN "DIGI_PN"
	1    4700 5700
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R3
U 1 1 5FF63245
P 4700 5500
F 0 "R3" V 4600 5550 50  0000 C CNN
F 1 "390" V 4600 5400 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 4700 5500 50  0001 C CNN
F 3 "~" H 4700 5500 50  0001 C CNN
F 4 "311-390LRTR-ND" H 4700 5500 50  0001 C CNN "DIGI_PN"
	1    4700 5500
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5FF638D1
P 4700 5300
F 0 "R2" V 4600 5350 50  0000 C CNN
F 1 "620" V 4600 5200 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 4700 5300 50  0001 C CNN
F 3 "~" H 4700 5300 50  0001 C CNN
F 4 "311-620LRTR-ND" H 4700 5300 50  0001 C CNN "DIGI_PN"
	1    4700 5300
	0    1    1    0   
$EndComp
Wire Wire Line
	4800 5300 4900 5300
Wire Wire Line
	4800 5500 4900 5500
Wire Wire Line
	4800 5700 4900 5700
Wire Wire Line
	4250 5300 4600 5300
Wire Wire Line
	4600 5500 4250 5500
Wire Wire Line
	4600 5700 4250 5700
Text Label 4250 5300 0    50   ~ 0
LED_R
Text Label 4250 5500 0    50   ~ 0
LED_G
Text Label 4250 5700 0    50   ~ 0
LED_B
$Comp
L power:+5V #PWR0121
U 1 1 5FF84F53
P 5400 5100
F 0 "#PWR0121" H 5400 4950 50  0001 C CNN
F 1 "+5V" H 5415 5273 50  0000 C CNN
F 2 "" H 5400 5100 50  0001 C CNN
F 3 "" H 5400 5100 50  0001 C CNN
	1    5400 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 5500 5400 5500
Wire Wire Line
	5400 5500 5400 5100
$Comp
L Device:Crystal_Small Y1
U 1 1 6001507D
P 6950 2050
F 0 "Y1" V 6904 2138 50  0000 L CNN
F 1 "32.768KHz" V 7000 2150 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_MicroCrystal_CC7V-T1A-2Pin_3.2x1.5mm_HandSoldering" H 6950 2050 50  0001 C CNN
F 3 "~" H 6950 2050 50  0001 C CNN
F 4 "2195-CM7VT1ALOWESR32.768KHZ12.5PF20TAQCTR-ND" H 6950 2050 50  0001 C CNN "DIGI_PN"
	1    6950 2050
	0    1    1    0   
$EndComp
Wire Wire Line
	8150 2100 7500 2100
Wire Wire Line
	7500 2000 8150 2000
Wire Wire Line
	7500 2000 7500 1900
Wire Wire Line
	7500 1900 6950 1900
Wire Wire Line
	6950 1900 6950 1950
Wire Wire Line
	7500 2100 7500 2200
Wire Wire Line
	7500 2200 6950 2200
Wire Wire Line
	6950 2200 6950 2150
Wire Wire Line
	6950 1900 6600 1900
Wire Wire Line
	6600 1900 6600 2250
Connection ~ 6950 1900
Wire Wire Line
	6950 2250 6950 2200
Connection ~ 6950 2200
$Comp
L power:GND #PWR0122
U 1 1 6010EBB1
P 6950 2550
F 0 "#PWR0122" H 6950 2300 50  0001 C CNN
F 1 "GND" H 6955 2377 50  0000 C CNN
F 2 "" H 6950 2550 50  0001 C CNN
F 3 "" H 6950 2550 50  0001 C CNN
	1    6950 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0123
U 1 1 6010F4E3
P 6600 2550
F 0 "#PWR0123" H 6600 2300 50  0001 C CNN
F 1 "GND" H 6605 2377 50  0000 C CNN
F 2 "" H 6600 2550 50  0001 C CNN
F 3 "" H 6600 2550 50  0001 C CNN
	1    6600 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 2550 6600 2450
Wire Wire Line
	6950 2550 6950 2450
Wire Wire Line
	8550 1700 8550 1600
Wire Wire Line
	8550 1600 8450 1600
Wire Wire Line
	8250 1600 8100 1600
Wire Wire Line
	8100 1600 8100 1700
Text Notes 6650 7700 0    50   ~ 0
lithe usb fight stick controller
Text Notes 9900 7850 0    50   ~ 0
0\n
Text Notes 6600 7450 0    50   ~ 0
1
Text Notes 6700 7450 0    50   ~ 0
1
Text Notes 7450 7850 0    50   ~ 0
2020-12-18
Text Notes 9250 7550 0    50   ~ 0
Copyright 2020 Erik Gilling\nLicensed under the MIT licence.
$Comp
L Mechanical:MountingHole H2
U 1 1 5FF1212C
P 6400 6100
F 0 "H2" H 6300 6000 50  0000 L CNN
F 1 "MountingHole" H 6500 6055 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_ISO7380" H 6400 6100 50  0001 C CNN
F 3 "~" H 6400 6100 50  0001 C CNN
	1    6400 6100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 5FF13653
P 6600 6100
F 0 "H4" H 6650 6000 50  0000 L CNN
F 1 "MountingHole" H 6700 6055 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_ISO7380" H 6600 6100 50  0001 C CNN
F 3 "~" H 6600 6100 50  0001 C CNN
	1    6600 6100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H1
U 1 1 5FF13B50
P 6400 5900
F 0 "H1" H 6300 6000 50  0000 L CNN
F 1 "MountingHole" H 6500 5855 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_ISO7380" H 6400 5900 50  0001 C CNN
F 3 "~" H 6400 5900 50  0001 C CNN
	1    6400 5900
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 5FF14BA4
P 6600 5900
F 0 "H3" H 6650 6000 50  0000 L CNN
F 1 "MountingHole" H 6700 5855 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_ISO7380" H 6600 5900 50  0001 C CNN
F 3 "~" H 6600 5900 50  0001 C CNN
	1    6600 5900
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R9
U 1 1 5FFAD33F
P 7150 4500
F 0 "R9" H 7209 4546 50  0000 L CNN
F 1 "DNI" H 7209 4455 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 7150 4500 50  0001 C CNN
F 3 "~" H 7150 4500 50  0001 C CNN
F 4 "311-0.0JRTR-ND" H 7150 4500 50  0001 C CNN "DIGI_PN"
	1    7150 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 4000 7150 4000
$Comp
L Device:R_Small R8
U 1 1 5FFC9F6F
P 6900 4500
F 0 "R8" H 6959 4546 50  0000 L CNN
F 1 "DNI" H 6959 4455 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 6900 4500 50  0001 C CNN
F 3 "~" H 6900 4500 50  0001 C CNN
F 4 "311-0.0JRTR-ND" H 6900 4500 50  0001 C CNN "DIGI_PN"
	1    6900 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R7
U 1 1 5FFD109D
P 6650 4500
F 0 "R7" H 6709 4546 50  0000 L CNN
F 1 "DNI" H 6709 4455 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 6650 4500 50  0001 C CNN
F 3 "~" H 6650 4500 50  0001 C CNN
F 4 "311-0.0JRTR-ND" H 6650 4500 50  0001 C CNN "DIGI_PN"
	1    6650 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R6
U 1 1 5FFD8418
P 6400 4500
F 0 "R6" H 6459 4546 50  0000 L CNN
F 1 "DNI" H 6459 4455 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 6400 4500 50  0001 C CNN
F 3 "~" H 6400 4500 50  0001 C CNN
F 4 "311-0.0JRTR-ND" H 6400 4500 50  0001 C CNN "DIGI_PN"
	1    6400 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 4100 6900 4100
Wire Wire Line
	8150 4200 6650 4200
Text Label 7600 4000 0    50   ~ 0
BOARD_ID0
Text Label 7600 4100 0    50   ~ 0
BOARD_ID1
Text Label 7600 4200 0    50   ~ 0
BOARD_ID2
Text Label 7600 4300 0    50   ~ 0
BOARD_ID3
Text Notes 6950 4050 2    50   ~ 0
BOARD ID 0\n
Wire Wire Line
	7150 4000 7150 4400
Wire Wire Line
	6900 4100 6900 4400
Wire Wire Line
	6650 4200 6650 4400
Wire Wire Line
	6400 4300 6400 4400
Wire Wire Line
	6400 4300 8150 4300
$Comp
L power:GND #PWR0128
U 1 1 600C7DC4
P 6400 4700
F 0 "#PWR0128" H 6400 4450 50  0001 C CNN
F 1 "GND" H 6405 4527 50  0000 C CNN
F 2 "" H 6400 4700 50  0001 C CNN
F 3 "" H 6400 4700 50  0001 C CNN
	1    6400 4700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 600C8928
P 6650 4700
F 0 "#PWR0129" H 6650 4450 50  0001 C CNN
F 1 "GND" H 6655 4527 50  0000 C CNN
F 2 "" H 6650 4700 50  0001 C CNN
F 3 "" H 6650 4700 50  0001 C CNN
	1    6650 4700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0130
U 1 1 600C8DB0
P 6900 4700
F 0 "#PWR0130" H 6900 4450 50  0001 C CNN
F 1 "GND" H 6905 4527 50  0000 C CNN
F 2 "" H 6900 4700 50  0001 C CNN
F 3 "" H 6900 4700 50  0001 C CNN
	1    6900 4700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0131
U 1 1 600C957C
P 7150 4700
F 0 "#PWR0131" H 7150 4450 50  0001 C CNN
F 1 "GND" H 7155 4527 50  0000 C CNN
F 2 "" H 7150 4700 50  0001 C CNN
F 3 "" H 7150 4700 50  0001 C CNN
	1    7150 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 4600 7150 4700
Wire Wire Line
	6900 4600 6900 4700
Wire Wire Line
	6650 4600 6650 4700
Wire Wire Line
	6400 4600 6400 4700
$Comp
L Diode:SM712_SOT23 D2
U 1 1 612A8252
P 2000 7000
F 0 "D2" V 2150 6800 50  0000 L CNN
F 1 "SM712_SOT23" V 1500 6750 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2000 6650 50  0001 C CNN
F 3 "https://www.littelfuse.com/~/media/electronics/datasheets/tvs_diode_arrays/littelfuse_tvs_diode_array_sm712_datasheet.pdf.pdf" H 1850 7000 50  0001 C CNN
	1    2000 7000
	0    -1   1    0   
$EndComp
Wire Wire Line
	2000 7350 2000 7400
Wire Wire Line
	2000 7400 1750 7400
Wire Wire Line
	2000 6650 2000 6600
$Comp
L Interface_UART:SN75176AD U4
U 1 1 612F4455
P 2850 6900
F 0 "U4" H 2750 7250 50  0000 C CNN
F 1 "SN75176AD" H 2600 6550 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 2850 6400 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn75176a.pdf" H 4450 6700 50  0001 C CNN
	1    2850 6900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2500 7400 2500 7100
Wire Wire Line
	2500 7100 2550 7100
Connection ~ 2000 7400
Wire Wire Line
	2550 7000 2500 7000
Wire Wire Line
	2500 7000 2500 6600
Connection ~ 2000 6600
$Comp
L power:GND #PWR0124
U 1 1 613044A5
P 2250 7150
F 0 "#PWR0124" H 2250 6900 50  0001 C CNN
F 1 "GND" H 2255 6977 50  0000 C CNN
F 2 "" H 2250 7150 50  0001 C CNN
F 3 "" H 2250 7150 50  0001 C CNN
	1    2250 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 7000 2250 7000
Wire Wire Line
	2250 7000 2250 7150
$Comp
L Connector:Screw_Terminal_01x03 J1
U 1 1 61329852
P 800 7100
F 0 "J1" H 718 6775 50  0000 C CNN
F 1 "Screw_Terminal_01x03" H 718 6866 50  0000 C CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_PT-1,5-3-3.5-H_1x03_P3.50mm_Horizontal" H 800 7100 50  0001 C CNN
F 3 "~" H 800 7100 50  0001 C CNN
	1    800  7100
	-1   0    0    1   
$EndComp
Wire Wire Line
	1750 7400 1750 7100
Wire Wire Line
	1750 7100 1000 7100
Wire Wire Line
	2000 6600 1750 6600
Wire Wire Line
	1750 6600 1750 7000
Wire Wire Line
	1750 7000 1000 7000
$Comp
L power:GND #PWR0125
U 1 1 61346186
P 1500 7300
F 0 "#PWR0125" H 1500 7050 50  0001 C CNN
F 1 "GND" H 1505 7127 50  0000 C CNN
F 2 "" H 1500 7300 50  0001 C CNN
F 3 "" H 1500 7300 50  0001 C CNN
	1    1500 7300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 7200 1500 7200
Wire Wire Line
	1500 7200 1500 7300
Wire Wire Line
	3150 6800 3800 6800
Wire Wire Line
	3150 7100 3800 7100
Text Label 3800 6800 2    50   ~ 0
DMX_TX
Text Label 3800 7100 2    50   ~ 0
DMX_RX
Wire Wire Line
	3150 6700 3800 6700
Text Label 3800 6700 2    50   ~ 0
DMX_TX_EN
Wire Wire Line
	3150 7000 3800 7000
Text Label 3800 7000 2    50   ~ 0
DMX_TX_EN
$Comp
L power:+3V3 #PWR0126
U 1 1 613AFA5E
P 2850 6400
F 0 "#PWR0126" H 2850 6250 50  0001 C CNN
F 1 "+3V3" H 2865 6573 50  0000 C CNN
F 2 "" H 2850 6400 50  0001 C CNN
F 3 "" H 2850 6400 50  0001 C CNN
	1    2850 6400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 613B0E63
P 2850 7450
F 0 "#PWR0127" H 2850 7200 50  0001 C CNN
F 1 "GND" H 2855 7277 50  0000 C CNN
F 2 "" H 2850 7450 50  0001 C CNN
F 3 "" H 2850 7450 50  0001 C CNN
	1    2850 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 6500 2850 6400
Wire Wire Line
	2850 7300 2850 7450
$Comp
L Connector:USB_B_Micro J3
U 1 1 613C451D
P 2300 3350
F 0 "J3" H 2357 3817 50  0000 C CNN
F 1 "USB_B_Micro" H 2357 3726 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 2450 3300 50  0001 C CNN
F 3 "~" H 2450 3300 50  0001 C CNN
	1    2300 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 3100 10400 3100
Text Label 10400 3100 2    50   ~ 0
UNUSED_SPI_CLK
Wire Wire Line
	9550 3000 10400 3000
Text Label 10400 3000 2    50   ~ 0
WS2812_DATA
$Comp
L Connector:Screw_Terminal_01x02 J2
U 1 1 614106BB
P 1350 5250
F 0 "J2" H 1268 4925 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 1268 5016 50  0000 C CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_PT-1,5-2-3.5-H_1x02_P3.50mm_Horizontal" H 1350 5250 50  0001 C CNN
F 3 "~" H 1350 5250 50  0001 C CNN
	1    1350 5250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0132
U 1 1 61412513
P 1650 5300
F 0 "#PWR0132" H 1650 5050 50  0001 C CNN
F 1 "GND" H 1655 5127 50  0000 C CNN
F 2 "" H 1650 5300 50  0001 C CNN
F 3 "" H 1650 5300 50  0001 C CNN
	1    1650 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 5250 1650 5250
Wire Wire Line
	1650 5250 1650 5300
Text Label 2250 5150 2    50   ~ 0
WS2812_DATA_5V
$Comp
L Logic_LevelTranslator:TXB0102DCU U3
U 1 1 6142450E
P 2750 5050
F 0 "U3" H 3000 4550 50  0000 C CNN
F 1 "TXB0102DCU" H 2450 4550 50  0000 C CNN
F 2 "Package_SO:VSSOP-8_2.4x2.1mm_P0.5mm" H 2750 4500 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/txb0102.pdf" H 2750 5020 50  0001 C CNN
	1    2750 5050
	-1   0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0133
U 1 1 6144BB67
P 2850 4450
F 0 "#PWR0133" H 2850 4300 50  0001 C CNN
F 1 "+3V3" H 2865 4623 50  0000 C CNN
F 2 "" H 2850 4450 50  0001 C CNN
F 3 "" H 2850 4450 50  0001 C CNN
	1    2850 4450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0134
U 1 1 6144C54F
P 2650 4450
F 0 "#PWR0134" H 2650 4300 50  0001 C CNN
F 1 "+5V" H 2665 4623 50  0000 C CNN
F 2 "" H 2650 4450 50  0001 C CNN
F 3 "" H 2650 4450 50  0001 C CNN
	1    2650 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0135
U 1 1 6144CE7D
P 2750 5650
F 0 "#PWR0135" H 2750 5400 50  0001 C CNN
F 1 "GND" H 2755 5477 50  0000 C CNN
F 2 "" H 2750 5650 50  0001 C CNN
F 3 "" H 2750 5650 50  0001 C CNN
	1    2750 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 5150 2350 5150
Wire Wire Line
	3150 5150 3750 5150
Text Label 3750 5150 2    50   ~ 0
WS2812_DATA
Wire Wire Line
	2850 4450 2850 4550
Wire Wire Line
	2650 4450 2650 4550
Wire Wire Line
	2750 5550 2750 5650
$Comp
L power:+3V3 #PWR0136
U 1 1 6147473F
P 3900 5250
F 0 "#PWR0136" H 3900 5100 50  0001 C CNN
F 1 "+3V3" H 3915 5423 50  0000 C CNN
F 2 "" H 3900 5250 50  0001 C CNN
F 3 "" H 3900 5250 50  0001 C CNN
	1    3900 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 5350 3900 5350
Wire Wire Line
	3900 5350 3900 5250
NoConn ~ 3000 4150
Wire Wire Line
	8150 3100 7600 3100
Text Label 7600 3100 0    50   ~ 0
DMX_TX
Wire Wire Line
	8150 3000 7600 3000
Text Label 7600 3000 0    50   ~ 0
DMX_RX
Wire Wire Line
	8150 3700 7600 3700
Wire Wire Line
	8150 3800 7600 3800
Wire Wire Line
	8150 3900 7600 3900
Text Label 7600 3700 0    50   ~ 0
LED_B
Text Label 7600 3800 0    50   ~ 0
LED_G
Text Label 7600 3900 0    50   ~ 0
LED_R
Wire Wire Line
	8150 2800 7600 2800
Text Label 7600 2800 0    50   ~ 0
DMX_TX_EN
Wire Wire Line
	2000 7400 2500 7400
Wire Wire Line
	2000 6600 2500 6600
Text Label 1050 7100 0    50   ~ 0
DMX_P
Text Label 1050 7000 0    50   ~ 0
DMX_M
$EndSCHEMATC