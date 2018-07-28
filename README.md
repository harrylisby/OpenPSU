# OpenPSU

![alt text](https://github.com/harrylisby/OpenPSU/blob/feature/batteryCharger/Images/Teared.png)

********************************************************************************
OpenPSU is a digitally controlled linear power supply developed by Harry Lisby
********************************************************************************

  This open source project makes possible a hackable power supply which allows you
  to have a high precision psu or a high power one depending on the needs.

  Uses very common signal processing modules for voltage referencing and feedback,
  based on an Arduino nano with an MCP4725 10 bit DAC, an ADS1115 16 bit ADC for
  cheap and high precision control. Aditionally uses an i2c 20x4 lcd screen and an
  incremental encoder with pushbutton for control and configuration.

  Also uses de PID_v1 library which allows fast transition between set voltages,
  working modes and changing loads [output stability].

********************************************************************************
