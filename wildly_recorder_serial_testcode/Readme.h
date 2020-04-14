WORKING DETAILS:

* This code records audio file in .wav format to sdcard available on teensy3.5/3.6 microcontroller(48MHZ).
* Device can record audio at different sample rate(8,16,32,44.1 and 48khz).
* Device creates a new directory every day in the sdcard and writes file inside it.
* Device can be configured by changing the OPERATING_MODE MACRO in device.h file.
* Instead of using onnboard RTC, an external RTC module is interfaced to Teensy board.

* Directory name format : DEVYYYYMMDD_HH (Eg: DEV:20200211_11)
  Filename format 	: AYYYYMMDD_HHMMSS.wav (Eg: A20200211_112259.wav)
* wav file contains device details in the wav file header in this format:
  Operator:NA, DeviceID:A5487100, Batt: 4.1V, Pct:93.4, Signal: NA-NA, Firm_rev: Ver1.0, Timestamp:2047/07/23-17:27:39, Latitude:NA, Longitude:NA,Clock:48000000

SLEEP MODE:

* Change the recording time window MACRO in device.h to set recording time for the device.
  (There are 2 time windows in the code, T1_WINDOWS to T2_WINDOW and T3_WINDOW to T4_WINDOW. Time window should be set in hours.)

CONFIGURATION MODE:

* When OPERATING_MODE MACRO in device.h file is set to 2, device will be in configuration mode.
  Pass following commands to set or get the configuration values:
  
  SAMPLE RATE:
  1) "SMF,value"   // To set sample rate of the device (value:8,16,32,44,48)
  2) "SMF "	   // To read value of sample rate
  MIC GAIN:
  1) "MCG,value"   // To set mic gain of the device (value: 5-20)
  2) "MCG "	   // To read value of mic gain
  RECORDING TIME:
  1) "RCT,value"   // To set recording time value in seconds (value:0-255)
  2) "RCT "	   // To read value of recording time
	

    
* "T:T1_WINDOW,T2_WINDOW,T3_WINDOW,T4_WINDOW"
   (Eg: T:02,10,15,24
   This will record from 2 midnight to morning 10 and 3 afternoon to 12 in night)

* To check time window set pass: (T#)

* To set time of the device, pass:"TIME:12:14:55"

* To check time set on the device, pass:"TIME#"

* To set date of the device, pass:"DATE:12/11/20"

* To check date set on the device, pass:"DATE#"

