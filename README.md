# wildly_recorder_firmware
Repository for the firmware implementation of wildly recorder boards

* This repository contains 3 different firmware versions for wildly recorder boards.
* "wildly_recorder" consist of Teensy and I2S mems mic with a coin cell battery as source to RTC.
* "wildly_recorder_rtcmodule" consist of Teensy and I2C mems mic with external RTC module.
* "wildly_recorder_serial_testcode" consist of Teensy and I2C mems mic with external RTC module with 	some extra custom commands to RESET the device, set device time&date and set recording time window.

Hardware Used: 

1) Teensy 3.6 board
   Link for the board documents:https://www.pjrc.com/store/teensy36.html

2) I2S MEMS Microphone This is I2S mems mic used with Arduino MKR Zero for wav file recording in I2S_Mic project.
   Mic Datasheet: https://learn.adafruit.com/adafruit-i2s-mems-microphone-breakout/

