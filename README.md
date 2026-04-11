# Hardware and software for a torque sensor to [VESC](https://github.com/vedderb/bldc) interface for my electric bicycle


## Hardware


![PCB render](./images/lauryno_torque_controller.png)

 - Uses an ESP32-S3 RF module to communicate to the VESC over CAN bus, setting a motor current value to provide proportional assist, based on the torque at the cranks, measured by the bottom bracket [torque sensor](./documents/S135daff485444e569ac136541eaddd83u.pdf).
 - FRAM chip for high endurance persistent storage to save the odometer value.
 - 2 MOSFETs and a step-down regulator module for front and rear LED light control.


PCB designed to adhere to AISLER simple 2 layer design constraints for affordable manufacturing.


## Software


Current software is vibe-coded for an ESP32 development board as proof of concept, while I wait for the PCB to be manufactured.

 - Features a web interface for parameter tweaking
 - Communication with VESC over serial interface.
 - VESC configuration and firmware updates can be done over BLE using VESC tool.

Build and upload using PlatformIO, after first upload firmware can be updated OTA.
