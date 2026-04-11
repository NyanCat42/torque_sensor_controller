#pragma once
#include <Arduino.h>
#include "config.h"

// Pins
#define TORQUE_PIN 36  // VP, ADC1_CH0
#define CADENCE_PIN 34 // input-only, PCNT

void sensorsInit(const AppConfig &cfg);
void sensorsUpdate(const AppConfig &cfg); // call each control loop iteration

float getTorqueNm();
float getTorqueSensorV();
float getCadenceRpm();
float getRiderPowerW();
