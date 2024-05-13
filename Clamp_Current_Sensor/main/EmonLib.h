/*
  Emon.h - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  GNU GPL
  modified to use up to 12 bits ADC resolution (ex. Arduino Due)
  by boredman@boredomprojects.net 26.12.2013
  Low Pass filter for offset removal replaces HP filter 1/1/2015 - RW
*/

#ifndef EmonLib_h
#define EmonLib_h

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// define theoretical vref calibration constant for use in readvcc()
// 1100mV*1024 ADC steps http://openenergymonitor.org/emon/node/1186
// override in your code with value for your specific AVR chip
// determined by procedure described under "Calibrating the internal reference voltage" at
// http://openenergymonitor.org/emon/buildingblocks/calibration
#ifndef READVCC_CALIBRATION_CONST
#define READVCC_CALIBRATION_CONST 1126400L
#endif

// to enable 12-bit ADC resolution on Arduino Due,
// include the following line in main sketch inside setup() function:
//  analogReadResolution(ADC_BITS);
// otherwise will default to 10 bits, as in regular Arduino-based boards.
#define ADC_BITS    12
#define ADC_COUNTS  (1 << ADC_BITS)

void EnergyMonitor_init();

void EnergyMonitor_voltageCalib(double _VCAL, double _PHASECAL);
void EnergyMonitor_currentCalib(double _ICAL);

void EnergyMonitor_calcVI(unsigned int crossings, unsigned int timeout);
double EnergyMonitor_calcIrms(unsigned int NUMBER_OF_SAMPLES);
void EnergyMonitor_serialprint();

#endif
