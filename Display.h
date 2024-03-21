/*
 * Display.h
 *
 *  Created on: 21/03/2024
 *      Author: hrc48
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "OrbitOLED/lib_OrbitOled/OrbitOled.h"

#define ADC_STEP_FOR_1V 1240;

enum DisplayMode { PERCENTAGE_ALTITUDE, MEAN_ADC, DISPLAY_OFF };

#ifndef DISPLAY_H_
#define DISPLAY_H_



void displayAltitude(uint16_t baseAltitude, uint16_t currentMean, uint8_t displayCycle);

void displayMeanVal(uint16_t meanVal, uint32_t count);

void initDisplay (void);


#endif /* DISPLAY_H_ */
