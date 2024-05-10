/*
 * milestone1.c
 *
 *  Created on: 21/03/2024
 *      Authors: jwi182, hrc48
 */


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
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
#include "circBufT.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "OrbitOLED/lib_OrbitOled/OrbitOled.h"
#include "buttons4.h"
#include "display.h"
#include "ADC.h"
#include "quadrature.h"
#include "pwmRotor.h"


//Task flags
static volatile bool flagController = false;
static volatile bool flagButtons = false;
static volatile bool flagDisplay = false;

#define CONTROL_PERIOD 1 //Corrosponds to 1ms
#define BUTTON_PERIOD 10 //Corrosponds to 10ms
#define DISPLAY_PERIOD 15 //Corrosponds to 15ms



//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}


//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    static uint8_t controllerCounter = 0;
    static uint8_t buttonsCounter = 0;
    static uint8_t displayCounter = 0;

    //
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3);

    if (controllerCounter >= CONTROL_PERIOD) {
        flagController = true;
        controllerCounter = 0;
    }

    if (buttonsCounter >= BUTTON_PERIOD) {
        flagButtons = true;
        buttonsCounter = 0;
    }

    if (displayCounter >= DISPLAY_PERIOD) {
        flagDisplay = true;
        displayCounter = 0;
    }


    controllerCounter++;
    buttonsCounter++;
    displayCounter++;
}

void
poleButtons(uint16_t* initialADC, enum DisplayMode* displayCycle, uint16_t* currentAlt, int32_t* currentYaw) {

    updateButtons();
    //
    // Background task: calculate the (approximate) mean of the values in the

    *currentAlt = getAltMean();
    *currentYaw = getYawPosition();

    // Reset Landed ADC value when button pushed
    if (checkButton(LEFT) == PUSHED) {
        *initialADC = *currentAlt;
    }

    // Cycle through display modes when button pushed
    if (checkButton(UP) == PUSHED) {
        *displayCycle += 1;
        if (*displayCycle == CYCLE_BACK) {
            *displayCycle = PROCESSED;
        }
    }
}


int
main(void)
{
    uint16_t currentAlt;
    int32_t currentYaw;
    uint16_t initLandedADC;
    enum DisplayMode displayCycle = PROCESSED;

    initClock ();
    initButtons();
    initADC ();
    initDisplay ();
    initQuad();
    initialisePWM();


    //
    // Enable interrupts to the processor.
    IntMasterEnable();

    SysCtlDelay (SysCtlClockGet() / 6);  // Wait for buffer to populate

    
    // Calculate and display the rounded mean of the buffer contents
    initLandedADC = getAltMean();

    while (1)
    {
        //Flag Controller
        if (flagController) {

            flagController = false;
        }
        if (flagButtons) {
            //poleButtons
            updateButtons();
            flagButtons = false;
        }
        if (flagDisplay) {
            // Refresh the display
            displayWrite(initLandedADC, currentAlt, currentYaw, displayCycle);
            flagButtons = false;
        }


        //
        // Background task: calculate the (approximate) mean of the values in the

        currentAlt = getAltMean();
        currentYaw = getYawPosition();

        // Reset Landed ADC value when button pushed
        if (checkButton(LEFT) == PUSHED) {
            initLandedADC = currentAlt;
        }

        // Cycle through display modes when button pushed
        if (checkButton(UP) == PUSHED) {
            displayCycle += 1;
            if (displayCycle == CYCLE_BACK) {
                displayCycle = PROCESSED;
            }
        }


        // Refresh the display
        displayWrite(initLandedADC, currentAlt, currentYaw, displayCycle);


        SysCtlDelay (SysCtlClockGet() / 100);  // Delay to prevent flickering
    }
}
