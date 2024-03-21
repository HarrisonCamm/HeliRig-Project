/*
 * milestone1.c
 *
 *  Created on: 21/03/2024
 *      Authors: jwi182, hrc48
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
#include "circBufT.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "OrbitOLED/lib_OrbitOled/OrbitOled.h"
#include "buttons4.h"
#include "Display.h"
#include "ADC.h"



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

int
main(void)
{
    uint16_t currentMean;
    uint16_t initLandedADC;
    enum DisplayMode displayCycle = PERCENTAGE_ALTITUDE;

    initClock ();
    initButtons();
    initADC ();
    initDisplay ();

    //
    // Enable interrupts to the processor.
    IntMasterEnable();

    SysCtlDelay (SysCtlClockGet() / 6);  // Wait for buffer to populate

    
    // Calculate and display the rounded mean of the buffer contents
    initLandedADC = updateBufMean();

    while (1)
    {
        updateButtons();
        //
        // Background task: calculate the (approximate) mean of the values in the

        currentMean = updateBufMean();

        if (checkButton(LEFT) == PUSHED) {
            initLandedADC = currentMean;
        }


        //displayMeanVal (rndMeanBuf, initLandedADC);

        if (checkButton(UP) == PUSHED) {
            displayCycle += 1;
            if (displayCycle == 3) {
                displayCycle = PERCENTAGE_ALTITUDE;
            }
        }

        displayAltitude(initLandedADC, currentMean, displayCycle);



        SysCtlDelay (SysCtlClockGet() / 100);  // Update display at ~ 2 Hz
    }
}
