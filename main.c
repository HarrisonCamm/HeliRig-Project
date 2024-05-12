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
#include "uart.h"


//Task flags
static volatile bool flagController = false;
static volatile bool flagButtons = false;
static volatile bool flagDisplay = false;
static volatile bool flagUART = false;

//********************************************************
// Global variables
//********************************************************
#define MAX_STR_LEN 90

char statusStr[MAX_STR_LEN + 1];

#define CONTROL_PERIOD 1    //Corrosponds to 1000Hz
#define BUTTON_PERIOD 10    //Corrosponds to 100Hz
#define DISPLAY_PERIOD 15   //Corrosponds to 66.67Hz
#define UART_PERIOD 200     //Corrosponds to 5Hz
#define START_DELAY 5       //200 ms delay



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
    static uint8_t uartCounter = 0;

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

    if (uartCounter >= UART_PERIOD) {
        flagUART = true;
        uartCounter = 0;
    }


    controllerCounter++;
    buttonsCounter++;
    displayCounter++;
    uartCounter++;
}

void
poleButtons(void) {

    updateButtons();

    // Left button decreases Yaw
    if (checkButton(LEFT) == PUSHED) {
        decYaw();
    }

    // Right button decreases Yaw
    if (checkButton(RIGHT) == PUSHED) {
        incYaw();
    }

    // Right button decreases Yaw
    if (checkButton(UP) == PUSHED) {
        incAlt();
    }

    // Right button decreases Yaw
    if (checkButton(DOWN) == PUSHED) {
        decAlt();
    }
}


int
main(void)
{
    uint16_t currentAlt;
    int32_t currentYaw;
    uint16_t initLandedADC;
    int32_t mainDuty;
    int32_t tailDuty;
    enum DisplayMode displayCycle = PROCESSED;

    initClock ();
    initButtons();
    initADC ();
    initDisplay ();
    initQuad();
    initialisePWM();
    initialiseUSB_UART();
    initialiseSwitch();


    //
    // Enable interrupts to the processor.
    IntMasterEnable();

    SysCtlDelay (SysCtlClockGet() / START_DELAY);  // ~200ms delay for buffer to populate

    
    // Calculate and display the rounded mean of the buffer contents
    initLandedADC = getAltMean();

    initAltLimits(initLandedADC);

    while (1)
    {
        //Flag Controller
        if (flagController) {
            //Update current sensor values
            currentAlt = getAltMean();
            currentYaw = getYawPosition();

            mainDuty = controllerMain(currentAlt);
            tailDuty = controllerTail(mainDuty, currentYaw);

            setDuty(mainDuty, tailDuty);

            flagController = false;
        }
        if (flagButtons) {
            poleButtons();
            UpdateHelicopter();
            flagButtons = false;
        }
        if (flagDisplay) {
            // Refresh the display
            displayWrite(initLandedADC, currentAlt, currentYaw, displayCycle);
            flagDisplay = false;
        }

        if (flagUART) {

            int32_t actualAlt = getAltPercent(initLandedADC, currentAlt);
            int32_t desireAlt = getAltPercent(initLandedADC, getAltSet());

            int32_t actualYaw = getYawDegree(currentYaw) / SCALE_BY_100;
            int32_t desireYaw = getYawDegree(getYawSet()) / SCALE_BY_100;

            //Update UART string
            usprintf (statusStr, "Alt(Actual/Set) %d/%d | Yaw(Actual/Set) %d/%d | Main Duty %d | Tail Duty %d \r\n", currentAlt, getAltSet(), actualYaw, desireYaw, mainDuty, tailDuty);
            UARTSend (statusStr);

            flagUART = false;
        }


        //SysCtlDelay (SysCtlClockGet() / 100);  // Delay to prevent flickering
    }
}
