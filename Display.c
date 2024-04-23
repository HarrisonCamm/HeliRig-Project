/*
 * Display.c
 *
 *  Created on: 21/03/2024
 *      Author: hrc48
 */

#include "Display.h"


void displayWrite(uint16_t baseAlt, uint16_t currentAlt, int32_t currentYaw, uint8_t displayCycle) {

    // Get Altitude as a percentage
    int32_t altPercentage = getAltPercent(baseAlt, currentAlt);

    // Get yaw as a degree
    int32_t yawDegree = getYawDegree(currentYaw);


    char lineString[17]; // 16 characters across the display
    if (displayCycle == PROCESSED) {
        OLEDStringDraw ("Helicopter Stats", 0, 0);

        // Display ADC input as a height percentage
        usnprintf(lineString, sizeof(lineString), "Altitude: %3d%% ", altPercentage);
        OLEDStringDraw (lineString, 0, 1);

        // Display YAW is degrees
        usnprintf(lineString, sizeof(lineString), "Yaw: %4d  ", yawDegree);
        OLEDStringDraw (lineString, 0, 2);


    } else if (displayCycle == RAW) {
        OLEDStringDraw ("Helicopter Stats", 0, 0);

        // Display the mean ADC value
        usnprintf(lineString, sizeof(lineString), "Mean ADC: %4d ", currentAlt);
        OLEDStringDraw (lineString, 0, 1);
        //Clear Yaw line
        OLEDStringDraw ("                ", 0, 2);

    } else {
        // Clear display
        OrbitOledClear();
    }

}

int32_t getAltPercent (uint16_t baseAltitude, int32_t altitude)
{
    // Calculate the altitude as a percentage (integer math)
    int32_t delta = baseAltitude - altitude; // Difference from the baseline
    int32_t altPercentage = (delta * 100) / ADC_STEP_FOR_1V; // Scale before division

    return altPercentage;
}

int32_t getYawDegree(int32_t currentYaw)
{
    return (currentYaw * 360) / YAW_STEPS;
}


void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}


//OLD FUNCTIONS
//*****************************************************************************
//
// Function to display the mean ADC value (10-bit value, note) and sample count.
//
//*****************************************************************************
/*void
displayMeanVal(uint16_t meanVal, uint32_t count)
{
    char string[17];  // 16 characters across the display

    OLEDStringDraw ("ADC demo 1", 0, 0);

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf (string, sizeof(string), "Mean ADC = %4d", meanVal);
    // Update line on display.
    OLEDStringDraw (string, 0, 1);

    usnprintf (string, sizeof(string), "Sample # %5d", count);
    OLEDStringDraw (string, 0, 3);
}*/


