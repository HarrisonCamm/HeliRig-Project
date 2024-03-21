/*
 * Display.c
 *
 *  Created on: 21/03/2024
 *      Author: hrc48
 */

#include "Display.h"


void displayAltitude(uint16_t baseAltitude, uint16_t currentMean, uint8_t displayCycle) {

    // Calculate the altitude as a percentage (integer math)
    int32_t delta = baseAltitude - currentMean; // Difference from the baseline
    int32_t altitudePercentage = (delta * 100) / ADC_STEP_FOR_1V; // Scale before division


    char string[17]; // 16 characters across the display
    if (displayCycle == PERCENTAGE_ALTITUDE) {
        // Display ADC input as a height percentage
        usnprintf(string, sizeof(string), "Altitude: %3d%% ", altitudePercentage);
        OLEDStringDraw ("Helicopter ADC", 0, 0);
        OLEDStringDraw (string, 0, 1);

    } else if (displayCycle == MEAN_ADC) {
        // Display the mean ADC value
        usnprintf(string, sizeof(string), "Mean ADC: %4d ", currentMean);
        OLEDStringDraw ("Helicopter ADC", 0, 0);
        OLEDStringDraw (string, 0, 1);

    } else {
        // Clear display
        OrbitOledClear();
    }

}


//*****************************************************************************
//
// Function to display the mean ADC value (10-bit value, note) and sample count.
//
//*****************************************************************************
void
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
}




void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}

