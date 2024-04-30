/*
 * Display.c
 *
 *  Created on: 21/03/2024
 *      Author: hrc48
 */

#include "display.h"


void displayWrite(uint16_t baseAlt, uint16_t currentAlt, int32_t currentYaw, enum DisplayMode displayCycle) {

    // Get Altitude as a percentage
    int32_t altPercentage = getAltPercent(baseAlt, currentAlt);

    // Get yaw as a degree
    int32_t yawDegree = getYawDegree(currentYaw);

    char lineString[17]; // 16 characters across the display

    switch (displayCycle) {
        case PROCESSED:
        {
            OLEDStringDraw ("Helicopter Stats", 0, 0);

            // Display ADC input as a height percentage
            usnprintf(lineString, sizeof(lineString), "Altitude: %3d%% ", altPercentage);
            OLEDStringDraw (lineString, 0, 1);

            // Calculate Yaw decimal point using modulo
            int32_t yawInt = yawDegree / SCALE_BY_100;
            int32_t yawDecimal = yawDegree % SCALE_BY_100;

            // Prevent the decimal portion from displaying as negative
            if (yawDecimal < 0) {
                yawDecimal *= -1;
            }

            //Display yaw in degrees to 2dp
            usnprintf(lineString, sizeof(lineString), "Yaw(deg):%d.%d   ", yawInt, yawDecimal);
            OLEDStringDraw (lineString, 0, 2);

            break;
        }
        case RAW:
        {
            OLEDStringDraw ("Helicopter Stats", 0, 0);

            // Display the mean ADC value
            usnprintf(lineString, sizeof(lineString), "Mean ADC: %4d ", currentAlt);
            OLEDStringDraw (lineString, 0, 1);
            //Clear Yaw line
            OLEDStringDraw ("                ", 0, 2);

            break;
        }
        case DISPLAY_OFF:
        {
            OrbitOledClear();
            break;
        }
        default:
            break;

    }

}

int32_t getAltPercent (uint16_t baseAltitude, int32_t altitude)
{
    // Calculate the altitude as a percentage (integer math)
    int32_t delta = baseAltitude - altitude; // Difference from the baseline
    int32_t altPercentage = (delta * SCALE_BY_100) / ADC_STEP_FOR_1V; // Scale by 100 before division to include 2 decimal point for modulo division

    return altPercentage;
}

int32_t getYawDegree(int32_t currentYaw)
{
    return (currentYaw * DEG_REV * SCALE_BY_100) / YAW_STEPS;
}


void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}



