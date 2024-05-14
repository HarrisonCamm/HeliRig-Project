/*
 * heliState.c
 *
 *  Created on: 12/05/2024
 *      Author: jwi182, hrc48
 */

#include "heliState.h"

// Initialize state
static HelicopterState heliState = LANDED;
static bool landedLock = true;
bool scanFlag = true;

// Corresponding array of strings for helicopter state enum
static char *HELISTATE_STRING[] = {
    "LANDED",
    "TAKING OFF",
    "FLYING",
    "LANDING"
};


void initialiseSwitch (void) {
    // Enable Peripheral Clocks for GPIO Port A (for PA7)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the GPIO pin for the mode slider switch (PA7)
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

}


void initialiseResetButton (void) {
    // Configure the GPIO pin for the virtual reset button
    // Set the direction of the pin as input and enable the pull-up resistor.
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
}

void initialiseYawRef (void) {
    // Configure the GPIO pin for the virtual reset button
    // Set the direction of the pin as input and enable the pull-up resistor.
    GPIOIntRegister(GPIO_PORTC_BASE, yawRefHandler);

    GPIOPinTypeGPIOInput (GPIO_PORTC_BASE, GPIO_PIN_4);

    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_LOW_LEVEL);

    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4);
}

bool readSwitchState(void) {
    // Read the current state of the switch (HIGH = UP)
    return GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7);
}

void readResetButtonState(void) {
    if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == 0) {
        SysCtlReset();
    }
}

void yawRefHandler (void) {
    if (scanFlag) {
        setYawZero();
        setYaw(0);
        scanFlag = false;
    }
    uint32_t status = GPIOIntStatus(GPIO_PORTC_BASE, true);
    GPIOIntClear(GPIO_PORTC_BASE, status);
    GPIOIntDisable(GPIO_PORTC_BASE, GPIO_PIN_4);
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

/********************************************************
 * Function to set the Helicopter state
 ********************************************************/
HelicopterState updateHelicopterState(int32_t currentYaw, uint16_t currentAlt) {
    bool sw1High = readSwitchState();

    switch (heliState) {
        case LANDED:
            if (!sw1High) {
                landedLock = false;
            }

            if (!landedLock && sw1High) {
                heliState = TAKING_OFF;
            } else {
                PWM_OFF();
            }

            break;

        case TAKING_OFF:
            // Activate motors to take off
            // Transition to FLYING after successful takeoff
            PWM_ON();
            if (takeoffComplete(currentYaw, currentAlt)) {
                heliState = FLYING;
            }
            break;

        case FLYING:
            poleButtons();
            if (!sw1High) {
                heliState = LANDING;
            }
            break;

        case LANDING:
            // Deactivate motors to land
            setYaw(0);
            // Remain in LANDING until landing is complete
            if (landingComplete(currentYaw, currentAlt)) {
                heliState = LANDED;
            }
            break;
    }
    return heliState;
}

char* getHeliState (void) {
    return HELISTATE_STRING[heliState];
}


/********************************************************
 * Function to set the Helicopter state
 ********************************************************/
bool landingComplete(int32_t yaw, uint16_t altitude) {
    // Logic to check if landing is complete
    uint16_t minAlt = getMIN_ALT();

    if (yaw >= -YAW_LIMIT && yaw < YAW_LIMIT) {
        setAlt(minAlt);
        if (altitude <= minAlt && altitude >= minAlt - ALT_LAND) {
            return true;
        }
        else {
            return false;
        }

    }
    else {
        return false;
    }
}

bool takeoffComplete (int32_t yaw, uint16_t altitude) {
    uint16_t minAlt = getMIN_ALT();

    setAlt(minAlt - ALT_TAKEOFF_5_PERCENT);
    if (altitude < minAlt - ALT_TAKEOFF_5_PERCENT + ALT_LAND) {
        if (!scanFlag) {
            return true;
        }
        else {
            setYaw(223);
            return false;
        }

    } else {
        return false;
    }


}

