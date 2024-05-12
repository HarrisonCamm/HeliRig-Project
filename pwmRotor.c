/*
 * pwmRotor.c
 *
 *  Created on: 7/05/2024
 *      Author: hrc48
 */

#include <pwmRotor.h>


static float KPMvar = 1;

static int16_t altSetPoint = 2900;
static int16_t yawSetPoint = 0;

static uint16_t MAX_ALT = 0;
static uint16_t MIN_ALT = 0;

// Initialize state
static HelicopterState heliState = LANDED;
static bool landedLock = true;

// Corresponding array of strings for helicopter state enum
static char *HELISTATE_STRING[] = {
    "LANDED",
    "TAKING OFF",
    "FLYING",
    "LANDING"
};


/*********************************************************
 * initialisePWM
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 * M1PWM5 (J3, PF1) is used for the tail rotor motor
 *********************************************************/
void
initialisePWM (void)
{
    //System clock
    uint32_t sysClock = SysCtlClockGet();

    //init main
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    uint32_t ui32Period = sysClock / PWM_DIVIDER / PWM_MAIN_FREQ;

    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM, ui32Period * PWM_START_DUTY / 100);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);


    //init Tail
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32PeriodTail = sysClock / PWM_DIVIDER / PWM_TAIL_FREQ;

    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32PeriodTail);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM, ui32PeriodTail * PWM_START_DUTY / 100);


    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}

void initialiseSwitch (void) {
    // Enable Peripheral Clocks for GPIO Port A (for PA7)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the GPIO pin for the mode slider switch (PA7)
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
}

void initAltLimits (uint16_t initLandedADC) {
    //Initialise min and max ADC heights
    MIN_ALT = initLandedADC;
    MAX_ALT = initLandedADC - ADC_STEP_FOR_1V;
}


/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
void
setDuty (uint32_t mainDuty, uint32_t tailDuty)
{
    //System clock
    static uint32_t sysClock = 0;
    if (sysClock == 0) {
        sysClock = SysCtlClockGet();
    }
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32PeriodMain = sysClock / PWM_DIVIDER / PWM_MAIN_FREQ;

    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM, ui32PeriodMain * mainDuty / 100);

    uint32_t ui32PeriodTail = sysClock / PWM_DIVIDER / PWM_TAIL_FREQ;

    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM, ui32PeriodTail * tailDuty / 100);

}


/********************************************************
 * PID controller function for main rotor, returns a duty cycle %
 ********************************************************/

int32_t
controllerMain (uint16_t sensor) {
    static float dI = 0;
    static uint16_t prevSensor = 0;

    float error = altSetPoint - sensor;
    float P = KPMvar * error;
    float I = KIM * error * DELTA_T;
    float D = KDM * (prevSensor - sensor) / DELTA_T;


    int32_t control = GRAVITY - P - (dI + I) - D;

    if (control > PWM_DUTY_MAX) {
        control = PWM_DUTY_MAX;
    } else if (control < PWM_DUTY_MIN) {
        control = PWM_DUTY_MIN;
    } else {
        I = I + dI;
    }
    prevSensor = sensor;
    return control;
}


/********************************************************
 * PID controller function for tail rotor, returns a duty cycle %
 ********************************************************/

int32_t
controllerTail (int32_t mainControl, int16_t sensor) {
    static float dI = 0;
    static int16_t prevSensor = 0;

    float error = yawSetPoint - sensor;
    float P = KPT * error;
    float I = KIT * error * DELTA_T;
    float D = KDT * (prevSensor - sensor) / DELTA_T;

    float coupling = mainControl * KC;

    int32_t control = P + (dI + I) + D + coupling;

    if (control > PWM_DUTY_MAX) {
        control = PWM_DUTY_MAX;
    } else if (control < PWM_DUTY_MIN) {
        control = PWM_DUTY_MIN;
    } else {
        I = I + dI;
    }

    prevSensor = sensor;
    return control;
}

void incKP (void) {
    KPMvar += 1;
}

void decKP (void) {
    KPMvar -= 1;
}


void incAlt (void) {
    altSetPoint -= ALT_STEP;
    if (altSetPoint < MAX_ALT){
        altSetPoint = MAX_ALT;
    }
}


void decAlt (void) {
    altSetPoint += ALT_STEP;
    if (altSetPoint > MIN_ALT){
        altSetPoint = MIN_ALT;
    }
}


void incYaw (void) {
    yawSetPoint += YAW_STEP;
    //special case
}


void decYaw (void) {
    yawSetPoint -= YAW_STEP;
    //special case
}

int32_t getAltSet (void) {
    return altSetPoint;
}

int32_t getYawSet (void) {
    return yawSetPoint;
}

char* getHeliState (void) {
    return HELISTATE_STRING[heliState];
}


/********************************************************
 * Function to set the Helicopter state
 ********************************************************/
void UpdateHelicopterState(int32_t currentYaw, int32_t currentAltitude) {
    bool sw1High = ReadSwitchState();

    switch (heliState) {
        case LANDED:
            if (!sw1High) {
                landedLock = false;
            }
            if (!landedLock && sw1High) {
                heliState = TAKING_OFF;
            } else {
                PWM_OFF();
                altSetPoint = MIN_ALT;
            }
            break;
        case TAKING_OFF:
            // Activate motors to take off
            // Transition to FLYING after successful takeoff
            heliState = FLYING;
            PWM_ON();
            break;
        case FLYING:
            if (!sw1High) {
                heliState = LANDING;
            }
            break;
        case LANDING:
            // Deactivate motors to land
            // Remain in LANDING until landing is complete
            if (landingComplete(currentYaw, currentAltitude)) {
                heliState = LANDED;
            }
            break;
    }
}




bool ReadSwitchState(void) {
    // Read the current state of the switch (HIGH = UP)
    return GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7);
}






bool landingComplete(int32_t yaw, int32_t altitude) {
    // Logic to check if landing is complete

    if (yaw == 0) {

        if(altitude == 0) {
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

void PWM_ON (void) {
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}

void PWM_OFF (void) {
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}
