/*
 * pwmRotor.c
 *
 *  Created on: 7/05/2024
 *      Author: hrc48
 */

#include <pwmRotor.h>


static int16_t altSetPoint = 0;
static int16_t yawSetPoint = 0;

static uint16_t MAX_ALT = 0; //since they're constants, they shouldn't be uppercase...
static uint16_t MIN_ALT = 0;



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


void initAltLimits (uint16_t initLandedADC) {
    //Initialise min and max ADC heights
    MIN_ALT = initLandedADC;
    MAX_ALT = initLandedADC - ADC_STEP_FOR_1V;
    altSetPoint = initLandedADC;
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
    float P = KPM * error;
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
    int16_t error = yawSetPoint - sensor;
    if (error < -224) {
        error = 448 + error;
    } else if (error > 224) {
        error = -448 + error;
    } else if ((error > -224) && (error < 224))  {
        error = error;
    }

    static float dI = 0;
    static int16_t prevSensor = 0;


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

void setAlt (int16_t setPoint) {
    altSetPoint = setPoint;
}

void incYaw (void) {
    yawSetPoint += YAW_STEP;
    //special case
    if (yawSetPoint > 224) {
        yawSetPoint = -224 + (yawSetPoint - 224);
    }
}


void decYaw (void) {
    yawSetPoint -= YAW_STEP;
    //special case
    if (yawSetPoint < -224) {
        yawSetPoint = 224 + (yawSetPoint + 224);
    }
}

void setYaw (int16_t setPoint) {
    yawSetPoint = setPoint;
}

int32_t getAltSet (void) {
    return altSetPoint;
}

int32_t getYawSet (void) {
    return yawSetPoint;
}

uint16_t getMIN_ALT (void) {
    return MIN_ALT;
}


uint16_t getMAX_ALT (void) {
    return MAX_ALT;
}





void PWM_ON (void) {
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}

void PWM_OFF (void) {
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}
