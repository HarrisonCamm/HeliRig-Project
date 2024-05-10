/*
 * pwmRotor.c
 *
 *  Created on: 7/05/2024
 *      Author: hrc48
 */

#include <pwmRotor.h>


static float KPMvar = 1;

static int16_t altSetPoint = 0;
static int16_t yawSetPoint = 0;

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


/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
void
setDUTY (uint32_t mainDuty, uint32_t tailDuty)
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


    int32_t control = P + (dI + I) + D + GRAVITY;

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
    altSetPoint += ALT_STEP;
    if (altSetPoint > MAX_ALT){
        altSetPoint = MAX_ALT;
    }
}


void decAlt (void) {
    altSetPoint -= ALT_STEP;
    if (altSetPoint < MIN_ALT){
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
