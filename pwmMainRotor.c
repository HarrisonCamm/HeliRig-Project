/*
 * pwmMainRotor.c
 *
 *  Created on: 7/05/2024
 *      Author: hrc48
 */

# include "pwmMainRotor.h"

void initMainRotor (void) {
    // Set the system clock
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Enable the necessary GPIO port and configure the pin for PWM output
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);  // GPIO Port C for PC5
    GPIOPinConfigure(GPIO_PC5_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);

    // Enable PWM Peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  // PWM Module 0

    // Configure PWM generator
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);

    // Set the PWM frequency
    uint32_t pwmClock = SysCtlClockGet() / 64;  // Using a divisor of 64
    uint32_t loadMain = (pwmClock / PWM_FREQUENCY) - 1;   // Set frequency to 200 Hz
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, loadMain);

    // Set initial duty cycle
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, loadMain * 0.05);  // Initial 5% duty cycle

    // Enable PWM output
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);

    // Start the PWM generator
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

}















