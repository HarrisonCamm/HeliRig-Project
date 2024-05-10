/*
 * Display.c
 *
 *  Created on: 21/03/2024
 *      Author: hrc48
 */

#include "quadrature.h"

static volatile int32_t yawPosition = 0;


void initQuad (void)
{
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOB);

    GPIOIntRegister(GPIO_PORTB_BASE, GPIOYawHandler);

    GPIOPinTypeGPIOInput (GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_BOTH_EDGES);

    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);


}

int32_t getYawPosition (void)
{
    return yawPosition;
}



// Interrupt handler for GPIO Port B Pins 0 and 1
void GPIOYawHandler(void)
{
    // Variables to keep track of the current and last state of the encoder
    static uint8_t last_state = 0;
    uint8_t state;

    // Read and clear the interrupt status for GPIO Port B
    uint32_t status = GPIOIntStatus(GPIO_PORTB_BASE, true);
    GPIOIntClear(GPIO_PORTB_BASE, status);

    // Read the current state of the pins connected to the encoder (PB0 and PB1)
    state = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Return immediately if there is no change in state
    if (state == last_state) return;

    // Update the yaw position based on the transition between states
    // These transitions are determined based on the expected behavior of a quadrature encoder
    if (((last_state == 0x00) && (state == 0x01)) ||  // Transition from 00 to 01
        ((last_state == 0x01) && (state == 0x03)) ||  // Transition from 01 to 11
        ((last_state == 0x03) && (state == 0x02)) ||  // Transition from 11 to 10
        ((last_state == 0x02) && (state == 0x00))) {  // Transition from 10 to 00
        // Counter-clockwise rotation: decrement yaw position
        yawPosition--;
    } else {
        // Clockwise rotation: Increment yaw position
        yawPosition++;
    }


    // Handle wrap around at 180 degrees
    if (yawPosition > WRAPSTEP) {
        yawPosition = -WRAPSTEP + (yawPosition - WRAPSTEP);
    }
    if (yawPosition < -WRAPSTEP) {
        yawPosition = WRAPSTEP - (yawPosition + WRAPSTEP);
    }

    // Update last_state to the current state for the next interrupt
    last_state = state;

}

