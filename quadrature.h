/*
 * Display.h
 *
 *  Created on: 21/03/2024
 *      Author: hrc48
 */

#include <stdint.h>
#include <stdbool.h>
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




#ifndef QUADRATURE_H_
#define QUADRATURE_H_


void initQuad (void);

int32_t getYawPosition (void);

void GPIOYawHandler (void);



#endif /* DISPLAY_H_ */
