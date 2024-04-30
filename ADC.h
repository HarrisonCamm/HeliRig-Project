/*
 * ADC.h
 *
 *  Created on: 21/03/2024
 *      Author: jwi182
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "circBufT.h"


//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 60
#define SAMPLE_RATE_HZ 500



//*****************************************************************************
// Global variables
//*****************************************************************************
    // Counter for the interrupts


#ifndef ADC_H_
#define ADC_H_

void ADCIntHandler(void);

void initADC (void);

uint16_t getAltMean (void);

void SysTickIntHandler(void);

#endif /* ADC_H_ */
