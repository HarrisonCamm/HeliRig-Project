/*
 * ADC.h
 *
 *  Created on: 21/03/2024
 *      Author: jwi182
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
#include "circBufT.h"


//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 60
#define SAMPLE_RATE_HZ 500


//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;    // Counter for the interrupts


#ifndef ADC_H_
#define ADC_H_

void ADCIntHandler(void);

void initADC (void);


#endif /* ADC_H_ */
