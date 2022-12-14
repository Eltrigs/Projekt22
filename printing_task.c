/*
 * printing_task.c
 *
 *  Created on: 27 Nov 2022
 *      Author: marek
 */
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "drivers/rgb.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/adc.h"                  // Defines and macros for ADC API of driverLib
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "inc/hw_gpio.h"                    // Defines Macros for GPIO hardware
#include "driverlib/interrupt.h"            // Defines and macros for NVIC Controller API of DriverLib
#include "driverlib/timer.h"                // Defines and macros for Timer API of driverLib
#include "driverlib/rom.h"                  // Defines and macros for ROM API of driverLib


// The stack size for the printing task.
#define PRINTINGTASKSTACKSIZE        128        // Stack size in words

// For getting access to UART without conflicts with other tasks
extern xSemaphoreHandle g_pUARTSemaphore;

// To be able to use global variables from data processing
extern uint32_t pressure;
extern uint32_t pressureTemp;
extern uint32_t temperature;
extern uint32_t ammeterCurrent;


int PrintMeasurements()
{
    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("Measurements in SI units:\t|\t");
    UARTprintf("pressure difference: %d Pa\t|\t", pressure);
    UARTprintf("pressureTemp: %d ?C\t|\t", pressureTemp);
    UARTprintf("Temperature: %d ?C\t|\t", temperature);
    UARTprintf("AmmeterCurrent: %d mA\t|\n", ammeterCurrent);
    xSemaphoreGive(g_pUARTSemaphore);
    return 0;
}

static void
PrintingTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32PrintingDelay = 12500;
    ui16LastTime = xTaskGetTickCount();

    //Infinite loop for printing periodically
    while(1)
    {
        PrintMeasurements();
        vTaskDelayUntil(&ui16LastTime, ui32PrintingDelay / portTICK_RATE_MS);
    }

}

//*****************************************************************************
//
// Initializes the task.
//
//*****************************************************************************
uint32_t
PrintingTaskInit(void)
{
    UARTprintf("\nPrinting task init()\n");

    if(xTaskCreate(PrintingTask, (const portCHAR *)"Printing", PRINTINGTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_PRINTING_TASK , NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}
