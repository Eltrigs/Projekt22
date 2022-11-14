/*
 * pressure_sensor_task.c
 *
 *  Created on: 10 Nov 2022
 *      Author: marek
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
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


// The stack size for the temperature measurement task.
#define TEMPERATURE_SENSOR_TASK_STACK_SIZE        128         // Stack size in words


// Set the address for slave module. This is a 7-bit address sent in the
// following format:
//                      [A6:A5:A4:A3:A2:A1:A0:RS]
//
// A zero in the "RS" position of the first byte means that the master
// transmits (sends) data to the selected slave, and a one in this position
// means that the master receives data from the slave.
#define PS_SLAVE_ADDRESS 0x28

// For getting access to UART without conflicts with other tasks
extern xSemaphoreHandle g_pUARTSemaphore;


//Gets the register values for the pressure sensor
int TSReadADC(void){

    uint32_t ui32ADC0Value[8];          // Array to store the ADC values
    uint32_t ui32ADCAvg;                // Variable to store the Average of ADC values
    // Clear the ADC Interrupt (if any generated) for Sequencer 0
    ROM_ADCIntClear(ADC0_BASE, 0);
    // Trigger the ADC Sampling for Sequencer 0
    ROM_ADCProcessorTrigger(ADC0_BASE, 0);
    // Wait the program till the conversion isn't complete
    while(!ROM_ADCIntStatus(ADC0_BASE, 0, false));
    // Store the values in sequencer 0 of ADC0 to an Array
    ROM_ADCSequenceDataGet(ADC0_BASE, 0, ui32ADC0Value);
    // Calculate the Average of the Readings
    ui32ADCAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3]
          + ui32ADC0Value[4] + ui32ADC0Value[5] + ui32ADC0Value[6] + ui32ADC0Value[7] + 4)/8;

    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("Average ADC value on temperature sensor: %d\n", ui32ADCAvg);
    xSemaphoreGive(g_pUARTSemaphore);

    return 0;
}


// This task gets the pressure sensor's reading with status and temperature
static void
TemperatureSensorTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32TemperatureSensorDelay = 2500;
    ui16LastTime = xTaskGetTickCount();

    //Infinite loop for the temperature sensor
    while(1)
    {
        UARTprintf("\nTemperature sensor task run 1()\n");
        TSReadADC();
        UARTprintf("\nTemperature sensor task run 2()\n");
        vTaskDelayUntil(&ui16LastTime, ui32TemperatureSensorDelay / portTICK_RATE_MS);
    }

}

//*****************************************************************************
//
// Initializes the task.
//
//*****************************************************************************
uint32_t
TemperatureSensorTaskInit(void)
{

    UARTprintf("\nTemperature sensor task init()\n");

    //
    // Create the temperature sensor task.
    //
    if(xTaskCreate(TemperatureSensorTask, (const portCHAR *)"TemperatureSensor", TEMPERATURE_SENSOR_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_TEMPERATURE_SENSOR_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}
