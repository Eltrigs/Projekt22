/*
 * printing_task.c
 *
 *  Created on: 27 Nov 2022
 *      Author: marek
 */

// The stack size for the printing task.
#define PRINTINGTASKSTACKSIZE        4096        // Stack size in words

// For using global variables from raw measurements
extern uint32_t pressureStatus;
extern uint32_t pressureRaw;
extern uint32_t pressureTempRaw;
extern uint32_t temperatureRaw;
extern uint32_t ammeterRaw;

static void
PrintingTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32MeasurementDelay = 12500;
    ui16LastTime = xTaskGetTickCount();

    //Infinite loop for printing periodically
    while(1)
    {

        vTaskDelayUntil(&ui16LastTime, ui32MeasurementDelay / portTICK_RATE_MS);
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
    UARTprintf("\nPressure sensor task init()\n");
    //DCDCTaskInit();
    //
    // Create the measurements.
    //
    if(xTaskCreate(PrintingTask, (const portCHAR *)"Printing", PRINTINGTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_PRESSURE_SENSOR_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}



