/*
 * pressure_sensor_task.c
 *
 *  Created on: 10 Nov 2022
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


// The stack size for the pressure measurement task.
#define PRESSURESENSORTASKSTACKSIZE        128         // Stack size in words


// Set the address for slave module. This is a 7-bit address sent in the
// following format:
//                      [A6:A5:A4:A3:A2:A1:A0:RS]
//
// A zero in the "RS" position of the first byte means that the master
// transmits (sends) data to the selected slave, and a one in this position
// means that the master receives data from the slave.
#define PS_SLAVE_ADDRESS 0x28

// For getting access to UART and I2C without conflicts with other tasks
extern xSemaphoreHandle g_pUARTSemaphore;
extern xSemaphoreHandle g_pI2CSemaphore;


//Gets the register values for the pressure sensor
int PSReadRegisters(void){

    int infobits;
    int status;
    int pressure;
    int temperature;

    xSemaphoreTake(g_pI2CSemaphore, portMAX_DELAY);
    I2CMasterSlaveAddrSet(I2C0_BASE, PS_SLAVE_ADDRESS, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C0_BASE)){}
    if(I2CMasterErr(I2C0_BASE))
    {
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Error receiving pressure 1\n");
        xSemaphoreGive(g_pUARTSemaphore);
    }

    //Status (2 bits) + Pressure bits (MSB first and 6 bits)
    infobits = I2CMasterDataGet(I2C0_BASE);
    status = infobits / 64;
    pressure = infobits % 64;
    pressure = pressure << 8;



    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C0_BASE)){}
        if(I2CMasterErr(I2C0_BASE))
        {
            xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf("Error receiving pressure 2\n");
            xSemaphoreGive(g_pUARTSemaphore);
        }

    //Pressure bits (8 bits, MSB first)
    infobits = I2CMasterDataGet(I2C0_BASE);
    pressure = pressure + infobits;


    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C0_BASE)){}
        if(I2CMasterErr(I2C0_BASE))
        {
            xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf("Error receiving pressure 3\n");
            xSemaphoreGive(g_pUARTSemaphore);
        }

    //Pressure bits (8 bits, MSB first)
    infobits = I2CMasterDataGet(I2C0_BASE);
    temperature = infobits << 8;


    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C0_BASE)){}
    if(I2CMasterErr(I2C0_BASE)){
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Error receiving pressure 4\n");
        xSemaphoreGive(g_pUARTSemaphore);
    }
    xSemaphoreGive(g_pI2CSemaphore);

    //Temperature lower 3 bits + 5 padding
    infobits = I2CMasterDataGet(I2C0_BASE);
    temperature = temperature + infobits;
    temperature = temperature >> 5;

    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("status: %d|\t",status);
    UARTprintf("Pressure: %d|\t",pressure);
    UARTprintf("Temperature: %d\n",temperature);
    xSemaphoreGive(g_pUARTSemaphore);

    return 0;
}


// This task gets the pressure sensor's reading with status and temperature
static void
PressureSensorTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32PressureSensorDelay = 2500;
    ui16LastTime = xTaskGetTickCount();

    //Infinite loop for the pressure sensor
    while(1)
    {
        PSReadRegisters();
        vTaskDelayUntil(&ui16LastTime, ui32PressureSensorDelay / portTICK_RATE_MS);
    }

}

//*****************************************************************************
//
// Initializes the task.
//
//*****************************************************************************
uint32_t
PressureSensorTaskInit(void)
{
    UARTprintf("\nPressure sensor task init()\n");

    //
    // Create the pressure sensor task.
    //
    if(xTaskCreate(PressureSensorTask, (const portCHAR *)"PressureSensor", PRESSURESENSORTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_PRESSURE_SENSOR_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}



