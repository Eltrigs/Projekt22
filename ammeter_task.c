/*
 * ammeter_task.c
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


// The stack size for the current measurement task.
#define AMMETER_TASK_STACK_SIZE        128         // Stack size in words


// Set the address for slave module. This is a 7-bit address sent in the
// following format:
//                      [A6:A5:A4:A3:A2:A1:A0:RS]
//
// A zero in the "RS" position of the first byte means that the master
// transmits (sends) data to the selected slave, and a one in this position
// means that the master receives data from the slave.
#define AMMETER_SLAVE_ADDRESS 0x4D

// For getting access to UART and I2C without conflicts with other tasks
extern xSemaphoreHandle g_pUARTSemaphore;
extern xSemaphoreHandle g_pI2CSemaphore;


//Gets the register values for the ammeter
int AmmeterReadRegisters(void){

    int result = 0;

    xSemaphoreTake(g_pI2CSemaphore, portMAX_DELAY);
    I2CMasterSlaveAddrSet(I2C0_BASE, AMMETER_SLAVE_ADDRESS, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C0_BASE));
    if(I2CMasterErr(I2C0_BASE))
    {
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Error transmitting current 1\n");
        xSemaphoreGive(g_pUARTSemaphore);
    }
    result = I2CMasterDataGet(I2C0_BASE) << 4;


    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C0_BASE));
    if(I2CMasterErr(I2C0_BASE))
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Error transmitting current 2\n");
        xSemaphoreGive(g_pUARTSemaphore);
    xSemaphoreGive(g_pI2CSemaphore);


    result = result + I2CMasterDataGet(I2C0_BASE);
    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("Ammeter current: %d\n", result);
    xSemaphoreGive(g_pUARTSemaphore);

    return 0;
}


// This task gets the ammeter's reading with status and temperature
static void
AmmeterTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32AmmeterDelay = 2500;
    ui16LastTime = xTaskGetTickCount();

    //Infinite loop for the current sensor
    while(1)
    {
        AmmeterReadRegisters();
        vTaskDelayUntil(&ui16LastTime, ui32AmmeterDelay / portTICK_RATE_MS);
    }

}

//*****************************************************************************
//
// Initializes the task.
//
//*****************************************************************************
uint32_t
AmmeterTaskInit(void)
{

    UARTprintf("\n ammeter task init()\n");

    //
    // Create the task.
    //
    if(xTaskCreate(AmmeterTask, (const portCHAR *)"Ammeter", AMMETER_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_AMMETER_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}

