/*
 * dcdc_task.c
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


// The stack size for the dcdc task.
#define DCDC_CONVERTER_STACK_SIZE        128         // Stack size in words


// Set the address for slave module. This is a 7-bit address sent in the
// following format:
//                      [A6:A5:A4:A3:A2:A1:A0:RS]
//
// A zero in the "RS" position of the first byte means that the master
// transmits (sends) data to the selected slave, and a one in this position
// means that the master receives data from the slave.
#define DCDC_SLAVE_ADDRESS 0x74

// For getting access to UART and I2C without conflicts with other tasks
extern xSemaphoreHandle g_pUARTSemaphore;
extern xSemaphoreHandle g_pI2CSemaphore;

int WriteDCDCRegister(uint32_t reg, uint32_t data){
    xSemaphoreTake(g_pI2CSemaphore, portMAX_DELAY);
    I2CMasterSlaveAddrSet(I2C0_BASE, DCDC_SLAVE_ADDRESS, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
     while(I2CMasterBusy(I2C0_BASE));
    //while(I2C0_MCS_R&1){}
     xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
     UARTprintf("sent\n");
     xSemaphoreGive(g_pUARTSemaphore);
    if(I2CMasterErr(I2C0_BASE)){
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Error transmitting DCDC write 1\n");
        xSemaphoreGive(g_pUARTSemaphore);
    }
    I2CMasterDataPut(I2C0_BASE, data);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C0_BASE));

    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("sent\n");
    xSemaphoreGive(g_pUARTSemaphore);

    if(I2CMasterErr(I2C0_BASE)){
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Error transmitting DCDC write 2\n");
        xSemaphoreGive(g_pUARTSemaphore);
    }
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_STOP);
    xSemaphoreGive(g_pI2CSemaphore);
    return 0;


}

int DCDCReadRegisters(void){
    int i;
    xSemaphoreTake(g_pI2CSemaphore, portMAX_DELAY);
    I2CMasterSlaveAddrSet(I2C0_BASE, DCDC_SLAVE_ADDRESS, false);

    I2CMasterDataPut(I2C0_BASE, 0);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));
    if(I2CMasterErr(I2C0_BASE)){
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Error transmitting DCDC 1\n");
        xSemaphoreGive(g_pUARTSemaphore);
    }
    //I2C0_MCS_R &= ~0x14;
    //I2C0_MCS_R |= 11;

    I2CMasterSlaveAddrSet(I2C0_BASE, DCDC_SLAVE_ADDRESS, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C0_BASE));
    if(I2CMasterErr(I2C0_BASE)){
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Error transmitting DCDC 2\n");
        xSemaphoreGive(g_pUARTSemaphore);
    }

    for(i = 0; i<8;i++){
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("register: %d data: %d\n",i, I2CMasterDataGet(I2C0_BASE));
        xSemaphoreGive(g_pUARTSemaphore);

        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C0_BASE))
                    {
                    }
        if(I2CMasterErr(I2C0_BASE)){
            xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf("Error receiving DCDC %d\n", i);
            xSemaphoreGive(g_pUARTSemaphore);
        }


        }
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    xSemaphoreGive(g_pI2CSemaphore);
    return 0;
}


// This task gets the dcdc converter's settings
static void
DCDCTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32DCDCDelay = 2500;
    ui16LastTime = xTaskGetTickCount();

    //Infinite loop for the dcdc converter
    while(1)
    {
        DCDCReadRegisters();
        vTaskDelayUntil(&ui16LastTime, ui32DCDCDelay / portTICK_RATE_MS);
    }

}

//*****************************************************************************
//
// Initializes the task.
//
//*****************************************************************************
uint32_t
DCDCTaskInit(void)
{
    UARTprintf("\nDCDC converter task init()\n");

    //uint8_t Data = [0x00, 0x20, 0x03, 0xE4, 0x01, 0x00, 0xE0, 0x21];
    uint32_t pui8DataTx[8];
    pui8DataTx[0] = 0x20;       // Reference voltage LSB
    pui8DataTx[1] = 0x3;        // Reference voltage MSB
    pui8DataTx[2] = 0xE4;       // Current sense voltage / limit setting, MSB on enables current limit, 7 bits for the voltage
                                // Vsns is voltage setting divided by 2
                                // current is measured across R4, which is 10 milliOhms on the evaluation board
                                // current limit is calculated by Iout = Vsns / Rsns
                                // the default value is 50 mV / 10 mOhm = 5 A
    pui8DataTx[3] = 0x1;        // Slew rate
    pui8DataTx[4] = 0x0;        // Output feedback voltage 4 settings, adjust the output voltage range
    pui8DataTx[5] = 0xE0;       // Cable voltage drop compensation setting
    pui8DataTx[6] = 0xA1;       // Mode control

    //
    // Display the example setup on the console.
    //
    UARTprintf("I2C DCDC configure->");
    UARTprintf("\n   Module = I2C0");
    UARTprintf("\n   Mode = Single Send/Receive");
    UARTprintf("\n   Rate = 100kbps\n\n");

    int i;

    for(i=0;i<7;i++){
        WriteDCDCRegister(i, pui8DataTx[i]);
    }


    //
    // Create the DCDC task.
    //
    if(xTaskCreate(DCDCTask, (const portCHAR *)"DCDC converter", DCDC_CONVERTER_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_DCDC_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}
