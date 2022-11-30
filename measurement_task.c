/*
 * measurement_task.c
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



// The stack size for the measurement task.
#define MEASUREMENTTASKSTACKSIZE        4096        // Stack size in words


// Set the address for slave module. This is a 7-bit address sent in the
// following format:
//                      [A6:A5:A4:A3:A2:A1:A0:RS]
//
// A zero in the "RS" position of the first byte means that the master
// transmits (sends) data to the selected slave, and a one in this position
// means that the master receives data from the slave.
#define PS_SLAVE_ADDRESS 0x28
#define DCDC_SLAVE_ADDRESS 0x74
#define AMMETER_SLAVE_ADDRESS 0x4D

// For getting access to UART and I2C without conflicts with other tasks
extern xSemaphoreHandle g_pUARTSemaphore;
extern xSemaphoreHandle g_pI2CSemaphore;

// For setting global variables from raw measurements
extern uint32_t pressureStatus;
extern uint32_t pressureRaw;
extern uint32_t pressureTempRaw;
extern uint32_t temperatureRaw;
extern uint32_t ammeterRaw;

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
    UARTprintf("Temperature: %d\t",temperature);
    xSemaphoreGive(g_pUARTSemaphore);

    pressureStatus = status;
    pressureRaw = pressure;
    pressureTempRaw = temperature;


    return 0;
}

//Gets the register values for the pressure sensor
int TSReadADC(void){

    uint32_t ui32ADC0Value[8];          // Array to store the ADC values
    uint32_t ui32ADCAvg;                // Variable to store the Average of ADC values

    ROM_FPULazyStackingEnable();
    ROM_FPUEnable();

    // Set the clocking to run directly from the crystal.
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
    //Enable clock for ADC0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    //Enable clock for all Ports B
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);


    // Enable hardware averaging on ADC0
    ROM_ADCHardwareOversampleConfigure(ADC0_BASE, 64);
    // Configure to use ADC0, sample sequencer 0, processor to trigger sequence and use highest priority
    ROM_ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    // Configure PB4 as Analog Input Pin
    ROM_GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4);
    // Configure all 8 steps on sequencer 0 to sample temperature sensor
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH10);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH10);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH10);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH10);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH10);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH10);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH10);
    // Mark as last conversion on sequencer 0 and enable interrupt flag generation on sampling completion
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH10|ADC_CTL_IE|ADC_CTL_END);
    // Enable Sequencer 0
    ROM_ADCSequenceEnable(ADC0_BASE, 0);
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
    UARTprintf("average temperature value: %d|\t",ui32ADCAvg);
    xSemaphoreGive(g_pUARTSemaphore);

    temperatureRaw = ui32ADCAvg;

    return 0;
}


int WriteDCDCRegister(uint32_t reg, uint32_t data){
    xSemaphoreTake(g_pI2CSemaphore, portMAX_DELAY);
    I2CMasterSlaveAddrSet(I2C0_BASE, DCDC_SLAVE_ADDRESS, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));
    UARTprintf("Sent register to be modified\n");
    if(I2CMasterErr(I2C0_BASE)){
        UARTprintf("Error transmitting DCDC write 1\n");
    }
    I2CMasterDataPut(I2C0_BASE, data);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C0_BASE));

    UARTprintf("Sent value to selected register\n");

    if(I2CMasterErr(I2C0_BASE)){
        UARTprintf("Error transmitting DCDC write 2\n");
    }
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_STOP);
    xSemaphoreGive(g_pI2CSemaphore);
    return 0;


}

int DCDCReadRegisters(void){
    int i;
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

    return 0;
}


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
    //DCDCReadRegisters();
    return 0;
}

//Gets the register values for the ammeter
int AmmeterReadRegisters(void){

    int infobits;
    int result;

    I2CMasterSlaveAddrSet(I2C0_BASE, AMMETER_SLAVE_ADDRESS, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C0_BASE)){}
    if(I2CMasterErr(I2C0_BASE))
    {
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Error receiving current 1\n");
        xSemaphoreGive(g_pUARTSemaphore);
    }

    infobits = I2CMasterDataGet(I2C0_BASE);
    result = infobits << 8;

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C0_BASE)){}
    if(I2CMasterErr(I2C0_BASE)){
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Error receiving pressure 4\n");
        xSemaphoreGive(g_pUARTSemaphore);
    }



    infobits = I2CMasterDataGet(I2C0_BASE);
    result = result + infobits;

    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("Ammeter current: %d|\t\n",result);
    xSemaphoreGive(g_pUARTSemaphore);

    ammeterRaw = result;

    return 0;
}

// This task gets the pressure sensor's reading with status and temperature
static void
MeasurementTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32MeasurementDelay = 5000;
    ui16LastTime = xTaskGetTickCount();

    //Infinite loop for the measurements
    while(1)
    {
        PSReadRegisters();
        TSReadADC();
        AmmeterReadRegisters();
        vTaskDelayUntil(&ui16LastTime, ui32MeasurementDelay / portTICK_RATE_MS);
    }

}

//*****************************************************************************
//
// Initializes the task.
//
//*****************************************************************************
uint32_t
MeasurementTaskInit(void)
{
    UARTprintf("\nMeasurement task init()\n");
    //DCDCTaskInit();
    //
    // Create the measurements task.
    //
    if(xTaskCreate(MeasurementTask, (const portCHAR *)"Measurements", MEASUREMENTTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_MEASUREMENT_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}


