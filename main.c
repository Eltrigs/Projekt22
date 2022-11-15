//*****************************************************************************
//
// freertos_demo.c - Simple FreeRTOS example.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "led_task.h"
#include "switch_task.h"
#include "pressure_sensor_task.h"
#include "ammeter_task.h"
#include "dcdc_task.h"
#include "temperature_sensor_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "inc/hw_gpio.h"                    // Defines Macros for GPIO hardware
#include "driverlib/interrupt.h"            // Defines and macros for NVIC Controller API of DriverLib
#include "driverlib/timer.h"                // Defines and macros for Timer API of driverLib
#include "driverlib/rom.h"                  // Defines and macros for ROM API of driverLib
#include "driverlib/adc.h"                  // Defines and macros for ADC API of driverLib

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>FreeRTOS Example (freertos_demo)</h1>
//!
//! This application demonstrates the use of FreeRTOS on Launchpad.
//!
//! The application blinks the user-selected LED at a user-selected frequency.
//! To select the LED press the left button and to select the frequency press
//! the right button.  The UART outputs the application status at 115,200 baud,
//! 8-n-1 mode.
//!
//! This application utilizes FreeRTOS to perform the tasks in a concurrent
//! fashion.  The following tasks are created:
//!
//! - An LED task, which blinks the user-selected on-board LED at a
//!   user-selected rate (changed via the buttons).
//!
//! - A Switch task, which monitors the buttons pressed and passes the
//!   information to LED task.
//!
//! In addition to the tasks, this application also uses the following FreeRTOS
//! resources:
//!
//! - A Queue to enable information transfer between tasks.
//!
//! - A Semaphore to guard the resource, UART, from access by multiple tasks at
//!   the same time.
//!
//! - A non-blocking FreeRTOS Delay to put the tasks in blocked state when they
//!   have nothing to do.
//!
//! For additional details on FreeRTOS, refer to the FreeRTOS web page at:
//! http://www.freertos.org/
//
//*****************************************************************************


//*****************************************************************************
//
// The mutex that protects concurrent access of UART and I2C line from multiple tasks.
//
//*****************************************************************************
xSemaphoreHandle g_pUARTSemaphore;
xSemaphoreHandle g_pI2CSemaphore;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}


void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
    UARTprintf("\nUART init done\n");
}

void ConfigureADC(void)
{
    // Set up ADC
    FPULazyStackingEnable();
    FPUEnable();
    //Enable clock for ADC0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Enable hardware averaging on ADC0
    ADCHardwareOversampleConfigure(ADC0_BASE, 64);
    // Configure to use ADC0, sample sequencer 0, processor to trigger sequence and use highest priority
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    // Configure PB4 as Analog Input Pin
    GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4);
    // Configure all 8 steps on sequencer 0 to sample temperature sensor
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH10);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH10);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH10);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH10);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH10);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH10);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH10);
    // Mark as last conversion on sequencer 0 and enable interrupt flag generation on sampling completion
    ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH10|ADC_CTL_IE|ADC_CTL_END);
    // Enable Sequencer 0
    ADCSequenceEnable(ADC0_BASE, 0);
    UARTprintf("\nADC init done\n");
}

void
ConfigureI2C(void)
{
    #if defined(TARGET_IS_TM4C129_RA0) ||                                         \
        defined(TARGET_IS_TM4C129_RA1) ||                                         \
        defined(TARGET_IS_TM4C129_RA2)
        uint32_t ui32SysClock;
    #endif



        //
        // Set the clocking to run directly from the external crystal/oscillator.
        // TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
        // crystal on your board.
        //
    #if defined(TARGET_IS_TM4C129_RA0) ||                                         \
        defined(TARGET_IS_TM4C129_RA1) ||                                         \
        defined(TARGET_IS_TM4C129_RA2)
        ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                           SYSCTL_OSC_MAIN |
                                           SYSCTL_USE_OSC), 25000000);
    #else
        SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);
    #endif

        //
        // The I2C0 peripheral must be enabled before use.
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

        //
        // For this example I2C0 is used with PortB[3:2].  The actual port and
        // pins used may be different on your part, consult the data sheet for
        // more information.  GPIO port B needs to be enabled so these pins can
        // be used.
        // TODO: change this to whichever GPIO port you are using.
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

        //
        // Configure the pin muxing for I2C0 functions on port B2 and B3.
        // This step is not necessary if your part does not support pin muxing.
        // TODO: change this to select the port/pin you are using.
        //
        GPIOPinConfigure(GPIO_PB2_I2C0SCL);
        GPIOPinConfigure(GPIO_PB3_I2C0SDA);

        //
        // Select the I2C function for these pins.  This function will also
        // configure the GPIO pins pins for I2C operation, setting them to
        // open-drain operation with weak pull-ups.  Consult the data sheet
        // to see which functions are allocated per pin.
        // TODO: change this to select the port/pin you are using.
        //
        GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
        GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

        //
        // Enable loopback mode.  Loopback mode is a built in feature that is
        // useful for debugging I2C operations.  It internally connects the I2C
        // master and slave terminals, which effectively let's you send data as
        // a master and receive data as a slave.
        // NOTE: For external I2C operation you will need to use external pullups
        // that are stronger than the internal pullups.  Refer to the datasheet for
        // more information.
        //
        //I2CLoopbackEnable(I2C0_BASE);

        //
        // Enable and initialize the I2C0 master module.  Use the system clock for
        // the I2C0 module.  The last parameter sets the I2C data transfer rate.
        // If false the data rate is set to 100kbps and if true the data rate will
        // be set to 400kbps.  For this example we will use a data rate of 100kbps.
        //
    #if defined(TARGET_IS_TM4C129_RA0) ||                                         \
        defined(TARGET_IS_TM4C129_RA1) ||                                         \
        defined(TARGET_IS_TM4C129_RA2)
        I2CMasterInitExpClk(I2C0_BASE, ui32SysClock, false);
    #else
        I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
    #endif
    UARTprintf("\nI2C init done\n");
}

//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int
main(void)
{

    // Set the clocking to run at 50 MHz from the PLL.
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    ConfigureUART();
    ConfigureADC();
    ConfigureI2C();

    // Print introduction.
    UARTprintf("\n\nWelcome to the likely crash of a EK-TM4C123GXL FreeRTOS program!\n");


    // Create a mutex to guard the UART and I2C.
    g_pUARTSemaphore = xSemaphoreCreateMutex();
    g_pI2CSemaphore = xSemaphoreCreateMutex();

    //Create all tasks
    if(LEDTaskInit() != 0)
    {
        while(1){}
    }

    if(SwitchTaskInit() != 0)
    {
        while(1){}
    }

    if(PressureSensorTaskInit() != 0)
    {
        while(1){}
    }

   if(AmmeterTaskInit() != 0)
    {
       while(1){}
    }

    if(DCDCTaskInit() != 0)
    {
        while(1){}
    }

    if(TemperatureSensorTaskInit() != 0)
    {
        while(1){}
    }

    // Start the scheduler.  This should not return.
    vTaskStartScheduler();

    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    while(1)
    {
    }
}
