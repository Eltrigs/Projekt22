/*
 * data_processing_task.c
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
#define DATAPROCESSINGTASKSTACKSIZE        1024        // Stack size in words

// For getting access to UART without conflicts with other tasks
extern xSemaphoreHandle g_pUARTSemaphore;

// For using global variables from raw measurements
extern uint32_t pressureStatus;
extern uint32_t pressureRaw;
extern uint32_t pressureTempRaw;
extern uint32_t temperatureRaw;
extern uint32_t ammeterRaw;

extern int pressure;
extern int pressureTemp;
extern int temperature;
extern int ammeterCurrent;


// Prototypes for functions used in processing data
int convertPressureToSI(uint32_t);
int convertPressureTempToSI(uint32_t);
int convertTemperatureToSI(uint32_t);
int convertAmmeterToSI(uint32_t);

const int tabley[] = {-50,-49,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,1500,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150};
const int tablex[] = {1955,1949,1942,1935,1928,1921,1915,1908,1900,1892,1885,1877,1869,1861,1853,1845,1838,1830,1822,1814,1806,1798,1790,1783,1775,1767,1759,1751,1743,1735,1727,1719,1711,1703,1695,1687,1679,1671,1663,1656,1648,1639,1631,1623,1615,1607,1599,1591,1583,1575,1567,1559,1551,1543,1535,1527,1519,1511,1502,1494,1486,1478,1470,1462,1454,1446,1438,1430,1421,1413,1405,1397,1389,1381,1373,1365,1356,1348,1340,1332,1324,1316,1308,1299,1291,1283,1275,1267,1258,1250,1242,1234,1225,1217,1209,1201,1192,1184,1176,1167,1159,1151,1143,1134,1126,1118,1109,1101,1093,1084,1076,1067,1059,1051,1042,1034,1025,1017,1008,1000,991,983,974,966,957,949,941,932,924,915,907,898,890,881,873,865,856,848,839,831,822,814,805,797,788,779,771,762,754,745,737,728,720,711,702,694,685,677,668,660,651,642,634,625,617,608,599,591,582,573,565,556,547,539,530,521,513,504,495,487,478,469,460,452,443,434,425,416,408,399,390,381,372,363,354,346,337,328,319,310,301};

int ConvertToSI()
{
    pressure = convertPressureToSI(pressureRaw);
    pressureTemp = convertPressureTempToSI(pressureTempRaw);
    temperature = convertTemperatureToSI(temperatureRaw);
    ammeterCurrent = convertAmmeterToSI(ammeterRaw);
    return 0;
}

static void
DataProcessingTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32DataProcessingDelay = 5000;
    ui16LastTime = xTaskGetTickCount();

    //Infinite loop for printing periodically
    while(1)
    {
        ConvertToSI();
        vTaskDelayUntil(&ui16LastTime, ui32DataProcessingDelay / portTICK_RATE_MS);
    }

}

//*****************************************************************************
//
// Initializes the task.
//
//*****************************************************************************
uint32_t
DataProcessingTaskInit(void)
{
    UARTprintf("\nData processing task init()\n\n");

    if(xTaskCreate(DataProcessingTask, (const portCHAR *)"Data Processing", DATAPROCESSINGTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_DATA_PROCESSING_TASK , NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}

int convertPressureToSI(uint32_t pressureRaw)
{
    int result = 123;
    // 1 psi = 6894 Pa
    // range from -5 to +5 psi = -34473.79 to 34473.79 Pa
    // range in bits = 2^11 = 16384
    // assuming int is 32 bits = 4,294,967,296 = -2,147,483,648 to 2,147,483,647
    // result =  68948 Pa / 16384  * (pressureRaw - 8192)= 4.208 Pa * (pressureRaw - 8192)
    // max value is 4208 * 8192 = 34,471,936
    // therefore the maximum result isnt higher or lower than |max_int|

    result = 4208 * (pressureRaw - 8192);
    result = result / 1000;

    return result;
}
int convertPressureTempToSI(uint32_t pressureTempRaw)
{
    int result = 123;
    // result = (pressureTempRaw * 105 / 2048) - 20
    // Temp. max value = 2^11 = 2048

    result = pressureTempRaw * 105;
    result = result / 2048;
    result = result - 20;

    return result;
}
int convertTemperatureToSI(uint32_t temperatureRaw)
{
    int result = 123;
    // result = (8.194 - sqrt( 8.192^2 + 4*0.00262 * (1324 - x)))/(2*(-0.00262)) + 30

    int millivolts = temperatureRaw * 3300;
    millivolts = (millivolts-320) / 4096;

    int i=0;
    for(i=0; i<200; i++)
    {
        if(tablex[i] < millivolts)
        {
            return tabley[i];
        }
    }

    return result;
}
int convertAmmeterToSI(uint32_t ammeterRaw)
{
    int result = 123;
    // Assuming linearity
    // I = -2.8 to 27.7 A -> ammeterRaw = 0 to 4096
    // y = 0.00744629x - 2.8
    // y = (7446 * ammeterRaw - 2800000) / 1000

    result = 7446*ammeterRaw - 2800000;
    result = result / 1000; // in mA
    return result;
}
