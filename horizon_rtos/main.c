/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty_min.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Log.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

/* Board Header file */
#include "Board.h"

#include "MPU9150.h"

static MPU9150_Handle mpu;

/*
 *  ======== gpioMPU9150DataReady ========
 *  This GPIO driver callback function is used notify the dataCollectionTaskFxn
 *  that new data is ready to be sampled via the sampleData Semaphore.
 */
void gpioMPU9150DataReady(unsigned int index)
{
    GPIO_clearInt(Board_MPU9150_INT_PIN);

    Semaphore_post(sampleData);
}

/*
 *  ======== dataCollectionTaskFxn ========
 *  This task simply collects data when it has been notified via the sampleData
 *  Semaphore.
 */
Void dataCollectionTaskFxn(UArg arg0, UArg arg1)
{
    /* Use the 1st instance of MPU9150 */
    mpu = MPU9150_init(0, Board_I2C_MPU9150, I2C_MPU9150_ADDR);
    if (mpu == NULL) {
        System_abort("MPU9150 could not be initialized");
    }

    while (1) {
        Semaphore_pend(sampleData, BIOS_WAIT_FOREVER);

        if (!MPU9150_read(mpu)) {
            GPIO_write(Board_LED2, Board_LED_ON);
            System_abort("Could not extract data registers from the MPU9150");
        }
    }
}

Void loggingTask(UArg arg0, UArg arg1)
{
    static uint16_t u16Count = 0;
    MPU9150_Data    tmpData;

    while (1) {
        if (u16Count++ >= 10) {
            GPIO_toggle(Board_LED1);
            u16Count = 0;
        }

        MPU9150_getAccelFloat(mpu, &tmpData);
        Log_info4("Accel %d: X:%f Y:%f Z:%f\r\n", u16Count, floatToArg(tmpData.xFloat / 9.81f), floatToArg(tmpData.yFloat / 9.81f), floatToArg(tmpData.zFloat / 9.81f));
        Task_sleep(100);
    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initI2C();
    // Board_initSDSPI();
    // Board_initSPI();
    // Board_initUART();
    // Board_initUSB(Board_USBDEVICE);
    // Board_initWatchdog();
    // Board_initWiFi();

    /* Turn on user LED  */
    GPIO_write(Board_LED0, Board_LED_OFF);

    /* install Button callback */
    GPIO_setCallback(Board_MPU9150_INT_PIN, gpioMPU9150DataReady);

    /* Enable interrupts */
    GPIO_enableInt(Board_MPU9150_INT_PIN);

    Log_info0("System started...");

    /* Start BIOS */
    BIOS_start();

    return (0);
}
