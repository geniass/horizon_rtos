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

#include <stdio.h>
#define STR_BUFFER_SIZE 128

/* Board Header file */
#include "Board.h"

#include "MPU9150.h"
#include "motion.h"
#include "filter/HeaveFilter.h"
#include "USBCDCD.h"

static MPU9150_Handle mpu;
IMU_State imu_state;
IMU_State filt_imu_state;

HeaveFilter g_sAccelZFilter;
HeaveFilter g_sAccelXFilter;
HeaveFilter g_sAccelYFilter;

HeaveFilter g_sGyroXFilter;
HeaveFilter g_sGyroYFilter;
HeaveFilter g_sGyroZFilter;

/*
 *  ======== gpioMPU9150DataReady ========
 *  This GPIO driver callback function is used notify the dataCollectionTaskFxn
 *  that new data is ready to be sampled via the sampleData Semaphore.
 */
void gpioMPU9150DataReady(unsigned int index)
{
    // interrupt is cleared automatically

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
        GPIO_write(Board_LED2, Board_LED_ON);
        System_abort("MPU9150 could not be initialized");
    }

    HeaveFilter_init(&g_sAccelZFilter);
    HeaveFilter_init(&g_sAccelXFilter);
    HeaveFilter_init(&g_sAccelYFilter);

    HeaveFilter_init(&g_sGyroXFilter);
    HeaveFilter_init(&g_sGyroYFilter);
    HeaveFilter_init(&g_sGyroZFilter);

    while (1) {
        Semaphore_pend(sampleData, BIOS_WAIT_FOREVER);

        if (!MPU9150_read(mpu)) {
            GPIO_write(Board_LED2, Board_LED_ON);
            System_abort("Could not extract data registers from the MPU9150");
        }

        MPU9150_Data tmpData;

        MPU9150_getAccelFloat(mpu, &tmpData);
        imu_state.x = tmpData.xFloat;
        imu_state.y = tmpData.yFloat;
        // MPU reports downward acceleration as positive
        imu_state.z = -tmpData.zFloat;

        MPU9150_getGyroFloat(mpu, &tmpData);
        imu_state.pitch = tmpData.xFloat;
        imu_state.roll = tmpData.yFloat;
        imu_state.yaw = tmpData.zFloat;

        HeaveFilter_put(&g_sAccelXFilter, imu_state.x);
        HeaveFilter_put(&g_sAccelYFilter, imu_state.y);
        HeaveFilter_put(&g_sAccelZFilter, imu_state.z);

        HeaveFilter_put(&g_sGyroXFilter, imu_state.roll);
        HeaveFilter_put(&g_sGyroYFilter, imu_state.pitch);
        HeaveFilter_put(&g_sGyroZFilter, imu_state.yaw);

        // TODO: maybe decimate?

        filt_imu_state.x = HeaveFilter_get(&g_sAccelXFilter);
        filt_imu_state.y = HeaveFilter_get(&g_sAccelYFilter);
        filt_imu_state.z = HeaveFilter_get(&g_sAccelZFilter);

        filt_imu_state.roll = HeaveFilter_get(&g_sGyroXFilter);
        filt_imu_state.pitch = HeaveFilter_get(&g_sGyroYFilter);
        filt_imu_state.yaw = HeaveFilter_get(&g_sGyroZFilter);

        Semaphore_post(dataProcessed);
    }
}

/*
 *  ======== transmitFxn ========
 *  Task to transmit serial data.
 *
 *  This task periodically sends data to the USB host once it's connected.
 */
Void transmitFxn(UArg arg0, UArg arg1)
{
    while (true) {
        Log_info0("Waiting for USB CDC...");
        /* Block while the device is NOT connected to the USB */
        USBCDCD_waitForConnect(BIOS_WAIT_FOREVER);
        Log_info0("Connected to USB CDC!");

        Semaphore_pend(dataProcessed, BIOS_WAIT_FOREVER);

//        GPIO_write(Board_LED0, Board_LED_ON);
        GPIO_toggle(Board_LED0);

        const unsigned char buffer[STR_BUFFER_SIZE];
        int len = snprintf((char *) buffer, STR_BUFFER_SIZE, "(%.5f %.5f %.5f %.5f %.5f %.5f)\r\n\0",
                                                               filt_imu_state.x, filt_imu_state.y, filt_imu_state.z,
                                                               filt_imu_state.roll, filt_imu_state.pitch, filt_imu_state.yaw);
        if (len >= STR_BUFFER_SIZE) {
            Log_error1("sprintf wrote %d bytes!", (IArg) len);
        }
        Log_info1("Size: %d", len);
        USBCDCD_sendData(buffer, len, BIOS_WAIT_FOREVER);

        /* Send data periodically */
//        Task_sleep(10);
    }
}

/*
 *  ======== receiveFxn ========
 *  Task to receive serial data.
 *
 *  This task will receive data when data is available and block while the
 *  device is not connected to the USB host or if no data was received.
 */
Void receiveFxn(UArg arg0, UArg arg1)
{
    unsigned int received;
    unsigned char data[32];

    while (true) {

        /* Block while the device is NOT connected to the USB */
        USBCDCD_waitForConnect(BIOS_WAIT_FOREVER);

        received = USBCDCD_receiveData(data, 31, BIOS_WAIT_FOREVER);
        data[received] = '\0';
        if (received) {
            System_printf("Received \"%s\" (%d bytes)\r\n", data, received);
        }

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
     Board_initUSB(Board_USBDEVICE);
    // Board_initWatchdog();
    // Board_initWiFi();

    USBCDCD_init();


    GPIO_write(Board_LED0, Board_LED_OFF);
    GPIO_write(Board_LED1, Board_LED_OFF);
    GPIO_write(Board_LED2, Board_LED_OFF);

    /* install Button callback */
    GPIO_setCallback(Board_MPU9150_INT_PIN, gpioMPU9150DataReady);

    /* Enable interrupts */
    GPIO_enableInt(Board_MPU9150_INT_PIN);

    Log_info0("System started...");

    /* Start BIOS */
    BIOS_start();

    return (0);
}
