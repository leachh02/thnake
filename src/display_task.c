/*
 * led_task
 *
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/******************************************************************************
 *
 * vLEDTask turns on the initial LED, configurations the buttons, and creates
 * prvProcessSwitchInputTask which handles processing the ISR result to adjust
 * the LED per the button inputs.
 *
 * prvProcessSwitchInputTask uses semaphore take to wait until it receives a
 * semaphore from the button ISR.  Once it does, then it processes the button
 * pressed.  Each time the SW1 or SW2 button is pressed, the LED index is
 * updated based on which button has been pressed and the corresponding LED
 * lights up.
 *
 * When either user switch SW1 or SW2 on the EK-TM4C1294XL is pressed, an
 * interrupt is generated and the switch pressed is logged in the global
 * variable g_pui32ButtonPressed.  Then the binary semaphore is given to
 * prvProcessSwitchInputTask before yielding to it.  This is an example of
 * using binary semaphores to defer ISR processing to a task.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"
#include "drivers/rtos_hw_drivers.h"

/* Display includes. */
#include "grlib/grlib.h"
#include "grlib/widget.h"
#include "grlib/canvas.h"

#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"

/*-----------------------------------------------------------*/
/*
 * Time stamp global variable.
 */
volatile uint32_t g_ui32TimeStamp = 0;

/*
 * Global variable to log the last GPIO button pressed.
 */
volatile static uint32_t g_pui32ButtonPressed = NULL;

/*
 * The binary semaphore used by the switch ISR & task.
 */
extern SemaphoreHandle_t xButtonSemaphore;

/*
 * data structures for the graphics library
 */
tContext sContext;
tRectangle sRect;
uint32_t g_ui32SysClock;

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvProcessSwitchInputTask(void *pvParameters);

/*
 * Called by main() to do example specific hardware configurations and to
 * create the Process Switch task.
 */
void vLEDTask(void);

/*
 * Hardware configuration for the LEDs.
 */
static void prvConfigureLED(void);

/*
 * Timer configuration
 */
static void prvConfigureHWTimer(void);

/*
 * Hardware configuration for the buttons SW1 and SW2 to generate interrupts.
 */
static void prvConfigureButton(void);
/*-----------------------------------------------------------*/

void vLEDTask(void)
{
    /* Light the initial LED. */
    prvConfigureLED();

    /* Configure the button to generate interrupts. */
    prvConfigureButton();

    /* Configure the hardware timer to run in periodic mode. */
    prvConfigureHWTimer();

    /* Create the task as described in the comments at the top of this file.
     *
     * The xTaskCreate parameters in order are:
     *  - The function that implements the task.
     *  - The text name for the LED Task - for debug only as it is not used by
     *    the kernel.
     *  - The size of the stack to allocate to the task.
     *  - The parameter passed to the task - just to check the functionality.
     *  - The priority assigned to the task.
     *  - The task handle is not required, so NULL is passed. */
    xTaskCreate(prvProcessSwitchInputTask,
                "LED",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    TimerEnable(TIMER0_BASE, TIMER_A);
}

/*-----------------------------------------------------------*/

static void prvConfigureHWTimer(void)
{
    /* The Timer 0 peripheral must be enabled for use. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    /* Configure Timer 0 in full-width periodic mode. */
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    /* Set the Timer 0A load value to run at 5 Hz. */
    TimerLoadSet(TIMER0_BASE, TIMER_A, configCPU_CLOCK_HZ / 5);

    /* Configure the Timer 0A interrupt for timeout. */
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Enable the Timer 0A interrupt in the NVIC. */
    IntEnable(INT_TIMER0A);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}

/*-----------------------------------------------------------*/

void xTimerHandler(void)
{

    /* Clear the hardware interrupt flag for Timer 0A. */
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Move snake position forward every second */
}

/*-----------------------------------------------------------*/

static void prvProcessSwitchInputTask(void *pvParameters)
{
    uint8_t ui8LEDIndex = 0;

    /* Initialise the screen */

    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240),
                                            120000000);

    //
    // Initialize the display driver.
    //
    Kentec320x240x16_SSD2119Init(g_ui32SysClock);

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

    //
    // Fill the top 24 rows of the screen with blue to create the banner.
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 23;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);

    //
    // Put a white box around the banner.
    //
    GrContextForegroundSet(&sContext, ClrWhite);
    GrRectDraw(&sContext, &sRect);

    //
    // Put the application name in the middle of the banner.
    //
    GrContextFontSet(&sContext, &g_sFontCm20);
    GrStringDrawCentered(&sContext, "snake", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 8, 0);
    GrContextForegroundSet(&sContext, ClrBlack);

    //
    // Put a white box below the banner.
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 24;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = GrContextDpyHeightGet(&sContext) - 1;
    GrContextForegroundSet(&sContext, ClrWhite);
    GrRectFill(&sContext, &sRect);

    //
    // Instructions.
    //
    GrContextForegroundSet(&sContext, ClrBlack);
    GrContextFontSet(&sContext, &g_sFontCm14);
    GrStringDrawCentered(&sContext,
                         "1. Press start/stop button to begin stopwatch.", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 32, 0);
    GrStringDrawCentered(&sContext,
                         "2. Press again to pause the stopwatch.", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 48, 0);
    GrStringDrawCentered(&sContext,
                         "a) The reset button reverts the system.", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 64, 0);
    GrStringDrawCentered(&sContext,
                         "b) Pressing the button continues the timer.", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 80, 0);
    GrStringDrawCentered(&sContext,
                         "3. The stopwatch becomes inactive after 60 seconds.", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 96, 0);

    char display_string[20] = "LED x";

    for (;;)
    {
        /* Block until the ISR gives the semaphore. */
        if (xSemaphoreTake(xButtonSemaphore, portMAX_DELAY) == pdPASS)
        {
            /* If the right button is hit, either increment by 1 or reset the
             * index to 0 if it is at 3. */
            if (g_pui32ButtonPressed == USR_SW1)
            {
                if (ui8LEDIndex == 3)
                {
                    ui8LEDIndex = 0;
                }
                else
                {
                    ui8LEDIndex++;
                }
            }
            /* If the left button is hit, either decrement by 1 or reset the
             * index to 3 if it is at 0. */
            else if (g_pui32ButtonPressed == USR_SW2)
            {
                if (ui8LEDIndex == 0)
                {
                    ui8LEDIndex = 3;
                }
                else
                {
                    ui8LEDIndex--;
                }
            }

            /* Turn off all LED's. */
            LEDWrite(LED_D1 | LED_D2 | LED_D3 | LED_D4, 0);

            /* Set the LED based on the updated Index. */
            switch (ui8LEDIndex)
            {
            case 0:
                LEDWrite(LED_D1, LED_D1);
                display_string[4] = '0';
                break;
            case 1:
                LEDWrite(LED_D2, LED_D2);
                display_string[4] = '1';
                break;
            case 2:
                LEDWrite(LED_D3, LED_D3);
                display_string[4] = '2';
                break;
            case 3:
                LEDWrite(LED_D4, LED_D4);
                display_string[4] = '3';
                break;
            }

            //
            // Fill the top 24 rows of the screen with blue to create the banner.
            //
            sRect.i16XMin = 0;
            sRect.i16YMin = 0;
            sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
            sRect.i16YMax = 23;
            GrContextForegroundSet(&sContext, ClrDarkBlue);
            GrRectFill(&sContext, &sRect);

            //
            // Put a white box around the banner.
            //
            GrContextForegroundSet(&sContext, ClrWhite);
            GrRectDraw(&sContext, &sRect);

            /*
            Update the display string based on which LED is on
            */
            GrContextFontSet(&sContext, &g_sFontCm20);
            GrStringDrawCentered(&sContext, display_string, -1, GrContextDpyWidthGet(&sContext) / 2, 8, 0);
        }
    }
}
/*-----------------------------------------------------------*/

static void prvConfigureLED(void)
{
    /* Configure initial LED state.  PinoutSet() has already configured
     * LED I/O. */
    LEDWrite(LED_D1, LED_D1);
}
/*-----------------------------------------------------------*/

static void prvConfigureButton(void)
{
    /* Initialize the LaunchPad Buttons. */
    ButtonsInit();

    /* Configure both switches to trigger an interrupt on a falling edge. */
    GPIOIntTypeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);

    /* Enable the interrupt for LaunchPad GPIO Port in the GPIO peripheral. */
    GPIOIntEnable(BUTTONS_GPIO_BASE, ALL_BUTTONS);

    /* Enable the Port F interrupt in the NVIC. */
    IntEnable(INT_GPIOJ);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}
/*-----------------------------------------------------------*/

void xButtonsHandler(void)
{
    BaseType_t xLEDTaskWoken;
    uint32_t ui32Status;

    /* Initialize the xLEDTaskWoken as pdFALSE.  This is required as the
     * FreeRTOS interrupt safe API will change it if needed should a
     * context switch be required. */
    xLEDTaskWoken = pdFALSE;

    /* Read the buttons interrupt status to find the cause of the interrupt. */
    ui32Status = GPIOIntStatus(BUTTONS_GPIO_BASE, true);

    /* Clear the interrupt. */
    GPIOIntClear(BUTTONS_GPIO_BASE, ui32Status);

    /* Debounce the input with 200ms filter */
    if ((xTaskGetTickCount() - g_ui32TimeStamp) > 200)
    {
        /* Log which button was pressed to trigger the ISR. */
        if ((ui32Status & USR_SW1) == USR_SW1)
        {
            g_pui32ButtonPressed = USR_SW1;
        }
        else if ((ui32Status & USR_SW2) == USR_SW2)
        {
            g_pui32ButtonPressed = USR_SW2;
        }

        /* Give the semaphore to unblock prvProcessSwitchInputTask.  */
        xSemaphoreGiveFromISR(xButtonSemaphore, &xLEDTaskWoken);

        /* This FreeRTOS API call will handle the context switch if it is
         * required or have no effect if that is not needed. */
        portYIELD_FROM_ISR(xLEDTaskWoken);
    }

    /* Update the time stamp. */
    g_ui32TimeStamp = xTaskGetTickCount();
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
    /* This function will be called by each tick interrupt if
        configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
        added here, but the tick hook is called from an interrupt context, so
        code must not attempt to block, and only the interrupt safe FreeRTOS API
        functions can be used (those that end in FromISR()). */

    /* Only the full demo uses the tick hook so there is no code is
        executed here. */
}
