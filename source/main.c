/******************************************************************************
* File Name:   main.c
*
* Created by Assoc. Prof. Wiroon Sriborrirux
*
* Description: This is the source code for FreeRTOS 10.403 and MQTT Client Example for ModusToolbox.
*
* Related Document: See README.md
*
*How to:
*	  1. create a "manifest.loc" file under "~/User/.modustoolbox/" directory
*	  2. add "https://github.com/iotexpert/mtb2-iotexpert-manifests/raw/master/iotexpert-super-manifest.xml" into a "manifest.loc" file
*
* Ref:
*   FreeRTOS v10.403:-
*   	https://community.element14.com/technologies/embedded/b/blog/posts/psoc-6-and-modustoolbox-create-a-freertos-10-3-project
*   NT Shell:-
*   	https://iotexpert.com/modustoolbox-2-2-template-project-freertos-ntshell/
*   	https://github.com/iotexpert/mtb2-freertos-ntshell-template
*   	https://github.com/iotexpert/middleware-ntshell
*
*******************************************************************************
* Copyright 2020-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/* Header file includes */
#include <blink_task.h>
#include <blink2_task.h>
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "capsense_task.h"

#include "mqtt_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "usrcmd.h"

/* Include serial flash library and QSPI memory configurations only for the
 * kits that require the Wi-Fi firmware to be loaded in external QSPI NOR flash.
 */
#if defined(CY_DEVICE_PSOC6A512K)
#include "cy_serial_flash_qspi.h"
#include "cycfg_qspi_memslot.h"
#endif


int main()
{
    cy_rslt_t result;

    /* Initialize the board support package. */
    result = cybsp_init();
    CY_ASSERT(CY_RSLT_SUCCESS == result);

    /* Initialize the User LED */
       result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
           CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

       /* Initialize the GPIO */
       CY_ASSERT(CY_RSLT_SUCCESS == result);

    /* To avoid compiler warnings. */
       (void) result;

    /* Enable global interrupts. */
    	__enable_irq();

    /* Create the LED Blinking task. */
    xTaskCreate(task_blink2, "blink task", configMINIMAL_STACK_SIZE,
        NULL, configMAX_PRIORITIES - 7, NULL);

    /* Initialize retarget-io to use the debug UART port. */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

#if defined(CY_DEVICE_PSOC6A512K)
    /* Initialize the QSPI serial NOR flash with clock frequency of 50 MHz. */
    const uint32_t bus_frequency = 50000000lu;
    cy_serial_flash_qspi_init(smifMemConfigs[0], CYBSP_QSPI_D0, CYBSP_QSPI_D1,
                                  CYBSP_QSPI_D2, CYBSP_QSPI_D3, NC, NC, NC, NC,
                                  CYBSP_QSPI_SCK, CYBSP_QSPI_SS, bus_frequency);

    /* Enable the XIP mode to get the Wi-Fi firmware from the external flash. */
    cy_serial_flash_qspi_enable_xip(true);
#endif


    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen. */
    printf("\x1b[2J\x1b[;H");

//
//    printf("===============================================================\n");
//#if defined(COMPONENT_CM0P)
//    printf("CE229889 - MQTT Client running on CM0+\n");
//#endif
//
//#if defined(COMPONENT_CM4)
//    printf("CE229889 - MQTT Client running on CM4\n");
//#endif
//    printf("===============================================================\n\n");
//
//    /* Create the MQTT Client task. */
//    xTaskCreate(mqtt_client_task, "MQTT Client task", MQTT_CLIENT_TASK_STACK_SIZE,
//                NULL, MQTT_CLIENT_TASK_PRIORITY, NULL);
//

    // Stack size in WORDs
    // Idle task = priority 0
    xTaskCreate(capsense_task, "CapSense"  ,configMINIMAL_STACK_SIZE*4,
    		NULL, 1, 0);

    // Start BDH Shell
    xTaskCreate(usrcmd_task, "usrcmd_task", configMINIMAL_STACK_SIZE*4,
    		0 /* args */ ,0 /* priority */, 0);

    /* Start the FreeRTOS scheduler. */
    vTaskStartScheduler();


    /* Should never get here. */
    CY_ASSERT(0);
}

/* [] END OF FILE */
