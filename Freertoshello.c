/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Task priorities. */
#define hello_task_PRIORITY1 (configMAX_PRIORITIES - 2)
#define hello_task_PRIORITY2 (configMAX_PRIORITIES - 1)
#define hello_task_PRIORITY3 (configMAX_PRIORITIES - 1)
#define hello_task_PRIORITY4 (configMAX_PRIORITIES - 1)
#define hello_task_PRIORITY5 (configMAX_PRIORITIES - 1)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void Task_Admin(void *pvParameters);
static void Task_Left(void *pvParameters);
static void Task_Right(void *pvParameters);
static void Task_Forward(void *pvParameters);
static void Task_Backward(void *pvParameters);

TaskHandle_t xHandle_Admin, xHandle_Left, xHandle_Right, xHandle_Forward, xHandle_Backward;


/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    if (xTaskCreate(Task_Admin, "Admin", configMINIMAL_STACK_SIZE + 100, NULL, hello_task_PRIORITY1, &xHandle_Admin) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }
    if (xTaskCreate(Task_Left, "Left", configMINIMAL_STACK_SIZE + 100, NULL, hello_task_PRIORITY2, &xHandle_Left) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }
    if (xTaskCreate(Task_Right, "Right", configMINIMAL_STACK_SIZE + 100, NULL, hello_task_PRIORITY3, &xHandle_Right) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }
    if (xTaskCreate(Task_Forward, "Forward", configMINIMAL_STACK_SIZE + 100, NULL, hello_task_PRIORITY4, &xHandle_Forward) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }
    if (xTaskCreate(Task_Backward, "Backward", configMINIMAL_STACK_SIZE + 100, NULL, hello_task_PRIORITY5, &xHandle_Backward) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void Task_Admin(void *pvParameters)
{
	/**
	 * Left = 1
	 * Right = 2
	 * Forward = 3
	 * Backward = 4
	 * */
	uint8_t cmd[] = {1,1,2,3,4,4,4,4,2,2,0};
	int i = 0;
    for (;;)
    {
    	while(cmd[i] != 0)
    	{
    		if(cmd[i] == 1)
    		{
    			/*Left*/
    			vTaskResume(xHandle_Left);
    		}
    		if(cmd[i] == 2)
    		{
    			/*Right*/
    			vTaskResume(xHandle_Right);
    		}
    		if(cmd[i] == 3)
    		{
    			/*Forward*/
    			vTaskResume(xHandle_Forward);
    		}
    		if(cmd[i] == 4)
    		{
    			/*Backward*/
    			vTaskResume(xHandle_Backward);
    		}
    		i++;
    		//vTaskDelay(2000 / portTICK_PERIOD_MS);
    	}
        PRINTF("Last command executed\r\n");
        vTaskSuspend(NULL);
    }
}
static void Task_Left(void *pvParameters)
{
    for (;;)
    {
    	vTaskSuspend(NULL);
        PRINTF("Left\r\n");
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        for(int i = 0; i < 100000;i++){}
    }
}
static void Task_Right(void *pvParameters)
{
    for (;;)
    {
    	vTaskSuspend(NULL);
        PRINTF("Right\r\n");
       // vTaskDelay(2000 / portTICK_PERIOD_MS);
        for(int i = 0; i < 100000;i++){}
    }
}static void Task_Forward(void *pvParameters)
{
    for (;;)
    {
    	vTaskSuspend(NULL);
        PRINTF("Forward\r\n");
       // vTaskDelay(2000 / portTICK_PERIOD_MS);
        for(int i = 0; i < 100000;i++){}
    }
}static void Task_Backward(void *pvParameters)
{
    for (;;)
    {
    	vTaskSuspend(NULL);
        PRINTF("Backward\r\n");
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        for(int i = 0; i < 100000;i++){}
    }
}
