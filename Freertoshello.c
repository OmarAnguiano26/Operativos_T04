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
#include "fsl_dspi.h"
#include "text.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Task priorities. */
#define hello_task_PRIORITY1 (configMAX_PRIORITIES - 2)
#define hello_task_PRIORITY2 (configMAX_PRIORITIES - 1)
#define hello_task_PRIORITY3 (configMAX_PRIORITIES - 1)
#define hello_task_PRIORITY4 (configMAX_PRIORITIES - 1)
#define hello_task_PRIORITY5 (configMAX_PRIORITIES - 1)

#define EXAMPLE_DSPI_MASTER_BASEADDR         SPI0
#define DSPI_MASTER_CLK_SRC                  DSPI0_CLK_SRC
#define DSPI_MASTER_CLK_FREQ                 CLOCK_GetFreq(DSPI0_CLK_SRC)
#define EXAMPLE_DSPI_MASTER_PCS_FOR_INIT     kDSPI_Pcs0
#define EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs0
#define EXAMPLE_DSPI_DEALY_COUNT             0xfffffU

#define TRANSFER_SIZE     1U     /*! Transfer dataSize */
#define TRANSFER_BAUDRATE 500000U /*! Transfer baudrate - 500k */

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t g_systickCounter  = 20U;

#define MAX7219_SEG_NUM 1 // The number matrices.
#define MAX7219_BUFFER_SIZE MAX7219_SEG_NUM * 8 // The size of the buffer
uint8_t max7219_buffer[MAX7219_BUFFER_SIZE];
uint8_t max7219_num;
uint8_t *__max7219_buffer;
uint8_t __max7219_buffer_size;

const uint8_t max7219_initseq[]  =
{
			0x09, 0x00,	// Decode-Mode Register, 00 = No decode
			0x0A, 0x01,	// Intensity Register, 0x00 .. 0x0f
			0x0B, 0x07,	// Scan-Limit Register, 0x07 to show all lines
			0x0C, 0x01,	// Shutdown Register, 0x01 = Normal Operation
			0x0F, 0x00,	// Display-Test Register, 0x01, 0x00 = Normal
};

uint8_t pattern_smileyface[] = { // Smile face (:
		0b00111100,
		0b01000010,
		0b10100101,
		0b10000001,
		0b10100101,
		0b10011001,
		0b01000010,
		0b00111100,
};

uint8_t letterG[]={
		0b00011100,
		0b00100010,
		0b01000000,
		0b01000000,
		0b01001110,
		0b01000010,
		0b00100010,
		0b00011100,
};


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void Task_Admin(void *pvParameters);
static void Task_Left(void *pvParameters);
static void Task_Right(void *pvParameters);
static void Task_Forward(void *pvParameters);
static void Task_Backward(void *pvParameters);

TaskHandle_t xHandle_Admin, xHandle_Left, xHandle_Right, xHandle_Forward, xHandle_Backward;

void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

void delay_ms(uint16_t miliseconds)
{
	/* Delay to wait slave is ready */
	if (SysTick_Config(SystemCoreClock / 1000U))
	{
		while (1)
		{
		}
	}
	/* Delay 100 ms */
	g_systickCounter = miliseconds;
	while (g_systickCounter != 0U)
	{
	}
}

/*Address - register where the data will the written*/
/*data - value to send to the matrix */
/*num - number of led matrixes*/
void max7219_word(uint8_t address, uint8_t data, uint8_t num)
{
	dspi_transfer_t masterXfer;
	uint8_t n;
	uint8_t masterTxData[2] = {0U};

	masterTxData[0] = data;
	masterTxData[1] = address;

	for (n = num; n != 0; n--) // Send multiple times for cascaded matrices
	{
		masterXfer.txData      = masterTxData;
		masterXfer.rxData      = NULL;
		masterXfer.dataSize    = sizeof(masterTxData);
		masterXfer.configFlags = kDSPI_MasterCtar0 | EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;

		//send via SPI
		DSPI_MasterTransferBlocking(EXAMPLE_DSPI_MASTER_BASEADDR, &masterXfer);
    }
}

/*Address - register where the data will the written*/
/*ptrdata - pointer to buffer value to send to the matrix */

void max7219_doubleLong(uint8_t address, uint8_t *ptrtodata)
{
	dspi_transfer_t masterXfer;
	uint8_t i;
	uint8_t masterTxData[8] = {0U};

	if(1)
	{
		masterTxData[0] = 0x00;
		masterTxData[1] = address;
		masterTxData[2] = *ptrtodata;
		masterTxData[3] = address;
		masterTxData[4] = 0x00;
		masterTxData[5] = address;
		masterTxData[6] = *ptrtodata;
		masterTxData[7] = address;
	}
	else
	{
		//Fill the SPI array to send.
		for (i=0; i<=6; i+=2)
		{
			masterTxData[i] = *ptrtodata;
			masterTxData[i+1] = address;
			ptrtodata++;
		}
	}

	masterXfer.txData      = masterTxData;
	masterXfer.rxData      = NULL;
	masterXfer.dataSize    = sizeof(masterTxData);
	masterXfer.configFlags = kDSPI_MasterCtar0 | EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;

	//send via SPI
	DSPI_MasterTransferBlocking(EXAMPLE_DSPI_MASTER_BASEADDR, &masterXfer);
}

void max7219_outputbuffer(void)
{
	uint8_t *bufferPtr = NULL;
    uint8_t row;
    uint8_t buffer_seg;

    bufferPtr = max7219_buffer;
    for (row = 1; row <= 8; row++)
    {
    	max7219_doubleLong(row, max7219_buffer);
		bufferPtr++;
		delay_ms(100);
    }
}

void max7219_init(uint8_t num, uint8_t *buffer, uint8_t buffer_size)
{
    uint8_t opcode;
    uint8_t opdata;
    uint8_t i;

    max7219_num = num;
    for (i = 0; i < sizeof (max7219_initseq);)
    {
		opcode = max7219_initseq[i++];
		opdata = max7219_initseq[i++];
		max7219_word(opcode, opdata, max7219_num);
    }

    //Clean the matrix
	for (i = 1; i <= 8; i++)
	{
		max7219_word(i, 0x00, max7219_num);
	}

	__max7219_buffer = buffer;
	__max7219_buffer_size = buffer_size;
}

void max7219_row(uint8_t address, uint8_t data)
{
    if (address >= 1 && address <= 8) max7219_word(address, data, max7219_num);
}

void max7219b_col(uint8_t x, uint8_t data)
{
	if (x < __max7219_buffer_size) __max7219_buffer[x] = data;
}

void max7219b_set(uint8_t x, uint8_t y)
{
    if (x < __max7219_buffer_size) __max7219_buffer[x] |= (1 << y);
}

void max7219b_clr(uint8_t x, uint8_t y)
{
    if (x < __max7219_buffer_size) __max7219_buffer[x] &= ~(1 << y);
}

uint8_t max7219b_get(uint8_t x)
{
    return __max7219_buffer[x];
}

void max7219b_left(void)
{
    memcpy(__max7219_buffer, __max7219_buffer + 1, __max7219_buffer_size - 1);
}

void SPI_init(void)
{
    dspi_master_config_t masterConfig;
	uint32_t srcClock_Hz;

	/* Master config */
    masterConfig.whichCtar                                = kDSPI_Ctar0;
    masterConfig.ctarConfig.baudRate                      = TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.bitsPerFrame                  = 16U*4U; //Creo que tienes que multiplicar este por 4 (el numero de matrix que tienes)
    masterConfig.ctarConfig.cpol                          = kDSPI_ClockPolarityActiveHigh;
    masterConfig.ctarConfig.cpha                          = kDSPI_ClockPhaseFirstEdge;
    masterConfig.ctarConfig.direction                     = kDSPI_MsbFirst;
    masterConfig.ctarConfig.pcsToSckDelayInNanoSec        = 1000000000U / TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec    = 1000000000U / TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;

    masterConfig.whichPcs           = EXAMPLE_DSPI_MASTER_PCS_FOR_INIT;
    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    masterConfig.enableContinuousSCK        = false;
    masterConfig.enableRxFifoOverWrite      = false;
    masterConfig.enableModifiedTimingFormat = false;
    masterConfig.samplePoint                = kDSPI_SckToSin0Clock;

    srcClock_Hz = DSPI_MASTER_CLK_FREQ;
    DSPI_MasterInit(EXAMPLE_DSPI_MASTER_BASEADDR, &masterConfig, srcClock_Hz);
}

void drawFace(void)
{
	int row;
	uint8_t *facePtr = pattern_smileyface;
	for (row = 1; row <= 8; row++)
	{
		//Draw a face
		//max7219_word(row, pattern_smileyface[row-1], 4);
		//max7219_word(row, *facePtr, 4);
		max7219_doubleLong(row, facePtr);
		facePtr++;
		delay_ms(100);
	}
	return;
}

void drawText(void)
{
	int row;
	uint8_t *charPtr = letterG;
	for (row = 1; row <= 8; row++)
	{
		//Draw a face
		//max7219_word(row, pattern_smileyface[row-1], 4);
		//max7219_word(row, *facePtr, 4);
		max7219_doubleLong(row, charPtr);
		charPtr++;
		delay_ms(100);
	}
	return;
}

void cleanMatrix(void)
{
    uint8_t i;
    uint8_t r;
    uint8_t d;

	for (r = 1; r <= 8; r++)
	{
		d = 1;
		for (i = 9; i > 0; i--)
		{
			max7219_row(r, d);
			//PRINTF("Data: 0b%b\n\r",d);
			d = d << 1;
			delay_ms(50);
		}
	}
}


/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
	/* PORTD0 (J2-06) is configured as SPI0_PCS0 (CS) */
    /* PORTD1 (J2-12) is configured as SPI0_SCK  (CLK)*/
    /* PORTD2 (J2-08) is configured as SPI0_SOUT (DIN)*/
    /* PORTD3 (J2-10) is configured as SPI0_SIN  (-)  */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    SPI_init();
    max7219_init(MAX7219_SEG_NUM, max7219_buffer, MAX7219_BUFFER_SIZE);

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
