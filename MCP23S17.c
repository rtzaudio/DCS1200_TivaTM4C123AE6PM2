/* ============================================================================
 *
 * DCS-1200 Digital Channel Selector for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2021, RTZ Professional Audio, LLC
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Professional Audio, LLC
 *
 * ============================================================================
 *
 * Copyright (c) 2014, Texas Instruments Incorporated
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
 * ============================================================================ */

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Assert.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/gates/GateMutex.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

/* Generic Includes */
#include <file.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

/* XDCtools Header files */
#include "Board.h"
#include "MCP23S17.h"

/*****************************************************************************
 * Default Register Configuration Data (all outputs)
 *****************************************************************************/

/* Default MCP23S17SO configuration - port A & B all outputs */

static MCP23S17_InitData initData[] = {
    { MCP_IOCONA, C_SEQOP },			/* Configure for byte mode */
    { MCP_IOCONB, C_SEQOP },            /* Configure for byte mode */
    { MCP_IODIRA, 0x00    },    		/* Port A - all outputs */
    { MCP_IODIRB, 0x00    },			/* Port B - all outputs */
};

/* Default MCP23S17 parameters structure */

const MCP23S17_Params MCP23S17_defaultParams = {
    initData,
    sizeof(initData)/sizeof(MCP23S17_InitData)
};

/*****************************************************************************
 * Initialize default driver parameters
 *****************************************************************************/

Void MCP23S17_Params_init(MCP23S17_Params *params)
{
    Assert_isTrue(params != NULL, NULL);

    *params = MCP23S17_defaultParams;
}

/*****************************************************************************
 * Construct the driver object
 *****************************************************************************/

MCP23S17_Handle MCP23S17_construct(
        MCP23S17_Object *obj,
        SPI_Handle spiHandle,
        uint32_t gpioCSIndex,
        MCP23S17_Params *params
        )
{
    /* Initialize the object's fields */
    obj->spiHandle = spiHandle;
    obj->gpioCS    = gpioCSIndex;

    GateMutex_construct(&(obj->gate), NULL);

    return (MCP23S17_Handle)obj;
}

/*****************************************************************************
 * Destruct the driver object
 *****************************************************************************/

Void MCP23S17_destruct(MCP23S17_Handle handle)
{
    Assert_isTrue((handle != NULL), NULL);

    GateMutex_destruct(&(handle->gate));
}

/*****************************************************************************
 * Create Handle to I/O expander and initialize it
 *****************************************************************************/

MCP23S17_Handle MCP23S17_create(
        SPI_Handle spiHandle,
        uint32_t gpioCSIndex,
        MCP23S17_Params *params
        )
{
    uint32_t i, initCount;
    MCP23S17_Handle handle;
    MCP23S17_InitData* initData;
    Error_Block eb;

    Error_init(&eb);

    handle = Memory_alloc(NULL, sizeof(MCP23S17_Object), NULL, &eb);

    if (handle == NULL)
        return (NULL);

    handle = MCP23S17_construct(handle, spiHandle, gpioCSIndex, params);

    if (handle != NULL)
    {
        /* Initialize the I/O expander */

        initData  = params->initData;
        initCount = params->initDataCount;

        for (i=0; i < initCount; i++)
        {
            /* Send the register config byte */
            MCP23S17_write(handle, initData->addr, initData->data);

            /* Increment to next element in register config data table */
            ++initData;
        }
    }

    return handle;
}

/*****************************************************************************
 * Destruct and free memory
 *****************************************************************************/

Void MCP23S17_delete(MCP23S17_Handle handle)
{
    MCP23S17_destruct(handle);

    Memory_free(NULL, handle, sizeof(MCP23S17_Object));
}

/*****************************************************************************
 * Write a register command byte to MCP23S17 expansion I/O controller.
 *****************************************************************************/

bool MCP23S17_write(
	MCP23S17_Handle	handle,
    uint8_t ucRegAddr,
    uint8_t ucData
    )
{
    bool success;
	uint8_t txBuffer[3];
	uint8_t rxBuffer[3];
	SPI_Transaction transaction;

	txBuffer[0] = 0x40;			/* write opcode */
	txBuffer[1] = ucRegAddr;	/* register address */
	txBuffer[2] = ucData;		/* register data */

	/* Initialize opcode transaction structure */
	transaction.count = 3;
	transaction.txBuf = (Ptr)&txBuffer;
	transaction.rxBuf = (Ptr)&rxBuffer;

	/* Hold SPI chip select low */
	GPIO_write(handle->gpioCS, PIN_LOW);

	/* Initiate SPI transfer of opcode */

	success = SPI_transfer(handle->spiHandle, &transaction);

	if (!success)
	{
	    System_printf("Unsuccessful SPI transfer to MCP23S17");
	}

	/* Release SPI chip select */
	GPIO_write(handle->gpioCS, PIN_HIGH);

	return success;
}

/*****************************************************************************
 * Read a register command byte from MCP23S17 expansion I/O controller.
 *****************************************************************************/

bool MCP23S17_read(
	MCP23S17_Handle	handle,
    uint8_t ucRegAddr,
    uint8_t* pucData
    )
{
    bool success;
	uint8_t txBuffer[3];
	uint8_t rxBuffer[3];
	SPI_Transaction transaction;

	txBuffer[0] = 0x41;			/* read opcode */
	txBuffer[1] = ucRegAddr;	/* register address */
	txBuffer[2] = 0;			/* dummy byte */

	/* Initialize opcode transaction structure */
	transaction.count = 3;
	transaction.txBuf = (Ptr)&txBuffer;
	transaction.rxBuf = (Ptr)&rxBuffer;

	/* Hold SPI chip select low */
	GPIO_write(handle->gpioCS, PIN_LOW);

	/* Initiate SPI transfer of opcode */
    success = SPI_transfer(handle->spiHandle, &transaction);

    if (!success)
	{
	    System_printf("Unsuccessful SPI transfer to MCP23S17");
	}

	/* Release SPI chip select */
	GPIO_write(handle->gpioCS, PIN_HIGH);

	/* Return the register data byte */
	*pucData = rxBuffer[2];

	return success;
}

// End-Of-File
