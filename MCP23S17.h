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

#ifndef _MCP23S17_H_
#define _MCP23S17_H_

/*****************************************************************************
 * MCP23017 Register Addresses (IOCON.BANK = 0)
 *****************************************************************************/

#define MCP_IODIRA      0x00        // I/O DIRECTION REGISTER
#define MCP_IODIRB      0x01        // I/O DIRECTION REGISTER
#define MCP_IOPOLA      0x02        // INPUT POLARITY REGISTER
#define MCP_IOPOLB      0x03        // INPUT POLARITY REGISTER
#define MCP_GPINTENA    0x04        // INTERRUPT-ON-CHANGE CONTROL REGISTER
#define MCP_GPINTENB    0x05        // INTERRUPT-ON-CHANGE CONTROL REGISTER
#define MCP_DEFVALA     0x06        // DEFAULT COMPARE REGISTER FOR INT-ON-CHANGE
#define MCP_DEFVALB     0x07        // DEFAULT COMPARE REGISTER FOR INT-ON-CHANGE
#define MCP_INTCONA     0x08        // INTERRUPT CONTROL REGISTER
#define MCP_INTCONB     0x09        // INTERRUPT CONTROL REGISTER
#define MCP_IOCONA      0x0A        // I/O EXPANDER CONFIGURATION REGISTER
#define MCP_IOCONB      0x0B        // I/O EXPANDER CONFIGURATION REGISTER
#define MCP_GPPUA       0x0C        // GPIO PULL-UP RESISTOR REGISTER
#define MCP_GPPUB       0x0D        // GPIO PULL-UP RESISTOR REGISTER
#define MCP_INTFA       0x0E        // INTERRUPT FLAG REGISTER
#define MCP_INTFB       0x0F        // INTERRUPT FLAG REGISTER
#define MCP_INTCAPA     0x10        // INTERRUPT CAPTURED VALUE FOR PORT REGISTER
#define MCP_INTCAPB     0x11        // INTERRUPT CAPTURED VALUE FOR PORT REGISTER
#define MCP_GPIOA       0x12        // GENERAL PURPOSE I/O PORT REGISTER
#define MCP_GPIOB       0x13        // GENERAL PURPOSE I/O PORT REGISTER
#define MCP_OLATA       0x14        // OUTPUT LATCH REGISTER
#define MCP_OLATB       0x15        // OUTPUT LATCH REGISTER

/* IOCON Configuration Register Bits */
#define C_INTPOL        0x02    /* INT output 1=Active-high, 0=Active-low. */
#define C_ODR           0x04    /* INT pin as an open-drain output         */
#define C_HAEN          0x08    /* Hardware address enable (N/A for I2C)   */
#define C_DISSLW        0x10    /* Slew rate disable bit                   */
#define C_SEQOP         0x20    /* Disable address pointer auto-increment  */
#define C_MIRROR        0x40    /* INT A/B pins mirrored                   */
#define C_BANK          0x80    /* port registers are in different banks   */

/*****************************************************************************
 * Driver Data Structures
 *****************************************************************************/

/* MCP23S17 Configuration array data element for initialization */
typedef struct MCP23S17_InitData {
    uint8_t             addr;           /* register address */
    uint8_t             data;           /* config data byte */
} MCP23S17_InitData;

/* MCP23S17 Parameters object points to init data */
typedef struct MCP23S17_Params {
    MCP23S17_InitData*  initData;       /* ptr to init data array  */
    uint32_t            initDataCount;  /* size of init data array */
} MCP23S17_Params;

/* MCP23S17 handle object */
typedef struct MCP23S17_Object {
    SPI_Handle      	spiHandle;      /* Handle for SPI object */
    uint32_t    		gpioCS;         /* Chip select in Board.h */
    GateMutex_Struct    gate;
} MCP23S17_Object;

/* Handle to I/O expander */
typedef MCP23S17_Object* MCP23S17_Handle;

/*****************************************************************************
 * Function Prototypes
 *****************************************************************************/

Void MCP23S17_Params_init(
        MCP23S17_Params *params
        );

MCP23S17_Handle MCP23S17_construct(
        MCP23S17_Object *obj,
        SPI_Handle spiHandle,
        uint32_t gpioCSIndex,
        MCP23S17_Params *params
        );

Void MCP23S17_destruct(
        MCP23S17_Handle handle
        );

MCP23S17_Handle MCP23S17_create(
        SPI_Handle spiHandle,
        uint32_t gpioCSIndex,
        MCP23S17_Params *params
        );

void MCP23S17_delete(
        MCP23S17_Handle handle
        );

bool MCP23S17_write(
        MCP23S17_Handle handle,
        uint8_t ucRegAddr,
        uint8_t ucData
        );

bool MCP23S17_read(
        MCP23S17_Handle handle,
        uint8_t ucRegAddr,
        uint8_t* pucData
        );

#endif /*_MCP23S17_H_*/
