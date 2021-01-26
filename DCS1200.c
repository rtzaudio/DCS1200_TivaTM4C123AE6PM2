/* ============================================================================
 *
 * DTC-1200 Digital Transport Controller for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2016, RTZ Professional Audio, LLC
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

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/gates/GateMutex.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Tivaware Driver files */
#include <driverlib/eeprom.h>
#include <driverlib/fpu.h>

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
#include "DCS1200.h"
#include "Utils.h"

/* Global Data Items */
SYSCONFIG g_cfg;
SYSDATA   g_sys;

/* Static Function Prototypes */
Int main();
Void MainTask(UArg a0, UArg a1);
bool Init_Devices(void);
bool Init_Peripherals(void);
bool WriteRegisterMask(MCP23S17_Handle handle, uint16_t mask);
uint16_t GetMonitorMaskFromTrackState(uint8_t* tracks);
uint16_t GetRecordHoldMaskFromTrackState(uint8_t* tracks);
void WriteAllMonitorModes(void);
void WriteAllRecordHoldModes(void);

//*****************************************************************************
// Main Program Entry Point
//*****************************************************************************

Int main()
{
    Error_Block eb;
    Task_Params taskParams;

    /* Enables Floating Point Hardware Unit */
    FPUEnable();

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initUART();
    Board_initI2C();
    Board_initSPI();

    /* Start the main application button polling task */

    Error_init(&eb);

    Task_Params_init(&taskParams);

    taskParams.stackSize = 1248;
    taskParams.priority  = 13;

    if (Task_create(MainTask, &taskParams, &eb) == NULL)
        System_abort("MainTask!\n");

    BIOS_start();    /* does not return */

    return(0);
}

//*****************************************************************************
// Initialize and open various system peripherals we'll be using
//*****************************************************************************

bool Init_Peripherals(void)
{
    SPI_Params  spiParams;
    I2C_Params  i2cParams;

    /*
     * Open the I2C ports for peripherals we need to communicate with.
     */

    /* Open I2C0 for U14 CAT24C08 EPROM */

    I2C_Params_init(&i2cParams);

    i2cParams.transferCallbackFxn = NULL;
    i2cParams.transferMode        = I2C_MODE_BLOCKING;
    i2cParams.bitRate             = I2C_100kHz;

    if ((g_sys.i2c0 = I2C_open(Board_i2cCAT24C08, &i2cParams)) == NULL)
    {
        System_printf("Error: Unable to openI2C3 port\n");
        System_flush();
        return false;
    }

    /*
     * Open the SPI ports for peripherals we need to communicate with.
     */

    /* SPI-0 bus */

    SPI_Params_init(&spiParams);

    spiParams.mode            = SPI_MASTER;
    spiParams.transferMode    = SPI_MODE_BLOCKING;
    spiParams.transferTimeout = 1000;
    spiParams.frameFormat     = SPI_POL0_PHA0;
    spiParams.bitRate         = 800000;            /* 800kHz */
    spiParams.dataSize        = 8;

    if ((g_sys.spiSpare = SPI_open(Board_spiSpare, &spiParams)) == NULL)
    {
        System_printf("Error: Unable to open SPI0 port\n");
        System_flush();
        return false;
    }

    /* SPI-1 bus */

    SPI_Params_init(&spiParams);

    spiParams.mode            = SPI_MASTER;
    spiParams.transferMode    = SPI_MODE_BLOCKING;
    spiParams.transferTimeout = 1000;
    spiParams.frameFormat     = SPI_POL0_PHA0;
    spiParams.bitRate         = 800000;            /* 800kHz */
    spiParams.dataSize        = 8;

    if ((g_sys.spiIoEx[0] = SPI_open(Board_spiIoCard1, &spiParams)) == NULL)
    {
        System_printf("Error: Unable to open SPI1 port\n");
        System_flush();
        return false;
    }

    /* SPI-2 bus */

    SPI_Params_init(&spiParams);

    spiParams.mode            = SPI_MASTER;
    spiParams.transferMode    = SPI_MODE_BLOCKING;
    spiParams.transferTimeout = 1000;
    spiParams.frameFormat     = SPI_POL0_PHA0;
    spiParams.bitRate         = 800000;            /* 800kHz */
    spiParams.dataSize        = 8;

    if ((g_sys.spiIoEx[1] = SPI_open(Board_spiIoCard2, &spiParams)) == NULL)
    {
        System_printf("Error: Unable to open SPI2 port\n");
        System_flush();
        return false;
    }

    /* SPI-3 bus */

    SPI_Params_init(&spiParams);

    spiParams.mode            = SPI_MASTER;
    spiParams.transferMode    = SPI_MODE_BLOCKING;
    spiParams.transferTimeout = 1000;
    spiParams.frameFormat     = SPI_POL0_PHA0;
    spiParams.bitRate         = 800000;            /* 800kHz */
    spiParams.dataSize        = 8;

    if ((g_sys.spiIoEx[2] = SPI_open(Board_spiIoCard3, &spiParams)) == NULL)
    {
        System_printf("Error: Unable to open SPI3 port\n");
        System_flush();
        return false;
    }

    return true;
}

//*****************************************************************************
// Initialize and open various system peripherals we'll be using
//*****************************************************************************

bool Init_Devices(void)
{
    MCP23S17_Params ioxParams;

    /* Reset ALL of the MCP23S17 I/O Expanders */
    GPIO_write(Board_ledStatus, Board_LED_ON);
    GPIO_write(Board_resetIoExpander, PIN_LOW);
    Task_sleep(100);
    GPIO_write(Board_ledStatus, Board_LED_OFF);
    GPIO_write(Board_resetIoExpander, PIN_HIGH);
    Task_sleep(100);
    GPIO_write(Board_ledStatus, Board_LED_ON);

    /* Create and attach I/O expanders to SPI ports */

    /* Setup the SPI tracks to initialize each I/O expander card on the DCS1200 motherboard.
     * Each I/O expander card controls 8-tracks and has two I/O expanders on each card.
     * The first expander selects the monitor mode (input, sync, repro) with two bits
     * to control the tri-state line drivers. The second expander selects the channel
     * record hold/enable) state for each of the 8 tracks on the I/O card.
     */

    /* Setup I/O Card-1 SPI tracks to drive monitor mode and record hold I/O expanders */

    MCP23S17_Params_init(&ioxParams);

    g_sys.handle_MonMode[0] = MCP23S17_create(g_sys.spiIoEx[0], Board_Card1_MonMode_SS, &ioxParams);
    g_sys.handle_RecHold[0] = MCP23S17_create(g_sys.spiIoEx[0], Board_Card1_RecHold_SS, &ioxParams);

    /* Setup I/O Card-2 SPI tracks to drive monitor mode and record hold I/O expanders */

    MCP23S17_Params_init(&ioxParams);

    g_sys.handle_MonMode[1] = MCP23S17_create(g_sys.spiIoEx[1], Board_Card2_MonMode_SS, &ioxParams);
    g_sys.handle_RecHold[1] = MCP23S17_create(g_sys.spiIoEx[1], Board_Card2_RecHold_SS, &ioxParams);

    /* Setup I/O Card-3 SPI tracks to drive monitor mode and record hold I/O expanders */

    MCP23S17_Params_init(&ioxParams);

    g_sys.handle_MonMode[2] = MCP23S17_create(g_sys.spiIoEx[2], Board_Card3_MonMode_SS, &ioxParams);
    g_sys.handle_RecHold[2] = MCP23S17_create(g_sys.spiIoEx[2], Board_Card3_RecHold_SS, &ioxParams);

    System_flush();

    return true;
}

//*****************************************************************************
// This functions scans eight channel state words from a track state array
// and creates a port-A and port-B mask value combined as a 16-bit word.
// The upper 8-bits contains the port-B register value for the monitor
// mode and the lower 8-bits contains the port-A register value. This word
// specifies the channel state (repro, sync or input) for 8-tracks of
// an I/O card on the DCS motherboard.
//*****************************************************************************

uint16_t GetMonitorMaskFromTrackState(uint8_t* tracks)
{
    uint16_t maskA = 0;
    uint16_t maskB = 0;
    uint16_t mask;

    /* Monitor mask for port-A on the I/O expander */
    maskA |= ((tracks[0] & DCS_TRACK_MASK) << 0);
    maskA |= ((tracks[1] & DCS_TRACK_MASK) << 2);
    maskA |= ((tracks[2] & DCS_TRACK_MASK) << 4);
    maskA |= ((tracks[3] & DCS_TRACK_MASK) << 6);

    /* Monitor mask for port-B on the I/O expander */
    maskB |= ((tracks[4] & DCS_TRACK_MASK) << 0);
    maskB |= ((tracks[5] & DCS_TRACK_MASK) << 2);
    maskB |= ((tracks[6] & DCS_TRACK_MASK) << 4);
    maskB |= ((tracks[7] & DCS_TRACK_MASK) << 6);

    /* Combine A-reg and B-reg into 16-bit value */
    mask = (maskB << 8) | (maskA & 0xFF);

    return mask;
}

//*****************************************************************************
// This functions reads 8 channel state bytes from a track state array
// and creates a port-A and port-B mask value combined as a 16-bit word.
// The upper 8-bits contains the port-B register value for the monitor
// mode and the lower 8-bits contains the port-A register value. This word
// specifies the channel state (repro, sync or input) for 8-tracks of
// an I/O card on the DCS motherboard.
//*****************************************************************************

uint16_t GetRecordHoldMaskFromTrackState(uint8_t* tracks)
{
    size_t i;
    uint16_t mask = 0;
    uint16_t bit  = 0x01;

    for (i=0; i < 8; i++)
    {
        if (tracks[i] & DCS_T_READY)
            mask |= bit;

        bit <<= 1;
    }

    return mask;
}

//*****************************************************************************
// This function writes a 16-bit register mask to the A/B port on an
// I/O expander to configure the track states of 8-tracks.
//*****************************************************************************

bool WriteRegisterMask(MCP23S17_Handle handle, uint16_t mask)
{
    /* Update Port-A & Port-B outputs */
    MCP23S17_write(handle, MCP_GPIOA, (uint8_t)(mask & 0xFF));
    MCP23S17_write(handle, MCP_GPIOB, (uint8_t)((mask >> 8) & 0xFF));
    return true;
}

//*****************************************************************************
// Write all monitors modes from the track state array to all 24 tracks
// across the I/O expanders.
//*****************************************************************************

void WriteAllMonitorModes(void)
{
    uint16_t mask;

    /*** Set monitor modes for tracks 1-8 ***/

    /* Read 8-bytes from the track state array and create 16-bit register mask */
    mask = GetMonitorMaskFromTrackState(&g_sys.trackState[0]);

    /* Set 16-bits on the I/O expander to configure the monitor modes */
    WriteRegisterMask(g_sys.handle_MonMode[0], mask);

    /*** Set monitor modes for tracks 9-16 ***/

    /* Read 8-bytes from the track state array and create 16-bit register mask */
    mask = GetMonitorMaskFromTrackState(&g_sys.trackState[8]);

    /* Set 16-bits on the I/O expander to configure the monitor modes */
    WriteRegisterMask(g_sys.handle_MonMode[1], mask);

    /*** Set monitor modes for tracks 17-24 ***/

    /* Read 8-bytes from the track state array and create 16-bit register mask */
    mask = GetMonitorMaskFromTrackState(&g_sys.trackState[16]);

    /* Set 16-bits on the I/O expander to configure the monitor modes */
    WriteRegisterMask(g_sys.handle_MonMode[2], mask);
}

//*****************************************************************************
// Write all reocrd hold modes from the track state array to all 24 tracks
// across the I/O expanders.
//*****************************************************************************

void WriteAllRecordHoldModes(void)
{
    uint16_t mask;

    /*** Set record hold modes for tracks 1-8 ***/

    /* Read 8-bytes from the track state array and create 16-bit register mask */
    mask = GetRecordHoldMaskFromTrackState(&g_sys.trackState[0]);

    /* Set 16-bits on the I/O expander to configure the monitor modes */
    WriteRegisterMask(g_sys.handle_RecHold[0], mask);

    /*** Set record hold modes for tracks 9-16 ***/

    /* Read 8-bytes from the track state array and create 16-bit register mask */
    mask = GetRecordHoldMaskFromTrackState(&g_sys.trackState[8]);

    /* Set 16-bits on the I/O expander to configure the monitor modes */
    WriteRegisterMask(g_sys.handle_RecHold[1], mask);

    /*** Set record hold modes for tracks 17-24 ***/

    /* Read 8-bytes from the track state array and create 16-bit register mask */
    mask = GetRecordHoldMaskFromTrackState(&g_sys.trackState[16]);

    /* Set 16-bits on the I/O expander to configure the monitor modes */
    WriteRegisterMask(g_sys.handle_RecHold[2], mask);
}

//*****************************************************************************
// The main application initialization, setup and button controler task.
//*****************************************************************************

Void MainTask(UArg a0, UArg a1)
{
    //Error_Block eb;
    //Task_Params taskParams;

    /* Initialize the default program data values */
    memset(&g_cfg, 0, sizeof(SYSCONFIG));
    memset(&g_sys, 0, sizeof(SYSDATA));

    /* Initialize all the peripherals */
    Init_Peripherals();

    /* Initialize the default system parameters */
    SysConfig_init(&g_cfg);

    /* Read the system configuration parameters from storage */
    SysConfig_read(&g_cfg);

    /* Initialize all the hardware devices */
    Init_Devices();

    /****************************************************************
     * Enter the main application button processing loop forever.
     ****************************************************************/

    for(;;)
    {

    }
}

/* End-Of-File */
