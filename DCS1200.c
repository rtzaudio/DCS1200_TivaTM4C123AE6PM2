/* ============================================================================
 *
 * DCS-1200 Digital Channel Switcher for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2023, RTZ Professional Audio, LLC
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

/*** Global Data Items ***/

SYSCFG g_cfg;
SYSDAT g_sys;

/* Message Mailbox Handles */

Mailbox_Handle mailboxCommand = NULL;

/* MCP23S17 init parameters for MONITOR control */

static MCP23S17_InitData initMon[] = {
    { MCP_IOCONA, C_SEQOP },            /* Configure for byte mode  */
    { MCP_IOCONB, C_SEQOP },            /* Configure for byte mode  */
    { MCP_IODIRA, 0x00    },            /* Port A - all outputs     */
    { MCP_IODIRB, 0x00    },            /* Port B - all outputs     */
    { MCP_GPIOA, 0x00     },            /* Port A - all outputs LOW */
    { MCP_GPIOB, 0x00     },            /* Port B - all outputs LOW */
};

/* Default MCP23S17 parameters structure */

static MCP23S17_Params monParams = {
    initMon,
    sizeof(initMon)/sizeof(MCP23S17_InitData)
};

/* MCP23S17 init parameters for RECORD control */

static MCP23S17_InitData initRec[] = {
    { MCP_IOCONA, C_SEQOP },            /* Configure for byte mode  */
    { MCP_IOCONB, C_SEQOP },            /* Configure for byte mode  */
    { MCP_IODIRA, 0x00    },            /* Port A - all outputs     */
    { MCP_IODIRB, 0x00    },            /* Port B - all outputs     */
    { MCP_GPIOA,  0xFF    },            /* Port A - all outputs HI  */
    { MCP_GPIOB,  0xFF    },            /* Port B - all outputs HI  */
};

static MCP23S17_Params recParams = {
    initRec,
    sizeof(initRec)/sizeof(MCP23S17_InitData)
};

/*** Static Function Prototypes ***/

Int main();

Void MainTask(UArg a0, UArg a1);
Void RecordTaskFxn(UArg arg0, UArg arg1);

bool Init_Hardware(void);
bool Init_Peripherals(void);

/* hardware interrupt handlers for gpio pins */
void HwiRecordPulse(unsigned int index);
void HwiRecordHold(unsigned int index);

uint8_t xlateStandby(uint8_t trackState);
uint16_t GetMonCtrlMaskFromTrackState(uint8_t* tracks);
uint16_t GetRecCtrlMaskFromTrackState(uint8_t* tracks);

void WriteMonitorModes(void);
void WriteRecordModes(void);
void WriteRecordDisable(void);

void QueueWriteMonitorModes(void);
void QueueWriteRecordModes(void);

int HandleSetTracks(IPCCMD_Handle handle, DCS_IPCMSG_SET_TRACKS* msg);
int HandleGetTracks(IPCCMD_Handle handle, DCS_IPCMSG_GET_TRACKS* msg);
int HandleSetTrack(IPCCMD_Handle handle, DCS_IPCMSG_SET_TRACK* msg);
int HandleGetTrack(IPCCMD_Handle handle, DCS_IPCMSG_GET_TRACK* msg);
int HandleSetSpeed(IPCCMD_Handle handle, DCS_IPCMSG_SET_SPEED* msg);
int HandleGetNumTracks(IPCCMD_Handle handle, DCS_IPCMSG_GET_NUMTRACKS* msg);

//*****************************************************************************
// Main Program Entry Point
//*****************************************************************************

Int main()
{
    Error_Block eb;
    Task_Params taskParams;
    Mailbox_Params mboxParams;

    /* Enables Floating Point Hardware Unit */
    FPUEnable();

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initUART();
    Board_initI2C();
    Board_initSPI();

    /* Create command task mailbox */

    Error_init(&eb);

    Mailbox_Params_init(&mboxParams);

    mailboxCommand = Mailbox_create(sizeof(RecordEventMessage), 10, &mboxParams, &eb);

    if (mailboxCommand == NULL)
        System_abort("Mailbox create failed\nAborting...");

    /* Start the main application button polling task */

    Error_init(&eb);

    Task_Params_init(&taskParams);

    taskParams.stackSize = 1248;
    taskParams.priority  = 10;

    if (Task_create(MainTask, &taskParams, &eb) == NULL)
        System_abort("MainTask!\n");

    BIOS_start();    /* does not return */

    return(0);
}

//*****************************************************************************
// Hardware GPIO's and such
//*****************************************************************************

bool Init_Hardware(void)
{
    uint32_t bits;

    /* Set status LED on */
    GPIO_write(Board_ledStatus, Board_LED_ON);

    /* Set speed select relay to high speed */
    GPIO_write(Board_SpeedSelect, PIN_LOW);

    /* Reset ALL of the MCP23S17 I/O Expanders */
    GPIO_write(Board_resetIoExpanders, PIN_LOW);
    Task_sleep(10);
    GPIO_write(Board_resetIoExpanders, PIN_HIGH);

    /* Set status LED off */
    GPIO_write(Board_ledStatus, Board_LED_OFF);

    /* Read the four lower bits of the DIP switch and invert */
    bits = Board_readDIPSwitch();

    /* Number of tracks supported is based on DIP switches 1 & 2 */
    g_sys.numTracks = (bits & 0x03) * 8;

    return true;
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

    /* Create and attach I/O expanders to SPI ports */

    /* Attach the SPI buses to each I/O expander card on the DCS1200 motherboard.
     * Each I/O expander card controls 8-tracks and has two I/O expanders on each card.
     * The first expander selects the monitor mode (input, sync, repro) with two bits
     * to control the tri-state line drivers. The second expander selects the channel
     * record hold/enable) state for each of the 8 tracks on the I/O card.
     */

    /* Setup I/O Card-1 SPI tracks to drive monitor mode and record hold I/O expanders */
    g_sys.handle_MonMode[0] = MCP23S17_create(g_sys.spiIoEx[0], Board_Card1_MonMode_SS, &monParams);
    g_sys.handle_RecCtrl[0] = MCP23S17_create(g_sys.spiIoEx[0], Board_Card1_RecCtrl_SS, &recParams);

    /* Setup I/O Card-2 SPI tracks to drive monitor mode and record hold I/O expanders */
    g_sys.handle_MonMode[1] = MCP23S17_create(g_sys.spiIoEx[1], Board_Card2_MonMode_SS, &monParams);
    g_sys.handle_RecCtrl[1] = MCP23S17_create(g_sys.spiIoEx[1], Board_Card2_RecCtrl_SS, &recParams);

    /* Setup I/O Card-3 SPI tracks to drive monitor mode and record hold I/O expanders */
    g_sys.handle_MonMode[2] = MCP23S17_create(g_sys.spiIoEx[2], Board_Card3_MonMode_SS, &monParams);
    g_sys.handle_RecCtrl[2] = MCP23S17_create(g_sys.spiIoEx[2], Board_Card3_RecCtrl_SS, &recParams);

    return true;
}

//*****************************************************************************
// This is a helper function takes a track state and checks to see if the
// flag bits DCS_T_MONITOR and DCS_T_STANDBY are set. If so, this means we
// want to override the current channel mode and switch the track to input
// mode instead to handle the standby monitor function.
//*****************************************************************************

uint8_t xlateStandby(uint8_t trackState)
{
    /* If monitor mode enabled and the standby monitor flag is active,
     * then we want to switch to input mode regardless.
     */
    if ((trackState & DCS_T_MONITOR) && (trackState & DCS_T_STANDBY))
    {
        /* Mask out the current track mode (sync/repro/input) */
        trackState &= ~(DCS_MODE_MASK);

        /* Force input mode since standby monitor is active */
        trackState |= DCS_TRACK_INPUT;
    };

    return trackState;
}

//*****************************************************************************
// This functions read 8-bytes and creates a monitor mode bit mask for the
// port-A and port-B values combined as a 16-bit word. The upper 8-bits
// contains the port-B register value for the monitor mode and the lower
// 8-bits contains the port-A register value. This word specifies the channel
// state (repro, sync or input) for 8-tracks of an I/O card on the DCS.
//*****************************************************************************

uint16_t GetMonCtrlMaskFromTrackState(uint8_t* tracks)
{
    uint16_t maskA = 0;
    uint16_t maskB = 0;
    uint16_t mask;

    /* Monitor mask for port-A on the I/O expander */
    maskA |= ((xlateStandby(tracks[0]) & DCS_MODE_MASK) << 0);
    maskA |= ((xlateStandby(tracks[1]) & DCS_MODE_MASK) << 2);
    maskA |= ((xlateStandby(tracks[2]) & DCS_MODE_MASK) << 4);
    maskA |= ((xlateStandby(tracks[3]) & DCS_MODE_MASK) << 6);

    /* Monitor mask for port-B on the I/O expander */
    maskB |= ((xlateStandby(tracks[4]) & DCS_MODE_MASK) << 0);
    maskB |= ((xlateStandby(tracks[5]) & DCS_MODE_MASK) << 2);
    maskB |= ((xlateStandby(tracks[6]) & DCS_MODE_MASK) << 4);
    maskB |= ((xlateStandby(tracks[7]) & DCS_MODE_MASK) << 6);

    /* Combine A-reg and B-reg into 16-bit value */
    mask = (maskB << 8) | (maskA & 0xFF);

    return mask;
}

//*****************************************************************************
// This functions reads 8 channel state bytes from a track state array
// and creates a port-A and port-B mask value combined as a 16-bit word that
// contains the record hold and record strobe enable bits for eight channels.
// The upper 8-bits contains the port-B register value for the record pulse
// signal and the lower 8-bits contains the port-A register value for the
// record hold signal. These two bits control the record functions for
// driving eight channels of record control.
//*****************************************************************************

uint16_t GetRecCtrlMaskFromTrackState(uint8_t* tracks)
{
    size_t i;
    uint16_t mask;
    uint16_t maskA = 0;
    uint16_t maskB = 0;
    uint16_t bitA  = 0x01;
    uint16_t bitB  = 0x01;

    for (i=0; i < 8; i++)
    {
        /* Lower byte is record HOLD bit flags */
        if (tracks[i] & DCS_T_READY)
            maskA |= bitA;

        bitA <<= 1;

        /* Upper byte is record STROBE bit flags */
        if (tracks[i] & DCS_T_RECORD)
        {
            /* If track isn't ready, then don't strobe record!
             * This indicates improper track status bits were
             * set, but we protect against this condition here
             * so we don't strobe the record relay with the
             * record hold flag not set also.
             */
            if (tracks[i] & DCS_T_READY)
                maskB |= bitB;
        }

        bitB <<= 1;
    }

    /* Combine A-reg and B-reg into 16-bit value */
    mask = (maskB << 8) | (maskA & 0xFF);

    /* Invert all bits for open collector output driver logic */
    mask = ~mask;

    return mask;
}

//*****************************************************************************
//
//*****************************************************************************

void WriteRecordModes(void)
{
    uint16_t mask1;
    uint16_t mask2;
    uint16_t mask3;

    /* Get record flags for tracks 1-8 */
    mask1 = GetRecCtrlMaskFromTrackState(&g_sys.trackState[0]);
    /* Get record flags for tracks 9-16 */
    mask2 = GetRecCtrlMaskFromTrackState(&g_sys.trackState[8]);
    /* Get record flags for tracks 17-24 */
    mask3 = GetRecCtrlMaskFromTrackState(&g_sys.trackState[16]);

    /*** Assert any RECORD HOLD lines (lower 8-bits active low) ***/

    /* Channels 01-08 */
    MCP23S17_write(g_sys.handle_RecCtrl[0], MCP_GPIOA, (uint8_t)(mask1 & 0xFF));
    /* Channels 09-16 */
    MCP23S17_write(g_sys.handle_RecCtrl[1], MCP_GPIOA, (uint8_t)(mask2 & 0xFF));
    /* Channels 17-24 */
    MCP23S17_write(g_sys.handle_RecCtrl[2], MCP_GPIOA, (uint8_t)(mask3 & 0xFF));

    /* Setup time after record hold lines are set */
    Task_sleep(10);

    /*** Assert any RECORD STROBE lines to begin strobe pulse ***/

    /* Channels 01-08 */
    MCP23S17_write(g_sys.handle_RecCtrl[0], MCP_GPIOB, (uint8_t)(mask1 >> 8));
    /* Channels 09-16 */
    MCP23S17_write(g_sys.handle_RecCtrl[1], MCP_GPIOB, (uint8_t)(mask2 >> 8));
    /* Channels 17-24 */
    MCP23S17_write(g_sys.handle_RecCtrl[2], MCP_GPIOB, (uint8_t)(mask3 >> 8));

    /* Record pulse low duration! */
    Task_sleep(50);

    /*** Set any RECORD STROBE lines back high non-asserted ***/

    /* Channels 01-08 */
    MCP23S17_write(g_sys.handle_RecCtrl[0], MCP_GPIOB, 0xFF);
    /* Channels 09-16 */
    MCP23S17_write(g_sys.handle_RecCtrl[1], MCP_GPIOB, 0xFF);
    /* Channels 17-24 */
    MCP23S17_write(g_sys.handle_RecCtrl[2], MCP_GPIOB, 0xFF);
}

//*****************************************************************************
//
//*****************************************************************************

void WriteRecordDisable(void)
{
    //size_t i;
    uint16_t mask1;
    uint16_t mask2;
    uint16_t mask3;

    /* Clear any record active flag */

    //for (i=0; i < DCS_NUM_TRACKS; i++)
    //    g_sys.trackState[i] &= ~(DCS_T_RECORD);

    /* Get record hold flags for tracks 1-8 */
    mask1 = GetRecCtrlMaskFromTrackState(&g_sys.trackState[0]);
    /* Get record hold flags for tracks 9-16 */
    mask2 = GetRecCtrlMaskFromTrackState(&g_sys.trackState[8]);
    /* Get record hold flags for tracks 17-24 */
    mask3 = GetRecCtrlMaskFromTrackState(&g_sys.trackState[16]);

    /*** Release any RECORD HOLD lines (lower 8-bits active low) ***/

    /* Channels 01-08 */
    MCP23S17_write(g_sys.handle_RecCtrl[0], MCP_GPIOA, 0xFF);
    /* Channels 09-16 */
    MCP23S17_write(g_sys.handle_RecCtrl[1], MCP_GPIOA, 0xFF);
    /* Channels 17-24 */
    MCP23S17_write(g_sys.handle_RecCtrl[2], MCP_GPIOA, 0xFF);

    /*** Set any RECORD STROBE lines back high non-asserted ***/

    /* Channels 01-08 */
    MCP23S17_write(g_sys.handle_RecCtrl[0], MCP_GPIOB, 0xFF);
    /* Channels 09-16 */
    MCP23S17_write(g_sys.handle_RecCtrl[1], MCP_GPIOB, 0xFF);
    /* Channels 17-24 */
    MCP23S17_write(g_sys.handle_RecCtrl[2], MCP_GPIOB, 0xFF);
}

//*****************************************************************************
// Write all monitors modes from the track state array to all 24 tracks
// across the I/O expanders.
//*****************************************************************************

void WriteMonitorModes(void)
{
    uint16_t mask;

    /*** Set monitor modes for tracks 1-8 ***/

    /* Read 8-bytes from the track state array and create 16-bit register mask */
    mask = GetMonCtrlMaskFromTrackState(&g_sys.trackState[0]);
    /* Set 16-bits on the I/O expander to configure the monitor modes */
    MCP23S17_write(g_sys.handle_MonMode[0], MCP_GPIOA, (uint8_t)(mask & 0xFF));
    MCP23S17_write(g_sys.handle_MonMode[0], MCP_GPIOB, (uint8_t)((mask >> 8) & 0xFF));

    /*** Set monitor modes for tracks 9-16 ***/

    /* Read 8-bytes from the track state array and create 16-bit register mask */
    mask = GetMonCtrlMaskFromTrackState(&g_sys.trackState[8]);
    /* Set 16-bits on the I/O expander to configure the monitor modes */
    MCP23S17_write(g_sys.handle_MonMode[1], MCP_GPIOA, (uint8_t)(mask & 0xFF));
    MCP23S17_write(g_sys.handle_MonMode[1], MCP_GPIOB, (uint8_t)((mask >> 8) & 0xFF));

    /*** Set monitor modes for tracks 17-24 ***/

    /* Read 8-bytes from the track state array and create 16-bit register mask */
    mask = GetMonCtrlMaskFromTrackState(&g_sys.trackState[16]);
    /* Set 16-bits on the I/O expander to configure the monitor modes */
    MCP23S17_write(g_sys.handle_MonMode[2], MCP_GPIOA, (uint8_t)(mask & 0xFF));
    MCP23S17_write(g_sys.handle_MonMode[2], MCP_GPIOB, (uint8_t)((mask >> 8) & 0xFF));
}

//*****************************************************************************
// Post a message to the record handler task to update all monitor modes.
//*****************************************************************************

void QueueWriteMonitorModes(void)
{
    RecordEventMessage msg;

    msg.eventType = WRITE_MONITOR_MODES;
    msg.ui32Index = 0;
    msg.ui32Mask  = 0;

    Mailbox_post(mailboxCommand, &msg, BIOS_NO_WAIT);
 }

void QueueWriteRecordModes(void)
{
    RecordEventMessage msg;

    msg.eventType = WRITE_RECORD_MODES;
    msg.ui32Index = 0;
    msg.ui32Mask  = 0;

    Mailbox_post(mailboxCommand, &msg, BIOS_NO_WAIT);
 }

//*****************************************************************************
// HWI Callback function for record pulse and record hold lines from DTC.
// These queue messages to the record handler task which enables and disables
// record modes for any or all of the tracks.
//*****************************************************************************

void HwiRecordPulse(unsigned int index)
{
    uint32_t mask;
    RecordEventMessage msg;

    //Assert_isTrue(index == Board_Record_Pulse);

    /* GPIO pin interrupt occurred, read the state */
    mask = GPIO_read(index);

    /* Clear the interrupt first */
    GPIO_clearInt(index);

    /* Enable the interrupt again so we'll get more */
    GPIO_enableInt(index);

    msg.eventType = RECORD_PULSE_CHANGE;
    msg.ui32Index = index;
    msg.ui32Mask  = mask;

    Mailbox_post(mailboxCommand, &msg, BIOS_NO_WAIT);
 }

void HwiRecordHold(unsigned int index)
{
    uint32_t mask;
    RecordEventMessage msg;

    //Assert_isTrue(index == Board_Record_Hold);

    /* GPIO pin interrupt occurred, read the state */
    mask = GPIO_read(index);

    /* Clear the interrupt first */
    GPIO_clearInt(index);

    /* Enable the interrupt again so we'll get more */
    GPIO_enableInt(index);

    msg.eventType = RECORD_HOLD_CHANGE;
    msg.ui32Index = index;
    msg.ui32Mask  = mask;

    Mailbox_post(mailboxCommand, &msg, BIOS_NO_WAIT);
}

//*****************************************************************************
// This task handles any record enable/disable signal events for all channels.
// GPIO interrupt events are triggered any time the record hold or record
// pulse line changes state. The DTC sets the REC_HOLD_N and REC_PULSE_N to
// signal the user is requesting record mode to be enabled or disabled via
// the transport record/play control buttons. A switcher card channel will
// remain in record until the hold line is dropped for that channel. The
// record pulse line triggers the switcher card to enter record mode.
//*****************************************************************************

Void RecordTaskFxn(UArg arg0, UArg arg1)
{
    RecordEventMessage msg;

    /*
     * Now begin the main program loop processing button press events
     */

    while (true)
    {
        /* Wait for a message */
        if (!Mailbox_pend(mailboxCommand, &msg, BIOS_WAIT_FOREVER))
            continue;

        switch(msg.eventType)
        {
        case RECORD_HOLD_CHANGE:
            // The REC_HOLD_N gpio line changed state to high. Ensure
            // we disable all record functions no matter what!
            if (msg.ui32Mask)
                WriteRecordDisable();
            break;

        case RECORD_PULSE_CHANGE:
            // The REC_PULSE_N gpio line changed state
            break;

        case WRITE_MONITOR_MODES:
            WriteMonitorModes();
            break;

        case WRITE_RECORD_MODES:
            WriteRecordModes();
            break;

        default:
            break;
        }
    }
}

//*****************************************************************************
// This is the main task to setup, initialize all communications via UART
// from the STC host controller. This task simply reads a command and performs
// any channel data configuration or other commands as requested.
//*****************************************************************************

Void MainTask(UArg a0, UArg a1)
{
    int rc;
    size_t i;
    uint8_t* msgBuf;
    Error_Block eb;
    Task_Params taskParams;
    UART_Params uartParams;
    UART_Handle uartHandle;
    IPCCMD_Params ipcParams;
    IPCCMD_Handle ipcHandle;

    /* Initialize the default program data values */
    memset(&g_cfg, 0, sizeof(SYSCFG));
    memset(&g_sys, 0, sizeof(SYSDAT));

    for (i=0; i < DCS_NUM_TRACKS; i++)
    {
        g_sys.trackState[i] = DCS_TRACK_MODE(DCS_TRACK_REPRO);
    }

    /* Initialize GPIO hardware pins */
    Init_Hardware();

    /* Create all the peripheral objects */
    Init_Peripherals();

    /* Initialize the default system parameters */
    SysConfig_Init(&g_cfg);

    /* Read the system configuration parameters from storage */
    SysConfig_Read(&g_cfg);

    /* Open the UART for binary mode */

    UART_Params_init(&uartParams);

    uartParams.readMode       = UART_MODE_BLOCKING;
    uartParams.writeMode      = UART_MODE_BLOCKING;
    uartParams.readTimeout    = 2000;                   // 1 second read timeout
    uartParams.writeTimeout   = BIOS_WAIT_FOREVER;
    uartParams.readCallback   = NULL;
    uartParams.writeCallback  = NULL;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.writeDataMode  = UART_DATA_BINARY;
    uartParams.readDataMode   = UART_DATA_BINARY;
    uartParams.readEcho       = UART_ECHO_OFF;
    uartParams.baudRate       = 115200;
    uartParams.stopBits       = UART_STOP_ONE;
    uartParams.parityType     = UART_PAR_NONE;

    uartHandle = UART_open(Board_uartIPC, &uartParams);

    if (uartHandle == NULL)
        System_abort("Error initializing UART\n");

    /* Create IPC command object with UART attached */

    IPCCMD_Params_init(&ipcParams);
    ipcParams.uartHandle = uartHandle;
    ipcHandle = IPCCMD_create(&ipcParams);
    if (ipcHandle == NULL)
        System_abort("IPCCMD_create() failed");

    /* Allocate memory for message receive buffer */

    Error_init(&eb);
    msgBuf = (uint8_t*)Memory_alloc(NULL, RXBUFSIZ, 0, &eb);
    if (msgBuf == NULL)
        System_abort("RxBuf allocation failed");

    /* Create the task that monitors the record pulse and record
     * hold signals from the DTC controller. The DTC drives these
     * two signals to handle enabling or disabling any tracks for
     * record during operation.
     */

    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.stackSize = 2048;
    taskParams.priority  = 10;
    Task_create((Task_FuncPtr)RecordTaskFxn, &taskParams, &eb);

    /* Setup the record HOLD and PULSE interrupt handlers
     * and enable interrupts for each of these GPIO lines.
     */

    /* Setup the callback GPIO hardware interrupt handlers */
    GPIO_setCallback(Board_Record_Pulse, HwiRecordPulse);
    GPIO_setCallback(Board_Record_Hold, HwiRecordHold);

    /* Enable GPIO record hold and record pulse pin interrupts */
    //GPIO_enableInt(Board_Record_Pulse);
    //GPIO_enableInt(Board_Record_Hold);

    /****************************************************************
     * Enter the main application button processing loop forever.
     ****************************************************************/

    for(;;)
    {
        /* Get pointer to header plus message receive buffer */
        DCS_IPCMSG_HDR* msg = (DCS_IPCMSG_HDR*)msgBuf;

        /* Specifies the max buffer size our receiver can hold. This gets updated on
         * return and contains the actual number of header and message bytes received.
         */
        msg->length = RXBUFSIZ;

        /* Attempt to receive an IPC message */
        rc = IPCCMD_ReadMessage(ipcHandle, msg);

        /* Toggle the status LED on each packet receive or timeout */
        GPIO_toggle(Board_ledStatus);

        /* No packet received, loop and continue waiting for a packet */
        if (rc == IPC_ERR_TIMEOUT)
            continue;

        /* Check for any error attempting to read a packet */
        if (rc != IPC_ERR_SUCCESS)
        {
            System_printf("IPCCMD_ReadMessage() error %d\n", rc);
            System_flush();
            continue;
        }

        /* Flash LED on each packet received */
        GPIO_write(Board_ledStatus, PIN_HIGH);

        /* reset return error/status fields */
        msg->error  = 0;
        msg->status = 0;

        /* Dispatch the message by opcode received */
        switch(msg->opcode)
        {
        case DCS_OP_SET_TRACKS:     /* set all 24-track states */
            rc = HandleSetTracks(ipcHandle, (DCS_IPCMSG_SET_TRACKS*)msgBuf);
            break;

        case DCS_OP_GET_TRACKS:     /* get all 24-track states */
            rc = HandleGetTracks(ipcHandle, (DCS_IPCMSG_GET_TRACKS*)msgBuf);
            break;

        case DCS_OP_SET_TRACK:      /* set single track state  */
            rc = HandleSetTrack(ipcHandle, (DCS_IPCMSG_SET_TRACK*)msgBuf);
            break;

        case DCS_OP_GET_TRACK:      /* get single track state  */
            rc = HandleGetTrack(ipcHandle, (DCS_IPCMSG_GET_TRACK*)msgBuf);
            break;

        case DCS_OP_SET_SPEED:      /* set tape speed hi/lo    */
            rc = HandleSetSpeed(ipcHandle, (DCS_IPCMSG_SET_SPEED*)msgBuf);
            break;

        case DCS_OP_GET_NUMTRACKS:  /* get num tracks supported */
            rc = HandleGetNumTracks(ipcHandle, (DCS_IPCMSG_GET_NUMTRACKS*)msgBuf);
            break;

        default:
            /* Transmit a NAK error response to client */
            rc = IPCCMD_WriteNAK(ipcHandle);
            break;
        }

        if (rc != IPC_ERR_SUCCESS)
        {
            System_printf("ipc tx error %d\n", rc);
            System_flush();
        }

        /* Flash LED on each packet received */
        GPIO_write(Board_ledStatus, PIN_LOW);
    }
}

//*****************************************************************************
//
//*****************************************************************************

int HandleSetTracks(
        IPCCMD_Handle handle,
        DCS_IPCMSG_SET_TRACKS* msg
        )
{
    int rc;

    /* Copy 24-tracks of the track state data to our global buffer */
    memcpy(g_sys.trackState, &msg->trackState, DCS_NUM_TRACKS);

    /* Set all the monitor modes on the channel I/O cards */
    QueueWriteMonitorModes();

    /* Enable/disable RECORD on ready channels if flag is set */
    if (msg->flags == 1)
        QueueWriteRecordModes();

    /* Transmit an ACK only response to client */
    rc = IPCCMD_WriteACK(handle);

    return rc;
}

//*****************************************************************************
//
//*****************************************************************************

int HandleGetTracks(
        IPCCMD_Handle handle,
        DCS_IPCMSG_GET_TRACKS* msg
        )
{
    int rc;

    /* Copy 24-tracks of the track state data to tx buffer */
    memcpy(msg->trackState, g_sys.trackState, DCS_NUM_TRACKS);

    /* Set length of return data */
    msg->hdr.length = sizeof(DCS_IPCMSG_GET_TRACKS);

    /* Write message plus ACK to client */
    rc = IPCCMD_WriteMessageACK(handle, &msg->hdr);

    return rc;
}

//*****************************************************************************
//
//*****************************************************************************

int HandleSetTrack(
        IPCCMD_Handle handle,
        DCS_IPCMSG_SET_TRACK* msg
        )
{
    int rc;

    size_t index = (size_t)msg->trackNum;

    if (index >= DCS_NUM_TRACKS)
    {
        /* Transmit an NAK only response to client */
        rc = IPCCMD_WriteNAK(handle);
    }
    else
    {
        /* Copy track state data to our global buffer */
        g_sys.trackState[index] = msg->trackState;

        /* Set all the monitor modes on the channel I/O cards */
        QueueWriteMonitorModes();

        /* Enable/disable RECORD on ready channels if flag is set */
        if (msg->flags == 1)
            QueueWriteRecordModes();

        /* Transmit an ACK only response client */
        rc = IPCCMD_WriteACK(handle);
    }

    return rc;
}

//*****************************************************************************
//
//*****************************************************************************

int HandleGetTrack(
        IPCCMD_Handle handle,
        DCS_IPCMSG_GET_TRACK* msg
        )
{
    int rc;

    size_t index = (size_t)msg->trackNum;

    if (index >= DCS_NUM_TRACKS)
    {
        /* Transmit an NAK only response to client */
        rc = IPCCMD_WriteNAK(handle);
    }
    else
    {
        /* Get track state data from our global buffer */
        msg->trackState = g_sys.trackState[index];

        /* Set length of send data */
        msg->hdr.length = sizeof(DCS_IPCMSG_GET_TRACK);

        /* Write message plus ACK to client */
        rc = IPCCMD_WriteMessageACK(handle, &msg->hdr);
    }

    return rc;
}

//*****************************************************************************
//
//*****************************************************************************

int HandleSetSpeed(
        IPCCMD_Handle handle,
        DCS_IPCMSG_SET_SPEED* msg
        )
{
    int rc;

    /* Save tape speed in config data */
    g_sys.tapeSpeed = (msg->tapeSpeed) ? 1 : 0;

    /* Get track state data from our global buffer */

    if (g_sys.tapeSpeed != 0)
    {
        /* Set speed select relay OFF for high speed */
        GPIO_write(Board_SpeedSelect, PIN_LOW);
    }
    else
    {
        /* Set speed select relay ON for low speed */
        GPIO_write(Board_SpeedSelect, PIN_HIGH);
    }

    /* Transmit an ACK only response client */
    rc = IPCCMD_WriteACK(handle);

    return rc;
}

//*****************************************************************************
//
//*****************************************************************************

int HandleGetNumTracks(
        IPCCMD_Handle handle,
        DCS_IPCMSG_GET_NUMTRACKS* msg
        )
{
    int rc;

    /* Get number of tracks configuration */
    msg->numTracks = (uint16_t)g_sys.numTracks;

    /* Set length of send data */
    msg->hdr.length = sizeof(DCS_IPCMSG_GET_NUMTRACKS);

    /* Write message plus ACK to client */
    rc = IPCCMD_WriteMessageACK(handle, &msg->hdr);

    return rc;
}

/* End-Of-File */

