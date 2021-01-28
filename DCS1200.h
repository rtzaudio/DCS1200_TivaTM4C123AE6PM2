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

/*** Global Constants ******************************************************/

/* VERSION INFO - The min build specifies the minimum build required
 * that does NOT force a default reset of all the config parameters
 * at run time. For instance, if the config loads build 5 and the minimum
 * is set to 3, then it will reset config for anything less than build 3.
 * Likewise, versions 3 or higher would load and use the config values from
 * eprom as normal. This provides a means to force run time config defaults
 * to be reset or not.
 */
#define FIRMWARE_VER        1           /* firmware version */
#define FIRMWARE_REV        0       	/* firmware revision */
#define FIRMWARE_BUILD      1           /* firmware build number */
#define FIRMWARE_MIN_BUILD  1           /* min build req'd to force reset */

#if (FIRMWARE_MIN_BUILD > FIRMWARE_BUILD)
#error "DCS build option FIRMWARE_MIN_BUILD set incorrectly"
#endif

#define MAGIC               0xCEB0FACE  /* magic number for EEPROM data */
#define MAKEREV(v, r)       ((v << 16) | (r & 0xFFFF))

#define UNDEFINED           ((uint32_t)(-1))

/* Timeout for SPI communications */
#define TIMEOUT_SPI			500

/*** Track State Constants *************************************************/

#define DCS_NUM_TRACKS      24

#define DCS_TRACK_REPRO     0x00        /* track is in repro mode     */
#define DCS_TRACK_SYNC      0x01        /* track is in sync mode      */
#define DCS_TRACK_INPUT     0x02        /* track is in input mode     */

#define DCS_TRACK_MASK      0x07        /* low 3-bits are track mode  */

#define TRACK_MODE(mask)    (mask & DCS_TRACK_MASK)

/* Upper bits indicate ready/record state */
#define DCS_T_RECORD        0x40        /* track is recording now     */
#define DCS_T_READY         0x80        /* track is armed for record  */

/*** System Structures *****************************************************/

/* This structure contains runtime and program configuration data that is
 * stored and read from EEPROM. The structure size must be 4 byte aligned.
 */

typedef struct _SYSCONFIG
{
	uint32_t magic;
	uint32_t version;
	uint32_t build;
    /*** GLOBAL PARAMETERS ***/
    int32_t debug;                     	/* debug level */
} SYSCONFIG;

/* Global System Data */

typedef struct _SYSDATA
{
    /* Global Handles */
    I2C_Handle i2c0;
    SPI_Handle spiSpare;
    SPI_Handle spiIoEx[3];
    MCP23S17_Handle handle_MonMode[3];
    MCP23S17_Handle handle_RecHold[3];
    /* Global Data */
    uint8_t trackState[DCS_NUM_TRACKS];
} SYSDATA;

/*** Macros & Function Prototypes ******************************************/

/* End-Of-File */
