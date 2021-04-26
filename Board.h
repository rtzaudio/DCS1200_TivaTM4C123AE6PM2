/*
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
 */

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "DCS1200_TivaTM4C123AE6PMI.h"

#define Board_initGeneral           DCS1200_initGeneral
#define Board_initGPIO              DCS1200_initGPIO
#define Board_initI2C               DCS1200_initI2C
#define Board_initSPI               DCS1200_initSPI
#define Board_initUART              DCS1200_initUART
#define Board_initWatchdog          DCS1200_initWatchdog

#define Board_i2cCAT24C08   		DCS1200_I2C0		    /* I2C0 : CAT24C08 EPROM       */

#define Board_spiSpare              DCS1200_SPI0		    /* SSI-0 : on connector P2     */
#define Board_spiIoCard1            DCS1200_SPI1		    /* SSI-1 : SPI to I/O Card 1   */
#define Board_spiIoCard2            DCS1200_SPI2		    /* SSI-2 : SPI to I/O Card 2   */
#define Board_spiIoCard3            DCS1200_SPI3            /* SSI-3 : SPI to I/O Card 3   */

#define Board_uartIPC               DCS1200_UART1

#define Board_Watchdog              DCS1200_WATCHDOG0
#define Board_SpeedSelect           DCS1200_SPEED           /* speed select relay */
#define Board_resetIoExpanders      DCS1200_RESET_IOX_N     /* I/O expander reset bus */

#define Board_Card1_MonMode_SS      DCS1200_SSI1FSS1        /* I/O Card 1 - channels 1-8   */
#define Board_Card1_RecHold_SS      DCS1200_SSI1FSS2

#define Board_Card2_MonMode_SS      DCS1200_SSI2FSS1        /* I/O Card 2 - channels 9-16  */
#define Board_Card2_RecHold_SS      DCS1200_SSI2FSS2

#define Board_Card3_MonMode_SS      DCS1200_SSI3FSS1        /* I/O Card 3 - channels 17-24 */
#define Board_Card3_RecHold_SS      DCS1200_SSI3FSS2

#define Board_ledStatus             DCS1200_LED_STAT

#define Board_readDIPSwitch         DCS1200_readDIPSwitch

#define Board_LED_ON                DCS1200_LED_ON
#define Board_LED_OFF               DCS1200_LED_OFF

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
