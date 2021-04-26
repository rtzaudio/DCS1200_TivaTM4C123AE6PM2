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
/** ============================================================================
 *  @file       DCS1200.h
 *
 *  @brief      DCS1200 Board Specific APIs
 *
 *  The DCS1200 header file should be included in an application as follows:
 *  @code
 *  #include <DCS1200.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef __DCS1200_TM4C123AE6PMI_H
#define __DCS1200_TM4C123AE6PMI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/GPIO.h>

/*** Hardware Constants *******************************************************/

#define DCS1200_LED_OFF ( 0)
#define DCS1200_LED_ON  (~0)

#define PIN_LOW			( 0)
#define PIN_HIGH		(~0)

/* I/O Expander Card Port Pin Definitions */

/* I/O Expander U1 Port-A Pins */
#define DCS_MON_HI_0    0x01
#define DCS_MON_LO_0    0x02
#define DCS_MON_HI_1    0x04
#define DCS_MON_LO_1    0x08
#define DCS_MON_HI_2    0x10
#define DCS_MON_LO_2    0x20
#define DCS_MON_HI_3    0x40
#define DCS_MON_LO_3    0x80
/* I/O Expander U1 Port-B Pins */
#define DCS_MON_HI_4    0x01
#define DCS_MON_LO_4    0x02
#define DCS_MON_HI_5    0x04
#define DCS_MON_LO_5    0x08
#define DCS_MON_HI_6    0x10
#define DCS_MON_LO_6    0x20
#define DCS_MON_HI_7    0x40
#define DCS_MON_LO_7    0x80

/* I/O Expander U2 Port-A Pins */
#define DCS_REC_HOLD_0  0x01
#define DCS_REC_HOLD_1  0x02
#define DCS_REC_HOLD_2  0x04
#define DCS_REC_HOLD_3  0x08
#define DCS_REC_HOLD_4  0x10
#define DCS_REC_HOLD_5  0x20
#define DCS_REC_HOLD_6  0x40
#define DCS_REC_HOLD_7  0x80

/*******************************************************************************
 * Functions and Constants
 ******************************************************************************/

/*!
 *  @def    DCS1200_GPIOName
 *  @brief  Enum of LED names on the DCS1200 dev board
 */
typedef enum DCS1200_GPIOName {
    DCS1200_BOOT_N,                 /* GPIO_PB2 */
    DCS1200_CFG1,                   /* GPIO_PE0 */
    DCS1200_CFG2,                   /* GPIO_PE1 */
    DCS1200_CFG3,                   /* GPIO_PE2 */
    DCS1200_CFG4,                   /* GPIO_PE3 */
    DCS1200_DOUT1,                  /* GPIO_PG4 */
    DCS1200_DOUT2,                  /* GPIO_PG5 */
    DCS1200_SSI2FSS2,               /* GPIO_PB3 */
    DCS1200_SSI2FSS1,               /* GPIO_PB5 */
    DCS1200_SSI3FSS1,               /* GPIO_PD1 */
    DCS1200_SSI3FSS2,               /* GPIO_PD4 */
    DCS1200_SPEED,                  /* GPIO_PD5 */
    DCS1200_LED_STAT,               /* GPIO_PD6 */
    DCS1200_RESET_IOX_N,            /* GPIO_PD7 */
    DCS1200_SSI1FSS1,               /* GPIO_PF3 */
    DCS1200_SSI1FSS2,               /* GPIO_PF4 */

    DCS1200_GPIOCOUNT
} DCS1200_GPIOName;

/*!
 *  @def    DCS1200_I2CName
 *  @brief  Enum of I2C names on the DCS1200 dev board
 */
typedef enum DCS1200_I2CName {
    DCS1200_I2C0 = 0,

    DCS1200_I2CCOUNT
} DCS1200_I2CName;

/*!
 *  @def    DCS1200_PWMName
 *  @brief  Enum of PWM names on the DCS1200 dev board
 */
typedef enum DCS1200_PWMName {
    DCS1200_PWM0 = 0,

    DCS1200_PWMCOUNT
} DCS1200_PWMName;

/*!
 *  @def    DCS1200_SPIName
 *  @brief  Enum of SPI names on the DCS1200 dev board
 */
typedef enum DCS1200_SPIName {
    DCS1200_SPI0 = 0,		/* SPARE EXPANSION SPI */
    DCS1200_SPI1,			/* MCP23S17SO Channels 1-8   */
    DCS1200_SPI2,			/* MCP23S17SO Channels 9-16  */
    DCS1200_SPI3,           /* MCP23S17SO Channels 17-24 */

    DCS1200_SPICOUNT
} DCS1200_SPIName;

/*!
 *  @def    DCS1200_UARTName
 *  @brief  Enum of UARTs on the DCS1200 dev board
 */
typedef enum DCS1200_UARTName {
    DCS1200_UART0 = 0,		/* UART0 debug console */
    DCS1200_UART1,			/* UART1 to STC-1200   */

    DCS1200_UARTCOUNT
} DCS1200_UARTName;

/*
 *  @def    DCS1200_WatchdogName
 *  @brief  Enum of Watchdogs on the DCS1200 dev board
 */
typedef enum DCS1200_WatchdogName {
    DCS1200_WATCHDOG0 = 0,

    DCS1200_WATCHDOGCOUNT
} DCS1200_WatchdogName;

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings. This include
 *     - Enable clock sources for peripherals
 */
extern void DCS1200_initGeneral(void);

/*!
 *  @brief  Initialize board specific GPIO settings
 *
 *  This function initializes the board specific GPIO settings and
 *  then calls the GPIO_init API to initialize the GPIO module.
 *
 *  The GPIOs controlled by the GPIO module are determined by the GPIO_config
 *  variable.
 */
extern void DCS1200_initGPIO(void);

extern uint32_t DCS1200_readDIPSwitch(void);

/*!
 *  @brief  Initialize board specific I2C settings
 *
 *  This function initializes the board specific I2C settings and then calls
 *  the I2C_init API to initialize the I2C module.
 *
 *  The I2C peripherals controlled by the I2C module are determined by the
 *  I2C_config variable.
 */
extern void DCS1200_initI2C(void);

/*!
 *  @brief  Initialize board specific PWM settings
 *
 *  This function initializes the board specific PWM settings and then calls
 *  the PWM_init API to initialize the PWM module.
 *
 *  The PWM peripherals controlled by the PWM module are determined by the
 *  PWM_config variable.
 */
extern void DCS1200_initPWM(void);

/*!
 *  @brief  Initialize board specific SPI settings
 *
 *  This function initializes the board specific SPI settings and then calls
 *  the SPI_init API to initialize the SPI module.
 *
 *  The SPI peripherals controlled by the SPI module are determined by the
 *  SPI_config variable.
 */
extern void DCS1200_initSPI(void);

/*!
 *  @brief  Initialize board specific UART settings
 *
 *  This function initializes the board specific UART settings and then calls
 *  the UART_init API to initialize the UART module.
 *
 *  The UART peripherals controlled by the UART module are determined by the
 *  UART_config variable.
 */
extern void DCS1200_initUART(void);

/*!
 *  @brief  Initialize board specific Watchdog settings
 *
 *  This function initializes the board specific Watchdog settings and then
 *  calls the Watchdog_init API to initialize the Watchdog module.
 *
 *  The Watchdog peripherals controlled by the Watchdog module are determined
 *  by the Watchdog_config variable.
 */
extern void DCS1200_initWatchdog(void);

#ifdef __cplusplus
}
#endif

#endif /* __DCS1200_TM4C123AE6PMI_H */
