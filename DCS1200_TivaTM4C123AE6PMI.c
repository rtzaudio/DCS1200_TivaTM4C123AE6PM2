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

#include <stdint.h>
#include <stdbool.h>

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ssi.h>
#include <inc/hw_i2c.h>

#include <driverlib/gpio.h>
#include <driverlib/flash.h>
#include <driverlib/eeprom.h>
#include <driverlib/sysctl.h>
#include <driverlib/i2c.h>
#include <driverlib/ssi.h>

#include <driverlib/gpio.h>
#include <driverlib/i2c.h>
#include <driverlib/pin_map.h>
#include <driverlib/pwm.h>
#include <driverlib/ssi.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <driverlib/udma.h>
#include <driverlib/adc.h>
#include <driverlib/qei.h>

#include "DCS1200_TivaTM4C123AE6PMI.h"

#ifndef TI_DRIVERS_UART_DMA
#define TI_DRIVERS_UART_DMA 0
#endif

/*
 *  =============================== DMA ===============================
 */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(dmaControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#endif
static tDMAControlTable dmaControlTable[32];
static bool dmaInitialized = false;

/* Hwi_Struct used in the initDMA Hwi_construct call */
static Hwi_Struct dmaHwiStruct;

/*
 *  ======== dmaErrorHwi ========
 */
static Void dmaErrorHwi(UArg arg)
{
    System_printf("DMA error code: %d\n", uDMAErrorStatusGet());
    uDMAErrorStatusClear();
    System_abort("DMA error!!");
}

/*
 *  ======== DCS1200_initDMA ========
 */
void DCS1200_initDMA(void)
{
    Error_Block eb;
    Hwi_Params  hwiParams;

    if (!dmaInitialized) {
        Error_init(&eb);
        Hwi_Params_init(&hwiParams);
        Hwi_construct(&(dmaHwiStruct), INT_UDMAERR, dmaErrorHwi, &hwiParams, &eb);
        if (Error_check(&eb)) {
            System_abort("Couldn't construct DMA error hwi");
        }

        SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
        uDMAEnable();
        uDMAControlBaseSet(dmaControlTable);

        dmaInitialized = true;
    }
}

/*
 *  =============================== General ===============================
 */
 
/*
 *  ======== DCS1200_initGeneral ========
 */
void DCS1200_initGeneral(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

	// Initialize the EEPROM so we can access it later

    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

    if (EEPROMInit() != EEPROM_INIT_OK)
    	System_printf("EEPROMInit() failed!\n");

    uint32_t size = EEPROMSizeGet();
}

/*
 *  =============================== GPIO ===============================
 */
 
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(GPIOTiva_config, ".const:GPIOTiva_config")
#endif

#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOTiva.h>

/* GPIO configuration structure */

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in DCS1200_TM4C123AE6PMI.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array.  Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {
    /*=== Input pins ===*/
    // GPIO_PB2 (BOOT_N) IN
    GPIOTiva_PB_2 | GPIO_CFG_INPUT,
    // GPIO_PE0 (CFG1) IN
    GPIOTiva_PE_0 | GPIO_CFG_INPUT | GPIO_CFG_IN_PU,
    // GPIO_PE1 (CFG2) IN
    GPIOTiva_PE_1 | GPIO_CFG_INPUT | GPIO_CFG_IN_PU,
    // GPIO_PE2 (CFG3) IN
    GPIOTiva_PE_2 | GPIO_CFG_INPUT | GPIO_CFG_IN_PU,
    // GPIO_PE3 (CFG4) IN
    GPIOTiva_PE_3 | GPIO_CFG_INPUT | GPIO_CFG_IN_PU,
    /*=== Output pins ===*/
    // GPIO_PG4 (DOUT1) OUT
    GPIOTiva_PG_4 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    // GPIO_PG5 (DOUT2) OUT
    GPIOTiva_PG_5 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    // GPIO_PB3 (SSI2FSS2) OUT
    GPIOTiva_PB_3 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    // GPIO_PB5 (SSI2FSS1) OUT
    GPIOTiva_PB_5 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    // GPIO_PD1 (SSI3FSS1) OUT
    GPIOTiva_PD_1 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    // GPIO_PD4 (SSI3FSS2) OUT
    GPIOTiva_PD_4 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    // GPIO_PD5 (SPEED) OUT
    GPIOTiva_PD_5 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    // GPIO_PD6 (LED_STAT) OUT
    GPIOTiva_PD_6 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    // GPIO_PD7 (RESET_IOX_N) OUT
    GPIOTiva_PD_7 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    // GPIO_PF3 (SSI1FSS1) OUT
    GPIOTiva_PF_3 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    // GPIO_PF4 (SSI1FSS2) OUT
    GPIOTiva_PF_4 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in DCS1200_TivaTM4C123AE6PMI.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,  /* DCS1200_MCP23S17T_INT1A (PG4) */
    NULL   /* DCS1200_MCP23S17T_INT2B (PG3) */
};

/* The device-specific GPIO_config structure */
const GPIOTiva_Config GPIOTiva_config = {
    .pinConfigs         = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks          = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = sizeof(gpioPinConfigs)/sizeof(GPIO_PinConfig),
    .numberOfCallbacks  = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority        = (~0)
};

/*
 *  ======== DCS1200_initGPIO ========
 */
void DCS1200_initGPIO(void)
{
    // GPIO_PB2 (BOOT_N)
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
    // GPIO_PG4 (DOUT1)
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_4);
    // GPIO_PG5 (DOUT2)
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_5);
    // GPIO_PB3 (SSI2FSS2)
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);
    // GPIO_PB5 (SSI2FSS1)
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
    // GPIO_PD1 (SSI3FSS1)
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
    // GPIO_PD4 (SSI3FSS2)
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_4);
    // for GPIO_PD5 (SPEED)
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_5);
    // GPIO_PD6 (LED_STAT)
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);
    // Unlock the Port Pin and Set the Commit Bit
    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE+GPIO_O_CR)   |= GPIO_PIN_7;
    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = 0x0;
    // GPIO_PD7 (RESET_IOX_N)
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);
    // GPIO_PE0 (CFG1)
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);
    // GPIO_PE1 (CFG2)
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
    // GPIO_PE2 (CFG3)
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2);
    // GPIO_PE3 (CFG4)
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_3);
    // GPIO_PF3 (SSI1FSS1)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
    // GPIO_PF4 (SSI1FSS2)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);

    /* Once GPIO_init is called, GPIO_config cannot be changed */
    GPIO_init();
}

/*
 *  =============================== I2C ===============================
 */
 
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(I2C_config, ".const:I2C_config")
#pragma DATA_SECTION(i2cTivaHWAttrs, ".const:i2cTivaHWAttrs")
#endif

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CTiva.h>

/* I2C objects */
I2CTiva_Object i2cTivaObjects[DCS1200_I2CCOUNT];

/* I2C configuration structure, describing which pins are to be used */

const I2CTiva_HWAttrs i2cTivaHWAttrs[DCS1200_I2CCOUNT] = {
    {
        .baseAddr    = I2C0_BASE,
        .intNum      = INT_I2C0,
        .intPriority = (~0)
    },
};

const I2C_Config I2C_config[] = {
    {
        .fxnTablePtr = &I2CTiva_fxnTable,
        .object      = &i2cTivaObjects[0],
        .hwAttrs     = &i2cTivaHWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== DCS1200_initI2C ========
 */
void DCS1200_initI2C(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    // Configure the GPIO Pin Mux for PE4
    // for I2C2SCL
    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);

    // Configure the GPIO Pin Mux for PE5
    // for I2C2SDA
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);


    I2C_init();
}


/*
 *  =============================== SPI ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(SPI_config, ".const:SPI_config")
#pragma DATA_SECTION(spiTivaDMAHWAttrs, ".const:spiTivaDMAHWAttrs")
#endif

#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPITivaDMA.h>

SPITivaDMA_Object spiTivaDMAObjects[DCS1200_SPICOUNT];

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(spiTivaDMAscratchBuf, 32)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=32
#elif defined(__GNUC__)
__attribute__ ((aligned (32)))
#endif
uint32_t spiTivaDMAscratchBuf[DCS1200_SPICOUNT];

const SPITivaDMA_HWAttrs spiTivaDMAHWAttrs[DCS1200_SPICOUNT] = {
    {
        .baseAddr               = SSI0_BASE,
        .intNum                 = INT_SSI0,
        .intPriority            = (~0),
        .scratchBufPtr          = &spiTivaDMAscratchBuf[0],
        .defaultTxBufValue      = 0,
        .rxChannelIndex         = UDMA_CHANNEL_SSI0RX,
        .txChannelIndex         = UDMA_CHANNEL_SSI0TX,
        .channelMappingFxn      = uDMAChannelAssign,
        .rxChannelMappingFxnArg = UDMA_CH10_SSI0RX,
        .txChannelMappingFxnArg = UDMA_CH11_SSI0TX
    },
    {
        .baseAddr               = SSI1_BASE,
        .intNum                 = INT_SSI1,
        .intPriority            = (~0),
        .scratchBufPtr          = &spiTivaDMAscratchBuf[1],
        .defaultTxBufValue      = 0,
        .rxChannelIndex         = UDMA_CHANNEL_SSI1RX,
        .txChannelIndex         = UDMA_CHANNEL_SSI1TX,
        .channelMappingFxn      = uDMAChannelAssign,
        .rxChannelMappingFxnArg = UDMA_CH24_SSI1RX,
        .txChannelMappingFxnArg = UDMA_CH25_SSI1TX
    },
	{
        .baseAddr               = SSI2_BASE,
        .intNum                 = INT_SSI2,
        .intPriority            = (~0),
        .scratchBufPtr          = &spiTivaDMAscratchBuf[2],
        .defaultTxBufValue      = 0,
        .rxChannelIndex         = UDMA_SEC_CHANNEL_UART2RX_12,
        .txChannelIndex         = UDMA_SEC_CHANNEL_UART2TX_13,
        .channelMappingFxn      = uDMAChannelAssign,
        .rxChannelMappingFxnArg = UDMA_CH12_SSI2RX,
        .txChannelMappingFxnArg = UDMA_CH13_SSI2TX
    },
    {
        .baseAddr               = SSI3_BASE,
        .intNum                 = INT_SSI3,
        .intPriority            = (~0),
        .scratchBufPtr          = &spiTivaDMAscratchBuf[3],
        .defaultTxBufValue      = 0,
        .rxChannelIndex         = UDMA_CHANNEL_I2S0RX,
        .txChannelIndex         = UDMA_CHANNEL_I2S0TX,
        .channelMappingFxn      = uDMAChannelAssign,
        .rxChannelMappingFxnArg = UDMA_CH12_SSI2RX,
        .txChannelMappingFxnArg = UDMA_CH13_SSI2TX
    }
};

const SPI_Config SPI_config[] = {
    {&SPITivaDMA_fxnTable, &spiTivaDMAObjects[0], &spiTivaDMAHWAttrs[0]},
    {&SPITivaDMA_fxnTable, &spiTivaDMAObjects[1], &spiTivaDMAHWAttrs[1]},
    {&SPITivaDMA_fxnTable, &spiTivaDMAObjects[2], &spiTivaDMAHWAttrs[2]},
    {&SPITivaDMA_fxnTable, &spiTivaDMAObjects[3], &spiTivaDMAHWAttrs[3]},
    {NULL, NULL, NULL},
};

/*
 *  ======== DCS1200_initSPI ========
 */
void DCS1200_initSPI(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);

    /* === Configure and Enable SSI0 === */

    // Configure GPIO PA4 for SSI0RX
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_4);
    // Configure GPIO PA3 for SSI0FSS
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_3);
    // Configure GPIO PA5 for SSI0TX
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5);
    // Configure GPIO PA2 for SSI0CLK
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2);

    /* === Configure and Enable SSI1 === */

    // Unlock the Port Pin and Set the Commit Bit
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR)   |= GPIO_PIN_0;
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = 0x0;
    // Configure GPIO PF0 for SSI1RX
    GPIOPinConfigure(GPIO_PF0_SSI1RX);
    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0);
    // Configure GPIO PF1 for SSI1TX
    GPIOPinConfigure(GPIO_PF1_SSI1TX);
    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_1);
    // Configure GPIO PF2 for SSI1CLK
    GPIOPinConfigure(GPIO_PF2_SSI1CLK);
    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_2);

    /* === Configure and Enable SSI2 === */

    // Configure GPIO PB6 for SSI2RX
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_6);
    // Configure GPIO PB7 for SSI2TX
    GPIOPinConfigure(GPIO_PB7_SSI2TX);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_7);
    // Configure GPIO PB4 for SSI2CLK
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4);

    /* === Configure and Enable SSI3 === */

    // Configure GPIO PD2 for SSI3RX
    GPIOPinConfigure(GPIO_PD2_SSI3RX);
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_2);
    // Configure GPIO PD3 for SSI3TX
    GPIOPinConfigure(GPIO_PD3_SSI3TX);
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3);
    // Configure GPIO PD0 for SSI3CLK
    GPIOPinConfigure(GPIO_PD0_SSI3CLK);
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0);

    DCS1200_initDMA();
    SPI_init();
}

/*
 *  =============================== UART ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UART_config, ".const:UART_config")
#pragma DATA_SECTION(uartTivaHWAttrs, ".const:uartTivaHWAttrs")
#endif

#include <ti/drivers/UART.h>
#if TI_DRIVERS_UART_DMA
#include <ti/drivers/uart/UARTTivaDMA.h>

UARTTivaDMA_Object uartTivaObjects[DCS1200_UARTCOUNT];

const UARTTivaDMA_HWAttrs uartTivaHWAttrs[DCS1200_UARTCOUNT] = {
    {
        .baseAddr       = UART0_BASE,
        .intNum         = INT_UART0,
        .intPriority    = (~0),
        .rxChannelIndex = UDMA_CH8_UART0RX,
        .txChannelIndex = UDMA_CH9_UART0TX,
    },
    {
        .baseAddr       = UART1_BASE,
        .intNum         = INT_UART1,
        .intPriority    = (~0),
        .rxChannelIndex = UDMA_CH22_UART1RX,
        .txChannelIndex = UDMA_CH23_UART1TX,
    },
};

const UART_Config UART_config[] = {
    {
        .fxnTablePtr = &UARTTivaDMA_fxnTable,
        .object      = &uartTivaObjects[0],
        .hwAttrs     = &uartTivaHWAttrs[0]
    },
    {
        .fxnTablePtr = &UARTTivaDMA_fxnTable,
        .object      = &uartTivaObjects[1],
        .hwAttrs     = &uartTivaHWAttrs[1]
    },
    {NULL, NULL, NULL}
};
#else
#include <ti/drivers/uart/UARTTiva.h>

UARTTiva_Object uartTivaObjects[DCS1200_UARTCOUNT];
unsigned char uartTivaRingBuffer[DCS1200_UARTCOUNT][32];

/* UART configuration structure */
const UARTTiva_HWAttrs uartTivaHWAttrs[DCS1200_UARTCOUNT] = {
    {
        .baseAddr    = UART0_BASE,
        .intNum      = INT_UART0,
        .intPriority = (~0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer[0],
        .ringBufSize = sizeof(uartTivaRingBuffer[0])
    },
    {
        .baseAddr    = UART1_BASE,
        .intNum      = INT_UART1,
        .intPriority = (~0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer[1],
        .ringBufSize = sizeof(uartTivaRingBuffer[1])
    },
};

const UART_Config UART_config[] = {
    {
        .fxnTablePtr = &UARTTiva_fxnTable,
        .object      = &uartTivaObjects[0],
        .hwAttrs     = &uartTivaHWAttrs[0]
    },
    {
        .fxnTablePtr = &UARTTiva_fxnTable,
        .object      = &uartTivaObjects[1],
        .hwAttrs     = &uartTivaHWAttrs[1]
    },
    {NULL, NULL, NULL}
};
#endif /* TI_DRIVERS_UART_DMA */

/*
 *  ======== DCS1200_initUART ========
 */
void DCS1200_initUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    // Configure the GPIO Pin Mux for PA0
    // for U0RX
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0);
    // Configure the GPIO Pin Mux for PA1
    // for U0TX
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);

    // Configure the GPIO Pin Mux for PB0
    // for U1RX
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0);
    // Configure the GPIO Pin Mux for PB1
    // for U1TX
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_1);
    // Configure the GPIO Pin Mux for PC4
    // for U1RTS
    GPIOPinConfigure(GPIO_PC4_U1RTS);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4);
    // Configure the GPIO Pin Mux for PC5
    // for U1CTS
    GPIOPinConfigure(GPIO_PC5_U1CTS);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_5);

    /* Initialize the UART driver */
#if TI_DRIVERS_UART_DMA
    DCS1200_initDMA();
#endif

    UART_init();
}


#if 0
/*
 *  =============================== PWM ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(PWM_config, ".const:PWM_config")
#pragma DATA_SECTION(pwmTivaHWAttrs, ".const:pwmTivaHWAttrs")
#endif

#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTiva.h>

PWMTiva_Object pwmTivaObjects[DCS1200_PWMCOUNT];

const PWMTiva_HWAttrs pwmTivaHWAttrs[DCS1200_PWMCOUNT] = {
    {
        .baseAddr = PWM1_BASE,
        .pwmOutput = PWM_OUT_6,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN
    },
    {
        .baseAddr = PWM1_BASE,
        .pwmOutput = PWM_OUT_7,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN
    }
};

const PWM_Config PWM_config[] = {
    {
        .fxnTablePtr = &PWMTiva_fxnTable,
        .object = &pwmTivaObjects[0],
        .hwAttrs = &pwmTivaHWAttrs[0]
    },
    {
        .fxnTablePtr = &PWMTiva_fxnTable,
        .object = &pwmTivaObjects[1],
        .hwAttrs = &pwmTivaHWAttrs[1]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== DCS1200_initPWM ========
 */
void DCS1200_initPWM(void)
{
    /* Enable PWM peripherals */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    /*
     * Enable PWM output on GPIO pins.  Board_LED1 and Board_LED2 are now
     * controlled by PWM peripheral - Do not use GPIO APIs.
     */
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 |GPIO_PIN_3);

    PWM_init();
}
#endif

/*
 *  =============================== Watchdog ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(Watchdog_config, ".const:Watchdog_config")
#pragma DATA_SECTION(watchdogTivaHWAttrs, ".const:watchdogTivaHWAttrs")
#endif

#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogTiva.h>

WatchdogTiva_Object watchdogTivaObjects[DCS1200_WATCHDOGCOUNT];

const WatchdogTiva_HWAttrs watchdogTivaHWAttrs[DCS1200_WATCHDOGCOUNT] = {
    {
        .baseAddr    = WATCHDOG0_BASE,
        .intNum      = INT_WATCHDOG,
        .intPriority = (~0),
        .reloadValue = 80000000 // 1 second period at default CPU clock freq
    },
};

const Watchdog_Config Watchdog_config[] = {
    {
        .fxnTablePtr = &WatchdogTiva_fxnTable,
        .object      = &watchdogTivaObjects[0],
        .hwAttrs     = &watchdogTivaHWAttrs[0]
    },
    {NULL, NULL, NULL},
};

/*
 *  ======== DCS1200_initWatchdog ========
 *
 * NOTE: To use the other watchdog timer with base address WATCHDOG1_BASE,
 *       an additional function call may need be made to enable PIOSC. Enabling
 *       WDOG1 does not do this. Enabling another peripheral that uses PIOSC
 *       such as ADC0 or SSI0, however, will do so. Example:
 *
 *       SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
 *       SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);
 *
 *       See the following forum post for more information:
 *       http://e2e.ti.com/support/microcontrollers/stellaris_arm_cortex-m3_microcontroller/f/471/p/176487/654390.aspx#654390
 */
void DCS1200_initWatchdog(void)
{
    /* Enable peripherals used by Watchdog */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

    Watchdog_init();
}

/* End-Of-File */
