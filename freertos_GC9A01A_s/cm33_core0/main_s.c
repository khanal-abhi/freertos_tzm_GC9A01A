/*
 * FreeRTOS Pre-Release V1.0.0
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/* Board specific includes. */

/* Trustzone config. */
#include "tzm_config.h"

/* FreeRTOS includes. */
#include "secure_port_macros.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_power.h"
#include "fsl_spi.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define VALID_STR "valid"
#define INVALID_STR "invalid"

#define GC9A01A_SPI_MASTER SPI7
#define GC9A01A_SPI_MASTER_IRQ FLEXCOMM7_IRQn
#define GC9A01A_SPI_MASTER_CLK_SRC kCLOCK_Flexcomm7
#define GC9A01A_SPI_MASTER_CLK_FREQ CLOCK_GetFlexCommClkFreq(7U)
#define GC9A01A_SPI_SSEL 1
#define GC9A01A_SPI_MASTER_RX_CHANNEL 18
#define GC9A01A_SPI_MASTER_TX_CHANNEL 19
#define GC9A01A_SPI_MASTER_SPI_SPOL kSPI_SpolActiveAllLow

#define TRANSFER_SIZE 64U /*! Transfer dataSize */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void GC9A01A_SPI_MasterInit(void);
void GC9A01A_SPI_MasterStarTransfer(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t masterRxData[TRANSFER_SIZE] = {0};
uint8_t masterTxData[TRANSFER_SIZE] = {0};

volatile bool isTransferCompleted = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void GC9A01A_SPI_MasterInit(void)
{
    /* SPI init */
    uint32_t srcClock_Hz = 0U;
    spi_master_config_t masterConfig;
    srcClock_Hz = GC9A01A_SPI_MASTER_CLK_FREQ;
    assert(!(srcClock_Hz == 0));

    SPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.sselNum = (spi_ssel_t)GC9A01A_SPI_SSEL;
    masterConfig.sselPol = (spi_spol_t)GC9A01A_SPI_MASTER_SPI_SPOL;
    SPI_MasterInit(GC9A01A_SPI_MASTER, &masterConfig, srcClock_Hz);
}

void GC9A01A_SPI_MasterStarTransfer(void)
{
    spi_transfer_t masterXfer;
    uint32_t i = 0U;

    /* Set up the transfer data */
    for (i = 0U; i < TRANSFER_SIZE; i++)
    {
        /* SPI is configured for 8 bits transfer - set only lower 8 bits of buffers */
        masterTxData[i] = i % 256U;
        masterRxData[i] = 0U;
    }

    /* Start master transfer */
    masterXfer.txData = (uint8_t *)&masterTxData;
    masterXfer.rxData = (uint8_t *)&masterRxData;
    masterXfer.dataSize = TRANSFER_SIZE * sizeof(masterTxData[0]);
    masterXfer.configFlags = kSPI_FrameAssert;

    status_t res = SPI_MasterTransferBlocking(GC9A01A_SPI_MASTER, &masterXfer);
    if (kStatus_Success != res)
    {
        PRINTF("Unable to tx data due to %d...\r\n", res);
    }
    else
    {
        PRINTF("Tx'd data successfully...\r\n");
    }
}

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**
 * @brief Start address of non-secure application.
 */
#define mainNONSECURE_APP_START_ADDRESS DEMO_CODE_START_NS

/**
 * @brief LED port and pins.
 */
#define LED_PORT BOARD_LED_BLUE_GPIO_PORT
#define GREEN_LED_PIN BOARD_LED_GREEN_GPIO_PIN
#define BLUE_LED_PIN BOARD_LED_BLUE_GPIO_PIN

/**
 * @brief typedef for non-secure Reset Handler.
 */
#if defined(__IAR_SYSTEMS_ICC__)
typedef __cmse_nonsecure_call void (*NonSecureResetHandler_t)(void);
#else
typedef void (*NonSecureResetHandler_t)(void) __attribute__((cmse_nonsecure_call));
#endif
/*-----------------------------------------------------------*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/**
 * @brief Application-specific implementation of the SystemInitHook().
 */
void SystemInitHook(void);

/**
 * @brief Boots into the non-secure code.
 *
 * @param[in] ulNonSecureStartAddress Start address of the non-secure application.
 */
void BootNonSecure(uint32_t ulNonSecureStartAddress);
/*-----------------------------------------------------------*/

/*******************************************************************************
 * Code
 ******************************************************************************/

void SystemInitHook(void)
{
    /* The TrustZone should be configured as early as possible after RESET.
     * Therefore it is called from SystemInit() during startup. The
     * SystemInitHook() weak function overloading is used for this purpose.
     */
    BOARD_InitTrustZone();
}
/*-----------------------------------------------------------*/

void BootNonSecure(uint32_t ulNonSecureStartAddress)
{
    NonSecureResetHandler_t pxNonSecureResetHandler;

    /* Main Stack Pointer value for the non-secure side is the first entry in
     * the non-secure vector table. Read the first entry and assign the same to
     * the non-secure main stack pointer(MSP_NS). */
    secureportSET_MSP_NS(*((uint32_t *)(ulNonSecureStartAddress)));

    /* Reset handler for the non-secure side is the second entry in the
     * non-secure vector table. */
    pxNonSecureResetHandler = (NonSecureResetHandler_t)(*((uint32_t *)((ulNonSecureStartAddress) + 4U)));

    /* Start non-secure state software application by jumping to the non-secure
     * reset handler. */
    pxNonSecureResetHandler();
}
/*-----------------------------------------------------------*/

/* Secure main(). */
/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */
    /* set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    gpio_pin_config_t xLedConfig = {.pinDirection = kGPIO_DigitalOutput, .outputLogic = 1};

    /* Initialize GPIO for LEDs. */
    GPIO_PortInit(GPIO, LED_PORT);
    GPIO_PinInit(GPIO, LED_PORT, GREEN_LED_PIN, &(xLedConfig));
    GPIO_PinInit(GPIO, LED_PORT, BLUE_LED_PIN, &(xLedConfig));

    /* Set non-secure vector table */
    SCB_NS->VTOR = mainNONSECURE_APP_START_ADDRESS;

    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM7);
    RESET_PeripheralReset(kFC7_RST_SHIFT_RSTn);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Initialize SPI master with configuration. */
    GC9A01A_SPI_MasterInit();

    /* Boot the non-secure code. */
    BootNonSecure(mainNONSECURE_APP_START_ADDRESS);

    /* Non-secure software does not return, this code is not executed. */
    for (;;)
    {
    }
}
/*-----------------------------------------------------------*/

void vGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
    /* These are volatile to try and prevent the compiler/linker optimising them
     * away as the variables never actually get used.  If the debugger won't show the
     * values of the variables, make them global my moving their declaration outside
     * of this function. */
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t r2;
    volatile uint32_t r3;
    volatile uint32_t r12;
    volatile uint32_t lr;  /* Link register. */
    volatile uint32_t pc;  /* Program counter. */
    volatile uint32_t psr; /* Program status register. */
    volatile uint32_t _CFSR;
    volatile uint32_t _HFSR;
    volatile uint32_t _DFSR;
    volatile uint32_t _AFSR;
    volatile uint32_t _SFSR;
    volatile uint32_t _BFAR;
    volatile uint32_t _MMAR;
    volatile uint32_t _SFAR;

    r0 = pulFaultStackAddress[0];
    r1 = pulFaultStackAddress[1];
    r2 = pulFaultStackAddress[2];
    r3 = pulFaultStackAddress[3];

    r12 = pulFaultStackAddress[4];
    lr = pulFaultStackAddress[5];
    pc = pulFaultStackAddress[6];
    psr = pulFaultStackAddress[7];

    /* Configurable Fault Status Register. Consists of MMSR, BFSR and UFSR. */
    _CFSR = (*((volatile unsigned long *)(0xE000ED28))); // 0x00100000

    /* Hard Fault Status Register. */
    _HFSR = (*((volatile unsigned long *)(0xE000ED2C))); // 0x40000000, must check others

    /* Debug Fault Status Register. */
    _DFSR = (*((volatile unsigned long *)(0xE000ED30))); // 0x00000001 (likely just means halt state while debugging)

    /* Auxiliary Fault Status Register. */
    _AFSR = (*((volatile unsigned long *)(0xE000ED3C))); // 0

    /* Secure Fault Status Register. */
    _SFSR = (*((volatile unsigned long *)(0xE000EDE4))); // 0

    /* Read the Fault Address Registers. Note that these may not contain valid
     * values. Check BFARVALID/MMARVALID to see if they are valid values. */
    /* MemManage Fault Address Register. */
    _MMAR = (*((volatile unsigned long *)(0xE000ED34)));

    /* Bus Fault Address Register. */
    _BFAR = (*((volatile unsigned long *)(0xE000ED38)));

    /* Secure Fault Address Register. */
    _SFAR = (*((volatile unsigned long *)(0xE000EDE8)));

    PRINTF("\r\n===================\r\n");
    PRINTF("!!! HARD FAULT !!!!\r\n");
    PRINTF("===================\r\n");
    PRINTF("===================\r\n");
    PRINTF("REG:  HEX VALUE\r\n");
    PRINTF("===================\r\n");
    PRINTF(" CFSR %08x\r\n", _CFSR);
    PRINTF(" HFSR %08x\r\n", _HFSR);
    PRINTF(" DFSR %08x\r\n", _DFSR);
    PRINTF(" AFSR %08x\r\n", _AFSR);
    PRINTF(" SFSR %08x\r\n", _SFSR);
    if (_CFSR & (1 << 7))
    {
        PRINTF(" MMAR 0x%08x\r\n", _MMAR);
    }
    if (_SFSR & (1 << 7))
    {
        PRINTF(" SFAR 0x%08x\r\n", _SFAR);
    }
    if (_CFSR & (1 << 15))
    {
        PRINTF(" BFAR 0x%08x\r\n", _BFAR);
    }
    PRINTF("Special cases:\r\n");
    if (_CFSR & (1 << 20))
    {
        PRINTF("  STKOF\r\n");
    }
    if (_CFSR & (1 << 24))
    {
        PRINTF("  UNALIGNED\r\n");
    }
    if (_CFSR & (1 << 25))
    {
        PRINTF("  DIVBYZERO\r\n");
    }
    PRINTF("\r\n===================\r\n");

    /* Remove compiler warnings about the variables not being used. */
    (void)r0;
    (void)r1;
    (void)r2;
    (void)r3;
    (void)r12;
    (void)lr;  /* Link register. */
    (void)pc;  /* Program counter. */
    (void)psr; /* Program status register. */
    (void)_CFSR;
    (void)_HFSR;
    (void)_DFSR;
    (void)_AFSR;
    (void)_SFSR;
    (void)_MMAR;
    (void)_BFAR;
    (void)_SFAR;

    /* When the following line is hit, the variables contain the register values. */
    for (;;)
    {
    }
}
// /*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/**
 * @brief The fault handler implementation calls a function called
 * vGetRegistersFromStack().
 */
void HardFault_Handler(void)
{
    __asm volatile(
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, handler2_address_const                            \n"
        " bx r1                                                     \n"
        "                                                           \n"
        " handler2_address_const: .word vGetRegistersFromStack      \n");
}
/*-----------------------------------------------------------*/
