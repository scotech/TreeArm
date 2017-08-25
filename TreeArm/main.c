//*****************************************************************************************
//  Embedded System Design Using ARM Technology
//  Extra Credit Lab - ADC
//  Author: Nicholas Scoville
//  File: main.c
//*****************************************************************************************

//*****************************************************************************************
//  Include files
//*****************************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/fpu.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "grlib/grlib.h"
#include "drivers/cfal96x64x16.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include "driverlib/udma.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "stdio.h"
#include "utils/random.h"
#include "utils/random.c"
#include "utils/ustdlib.c"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

//*****************************************************************************************
//  Definitions
//*****************************************************************************************
#define TIMER0LOAD          0x000A2C2A    //Load Value for Timer0,     ~60Hz at 40MHz Sys Clock
#define LED_SIZE            100
#define UART_BUF_SIZE       96
#define MAX_FRAMES          25
#define FPS                 4

//*****************************************************************************************
//  Global Variables
//*****************************************************************************************
bool Service_Timer0 = false;   //Flag for Timer 0 Interrupt Service
static uint8_t LEDBufA[LED_SIZE][3];
static uint8_t LEDBufB[LED_SIZE][3];
static uint8_t TxBufA[UART_BUF_SIZE];
static uint8_t TxBufB[UART_BUF_SIZE];
static uint32_t uDMAErrCount = 0;
static uint32_t num_frames = 0;
static int count = 0;
static char str[32];               //String to format results
static tContext sContext;

//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t ui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ui8ControlTable, 1024)
uint8_t ui8ControlTable[1024];
#else
uint8_t ui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

//*****************************************************************************************
//  Function:       Timer0IntHandler
//  Description:    Interrupt Handler for Timer 0
//  Parameters:     None
//  Return:         None
//*****************************************************************************************
void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //Clear Timer0 Interrupt Flag
    //Do Some Work!!!                             //Enable Interrupts
}

void SysTickHandler(void)
{
    IntMasterDisable();                                 //Disable Interrupts
    Service_Timer0 = true;
    count += 1;
    IntMasterEnable();
}

void uDMAErrorHandler(void)
{
    uint32_t ui32Status;

    //
    // Check for uDMA error bit
    //
    ui32Status = uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if (ui32Status)
    {
        uDMAErrorStatusClear();
        uDMAErrCount++;
    }
}

void UART4IntHandler(void)
{
    uint32_t UARTStatus, uDMAModeA, uDMAModeB;

    UARTStatus = UARTIntStatus(UART4_BASE, 1);
    UARTIntClear(UART4_BASE, UARTStatus);

    IntMasterDisable();
    //if (num_frames && (num_frames < MAX_FRAMES))
    //if(1)
   //{
        uDMAModeA = uDMAChannelModeGet(UDMA_CHANNEL_TMR0B | UDMA_PRI_SELECT);
        uDMAModeB = uDMAChannelModeGet(UDMA_CHANNEL_TMR0B | UDMA_PRI_SELECT);

        if (uDMAModeA == UDMA_MODE_STOP)
        {

            uDMAChannelTransferSet(UDMA_CHANNEL_TMR0B | UDMA_PRI_SELECT,
            UDMA_MODE_PINGPONG,
                                   TxBufA, (void *) (UART4_BASE),
                                   UART_BUF_SIZE);
            num_frames++;
        }
        else if (uDMAModeB == UDMA_MODE_STOP)
        {
            uDMAChannelTransferSet(
            UDMA_CHANNEL_TMR0B | UDMA_ALT_SELECT,
                                   UDMA_MODE_PINGPONG, TxBufB,
                                   (void *) (UART4_BASE), UART_BUF_SIZE);
            num_frames++;
        }

        else
        {
/*            uDMAChannelTransferSet(UDMA_CHANNEL_TMR0B | UDMA_PRI_SELECT,
            UDMA_MODE_PINGPONG,
                                   TxBufA, (void *) (UART4_BASE), UART_BUF_SIZE);
            uDMAChannelTransferSet(UDMA_CHANNEL_TMR0B | UDMA_ALT_SELECT,
            UDMA_MODE_PINGPONG,
                                   TxBufB, (void *) (UART4_BASE), UART_BUF_SIZE);

            uDMAChannelEnable(UDMA_CHANNEL_TMR0B);
*/        }
    //}
    //count+=1;
        uDMAChannelEnable(UDMA_CHANNEL_TMR0B);
    IntMasterEnable();
}

void send_frame(void)
{
    uDMAChannelTransferSet(UDMA_CHANNEL_TMR0B | UDMA_PRI_SELECT,
    UDMA_MODE_PINGPONG,
                           TxBufA, (void *) (UART4_BASE), UART_BUF_SIZE);
    uDMAChannelTransferSet(UDMA_CHANNEL_TMR0B | UDMA_ALT_SELECT,
    UDMA_MODE_PINGPONG,
                           TxBufB, (void *) (UART4_BASE), UART_BUF_SIZE);

    uDMAChannelEnable(UDMA_CHANNEL_TMR0B);
    num_frames = 2;
}

void init_clock(void)
{
    //
    //Setup System Click and System Tick to 40Mhz and 60Hz
    //
    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
                    | SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralClockGating(true);
    SysTickPeriodSet(SysCtlClockGet() / FPS);
    SysTickIntEnable();
    SysTickEnable();
}

void init_display(void)
{
    CFAL96x64x16Init();
    GrContextInit(&sContext, &g_sCFAL96x64x16);
    GrContextForegroundSet(&sContext, ClrWhite);
    GrContextFontSet(&sContext, g_psFontCm12);
}

void init_gpo(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
    {
    }
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOJ);
    GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure((GPIO_PJ0_U4RX | GPIO_PJ1_U4TX));
}

void init_UART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART4))
    {
    }
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART4);
    UARTConfigSetExpClk(
            UART4_BASE, SysCtlClockGet(), 4000000,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTFIFOLevelSet(UART4_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
    UARTEnable(UART4_BASE);
    UARTDMAEnable(UART4_BASE, UART_DMA_TX);
    IntEnable(INT_UART4);

    //UARTFIFOEnable(UART4_BASE);

    //HWREG(UART4_BASE + UART_O_CTL) ^= UART_CTL_HSE;
}

void init_uDMA(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA))
    {
    }
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
    IntEnable(INT_UDMAERR);
    uDMAEnable();
    uDMAControlBaseSet(ui8ControlTable);
    uDMAChannelAssign(UDMA_CH19_UART4TX);

    uDMAChannelAttributeDisable(UDMA_CHANNEL_TMR0B,
    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
    UDMA_ATTR_HIGH_PRIORITY |
    UDMA_ATTR_REQMASK);
    uDMAChannelAttributeEnable(UDMA_CHANNEL_TMR0B, UDMA_ATTR_USEBURST);
    uDMAChannelControlSet(UDMA_CHANNEL_TMR0B | UDMA_PRI_SELECT,
    UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
    UDMA_ARB_4);
    uDMAChannelControlSet(UDMA_CHANNEL_TMR0B | UDMA_ALT_SELECT,
    UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
    UDMA_ARB_4);

}

void init_timer(void)
{
    //
    //Timer setup, using Timer 0 as a full width timer
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);       //Enable Timer0
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) //Wait for peripheral to finish init
    {
    }
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); //Configure Timer0 to Full-Width, Periodic
    TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER0LOAD); //Load Timer0 with value defined by TIMER0LOAD
    IntEnable(INT_TIMER0A);                            //Enable Timer0 Interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //Set Timer0 Interrupt type
    TimerEnable(TIMER0_BASE, TIMER_A);                  //Enable Timer0
}

//*****************************************************************************************
//  Function:       Main
//  Description:    Solution for Extra Credit Lab - UART
//  Parameters:     None
//  Return:         0 if everything works
//*****************************************************************************************
int main(void)
{
    for (count = 0; count < UART_BUF_SIZE; count++)
    {
        TxBufA[count] = 0xFE;
        TxBufB[count] = 0xF0;
        //TxBufB[count] =   0xF0;
        //TxBufA[count] =   0xAA;
        //TxBufB[count] =   0x55;
    }
    count = 0;
    //
    //Peripheral setup
    //
    init_clock();
    init_display();
    init_uDMA();
    init_gpo();
    init_UART();
    init_timer();

    sprintf(str, "Clock: %d", SysCtlClockGet());   //format result string
    GrStringDrawCentered(&sContext, str, -1,
                         GrContextDpyWidthGet(&sContext) / 2, 15, 1); //Draw line 1

    IntMasterEnable();                                  //Enable Interrupts
    send_frame();
    while (1)
    {
        if (Service_Timer0)
        {
            IntMasterDisable();                             //Disable Interrupts
            Service_Timer0 = false;     //Reset Timer0 Interrupt Service Flag
            IntMasterEnable();

            sprintf(str, "Seconds: %d", num_frames);   //format result string
            GrStringDrawCentered(&sContext, str, -1,
                                 GrContextDpyWidthGet(&sContext) / 2, 25, 1); //Draw line 1


            /*uDMAChannelTransferSet(UDMA_CHANNEL_TMR0B | UDMA_PRI_SELECT,
             UDMA_MODE_PINGPONG,TxBufA,
             (void *)(UART4_BASE), UART_BUF_SIZE);
             uDMAChannelTransferSet(UDMA_CHANNEL_TMR0B | UDMA_ALT_SELECT,
             UDMA_MODE_PINGPONG,TxBufB,
             (void *)(UART4_BASE), UART_BUF_SIZE);*/
            //uDMAChannelEnable(UDMA_CHANNEL_TMR0B);
            //uDMAChannelEnable(UDMA_CHANNEL_TMR0B);
            //UARTBreakCtl(UART4_BASE, false);
            // UARTCharPutNonBlocking (UART4_BASE, 0xFE);
            //UARTCharPutNonBlocking (UART4_BASE, 0xEF);
            // UARTCharPutNonBlocking (UART4_BASE, 0xFE);
            //UARTCharPutNonBlocking (UART4_BASE, 0xF0);
            // UARTCharPutNonBlocking (UART4_BASE, 0xFE);
            //UARTCharPutNonBlocking (UART4_BASE, 0xF0);
            //UARTCharPutNonBlocking (UART4_BASE, 0xFE);
            //UARTBreakCtl(UART4_BASE, true);
        }

    }
    return 0;
}
