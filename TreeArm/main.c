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
#include "inc/hw_memmap.h"
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
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "stdio.h"
#include "utils/random.h"
#include "utils/random.c"
#include "utils/ustdlib.c"
#include "inc/hw_ints.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

//*****************************************************************************************
//  Global Variables
//*****************************************************************************************
bool Service_Timer0 = false;   //Flag for Timer 0 Interrupt Service

//*****************************************************************************************
//  Definitions
//*****************************************************************************************
#define TIMER0LOAD      0x000A2C2A    //Load Value for Timer0,     ~60Hz at 40MHz Sys Clock
#define LED_BUF_SIZE    300
#define UART_BUF_SIZE   96

//*****************************************************************************************
//  Function:       Timer0IntHandler
//  Description:    Interrupt Handler for Timer 0
//  Parameters:     None
//  Return:         None
//*****************************************************************************************
void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //Clear Timer0 Interrupt Flag
    IntMasterDisable();                                 //Disable Interrupts
    Service_Timer0 = true;
    IntMasterEnable();                                  //Enable Interrupts
}

//*****************************************************************************************
//  Function:       Main
//  Description:    Solution for Extra Credit Lab - UART
//  Parameters:     None
//  Return:         0 if everything works
//*****************************************************************************************
int main(void)
{
    tContext sContext;
    char str[32];               //String to format results
    int count = 0;
    //
    //Graphics Setup
    //
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                           SYSCTL_XTAL_16MHZ);

    CFAL96x64x16Init();
    GrContextInit(&sContext, &g_sCFAL96x64x16);
    GrContextForegroundSet(&sContext, ClrWhite);
    GrContextFontSet(&sContext, g_psFontCm12);
    //
    //UART setup
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
    {
    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART4))
    {
    }

    GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure((GPIO_PJ0_U4RX|GPIO_PJ1_U4TX));
    UARTConfigSetExpClk(UART4_BASE, SysCtlClockGet(), 4000000, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    //HWREG(UART4_BASE + UART_O_CTL) ^= UART_CTL_HSE;
    //UARTFIFOEnable(UART4_BASE);
    UARTEnable(UART4_BASE);
     //
    //Timer setup, using Timer 0 as a full width timer
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);       //Enable Timer0
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) //Wait for peripheral to finish init
    {
    }
    IntMasterEnable();                                  //Enable Interrupts
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); //Configure Timer0 to Full-Width, Periodic
    TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER0LOAD); //Load Timer0 with value defined by TIMER0LOAD
    IntEnable(INT_TIMER0A);                            //Enable Timer0 Interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //Set Timer0 Interrupt type
    TimerEnable(TIMER0_BASE, TIMER_A);                  //Enable Timer0
    sprintf(str, "Clock: %d", SysCtlClockGet());   //format result string
    GrStringDrawCentered(&sContext, str, -1,
                         GrContextDpyWidthGet(&sContext) / 2, 15, 1); //Draw line 1

    while (1)
    {
        if (Service_Timer0)
        {
            Service_Timer0 = false;     //Reset Timer0 Interrupt Service Flag


            sprintf(str, "Seconds: %d", count);   //format result string
            GrStringDrawCentered(&sContext, str, -1,
                                 GrContextDpyWidthGet(&sContext) / 2, 25, 1); //Draw line 1
            count+=1;
            //UARTBreakCtl(UART4_BASE, false);
            UARTCharPutNonBlocking (UART4_BASE, 0xFE);
            //UARTCharPutNonBlocking (UART4_BASE, 0xEF);
            UARTCharPutNonBlocking (UART4_BASE, 0xFE);
            //UARTCharPutNonBlocking (UART4_BASE, 0xF0);
            UARTCharPutNonBlocking (UART4_BASE, 0xFE);
            //UARTCharPutNonBlocking (UART4_BASE, 0xF0);
            UARTCharPutNonBlocking (UART4_BASE, 0xFE);

            //UARTBreakCtl(UART4_BASE, true);
        }

    }
    return 0;
}
