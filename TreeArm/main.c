//*****************************************************************************************
//  Embedded System Design Using ARM Technology
//  Final Project - Tree ARM
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
#include "drivers/buttons.h"

//*****************************************************************************************
//  Definitions
//*****************************************************************************************
#define TIMER0LOAD          0x007A1200  //Load Value for Timer0, 2 Hz at 80 MHz Sys Clock
#define TIMER1LOAD          0x003D0900  //Load Value for Timer1, 20 Hz at 80 MHz Sys Clock
#define red_start           64          //Starting index value for red sine wave
#define green_start         128         //Starting index value for green sine wave
#define blue_start          192         //Starting index value for blue sine wave
#define LED_SIZE            100         //Number of RGB leds in light strand
#define UART_BUF_SIZE       96          //Size of buffer in bytes for UART ping pong
#define FPS                 60          //Number of times a second to refresh the lights
#define BAUD_RATE           9600000     //Baud rate for UART = ~800kb/s to the lights

//
//  Assembly function to load TX Buffers from the LED buffer
//
extern void __load_buffer(uint8_t * source_r, uint8_t * source_g,
                          uint8_t * source_b, uint8_t * buffer);

//
// Sine wave table
//
const uint8_t sine_wave[256] =
        { 0x80, 0x83, 0x86, 0x89, 0x8C, 0x90, 0x93, 0x96, 0x99, 0x9C, 0x9F,
          0xA2, 0xA5, 0xA8, 0xAB, 0xAE, 0xB1, 0xB3, 0xB6, 0xB9, 0xBC, 0xBF,
          0xC1, 0xC4, 0xC7, 0xC9, 0xCC, 0xCE, 0xD1, 0xD3, 0xD5, 0xD8, 0xDA,
          0xDC, 0xDE, 0xE0, 0xE2, 0xE4, 0xE6, 0xE8, 0xEA, 0xEB, 0xED, 0xEF,
          0xF0, 0xF1, 0xF3, 0xF4, 0xF5, 0xF6, 0xF8, 0xF9, 0xFA, 0xFA, 0xFB,
          0xFC, 0xFD, 0xFD, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFD, 0xFD, 0xFC, 0xFB, 0xFA, 0xFA,
          0xF9, 0xF8, 0xF6, 0xF5, 0xF4, 0xF3, 0xF1, 0xF0, 0xEF, 0xED, 0xEB,
          0xEA, 0xE8, 0xE6, 0xE4, 0xE2, 0xE0, 0xDE, 0xDC, 0xDA, 0xD8, 0xD5,
          0xD3, 0xD1, 0xCE, 0xCC, 0xC9, 0xC7, 0xC4, 0xC1, 0xBF, 0xBC, 0xB9,
          0xB6, 0xB3, 0xB1, 0xAE, 0xAB, 0xA8, 0xA5, 0xA2, 0x9F, 0x9C, 0x99,
          0x96, 0x93, 0x90, 0x8C, 0x89, 0x86, 0x83, 0x80, 0x7D, 0x7A, 0x77,
          0x74, 0x70, 0x6D, 0x6A, 0x67, 0x64, 0x61, 0x5E, 0x5B, 0x58, 0x55,
          0x52, 0x4F, 0x4D, 0x4A, 0x47, 0x44, 0x41, 0x3F, 0x3C, 0x39, 0x37,
          0x34, 0x32, 0x2F, 0x2D, 0x2B, 0x28, 0x26, 0x24, 0x22, 0x20, 0x1E,
          0x1C, 0x1A, 0x18, 0x16, 0x15, 0x13, 0x11, 0x10, 0x0F, 0x0D, 0x0C,
          0x0B, 0x0A, 0x08, 0x07, 0x06, 0x06, 0x05, 0x04, 0x03, 0x03, 0x02,
          0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02,
          0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x06, 0x07, 0x08, 0x0A, 0x0B,
          0x0C, 0x0D, 0x0F, 0x10, 0x11, 0x13, 0x15, 0x16, 0x18, 0x1A, 0x1C,
          0x1E, 0x20, 0x22, 0x24, 0x26, 0x28, 0x2B, 0x2D, 0x2F, 0x32, 0x34,
          0x37, 0x39, 0x3C, 0x3F, 0x41, 0x44, 0x47, 0x4A, 0x4D, 0x4F, 0x52,
          0x55, 0x58, 0x5B, 0x5E, 0x61, 0x64, 0x67, 0x6A, 0x6D, 0x70, 0x74,
          0x77, 0x7A, 0x7D };

//
// Gamma correction table for leds
//
const uint8_t gamma[] =
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2,
          2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6,
          7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 13, 13,
          13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21,
          22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32,
          33, 34, 35, 35, 36, 37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47,
          48, 49, 50, 50, 51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64,
          66, 67, 68, 69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86,
          87, 89, 90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110,
          112, 114, 115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135,
          137, 138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162,
          164, 167, 169, 171, 173, 175, 177, 180, 182, 184, 186, 189, 191, 193,
          196, 198, 200, 203, 205, 208, 210, 213, 215, 218, 220, 223, 225, 228,
          231, 233, 236, 239, 241, 244, 247, 249, 252, 255 };

//*****************************************************************************************
//  Global Variables
//*****************************************************************************************
bool Service_SysTick = false;           //Flag for SysTick Interrupt Service
bool Service_Timer0 = false;            //Flag for Timer0 Interrupt Service
bool Service_Timer1 = false;            //Flag for Timer1 Interrupt Service
static uint8_t LEDBufA[3][LED_SIZE];    //Buffer holding RGB values for all leds
static uint8_t TxBufA[UART_BUF_SIZE];   //Tx Buffer A for UART uDMA ping pong
static uint8_t TxBufB[UART_BUF_SIZE];   //Tx Buffer B for UART uDMA ping pong
static uint32_t uDMAErrCount = 0;       //Error counter for uDMA errors
static uint32_t tx_sent = 0;
static int count = 0;
static char str[32];                    //String to format results
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
//  Description:    Interrupt Handler for Timer 0 - used for drawing to LED buffer
//  Parameters:     None
//  Return:         None
//*****************************************************************************************
void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //Clear Timer0 Interrupt Flag
    IntMasterDisable();                             //Disable Interrupts
    Service_Timer0 = true;                          //Set Service flag
    IntMasterEnable();                              //Enable Interrupts
}

//*****************************************************************************************
//  Function:       Timer1IntHandler
//  Description:    Interrupt Handler for Timer 1 - used for polling buttons
//  Parameters:     None
//  Return:         None
//*****************************************************************************************
void Timer1IntHandler(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); //Clear Timer1 Interrupt Flag
    IntMasterDisable();                             //Disable Interrupts
    Service_Timer1 = true;                          //Set Service flag
    IntMasterEnable();                              //Enable Interrupts
}

//*****************************************************************************************
//  Function:       SysTickIntHandler
//  Description:    Interrupt Handler for System Tick - used for sending frames to the leds
//  Parameters:     None
//  Return:         None
//*****************************************************************************************
void SysTickHandler(void)
{
    IntMasterDisable();         //Disable Interrupts
    Service_SysTick = true;     //Set Service flag
    count += 1;                 //Increment frame counter
    IntMasterEnable();          //Enable Interrupts
}

//*****************************************************************************************
//  Function:       uDMAErrorHandler
//  Description:    Interrupt Handler for uDMA errors
//  Parameters:     None
//  Return:         None
//*****************************************************************************************
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

//******************************************************************************
//  Function:       UART4IntHandler
//  Description:    Handler for UART interrupts, mostly used for uDMA control
//  Parameters:     None
//  Return:         None
//******************************************************************************
void UART4IntHandler(void)
{
    uint32_t UARTStatus, uDMAModeA, uDMAModeB;

    UARTStatus = UARTIntStatus(UART4_BASE, 0);
    UARTIntClear(UART4_BASE, UARTStatus);

    IntMasterDisable();
    if (tx_sent && (tx_sent <= LED_SIZE))
    {
        uDMAModeA = uDMAChannelModeGet(UDMA_CHANNEL_TMR0B | UDMA_PRI_SELECT);
        uDMAModeB = uDMAChannelModeGet(UDMA_CHANNEL_TMR0B | UDMA_ALT_SELECT);

        if (uDMAModeA == UDMA_MODE_STOP)
        {

            __load_buffer(&LEDBufA[0][tx_sent], &LEDBufA[1][tx_sent],
                          &LEDBufA[2][tx_sent], TxBufA);
            tx_sent += 4;
            uDMAChannelTransferSet(UDMA_CHANNEL_TMR0B | UDMA_PRI_SELECT,
            UDMA_MODE_PINGPONG,
                                   TxBufA, (void *) (UART4_BASE),
                                   UART_BUF_SIZE);
        }
        else if (uDMAModeB == UDMA_MODE_STOP)
        {
            __load_buffer(&LEDBufA[0][tx_sent], &LEDBufA[1][tx_sent],
                          &LEDBufA[2][tx_sent], TxBufB);
            tx_sent += 4;
            uDMAChannelTransferSet(
            UDMA_CHANNEL_TMR0B | UDMA_ALT_SELECT,
                                   UDMA_MODE_PINGPONG, TxBufB,
                                   (void *) (UART4_BASE), UART_BUF_SIZE);
        }
    }
    else if (tx_sent > LED_SIZE)
    {
        tx_sent = 0;
        uDMAChannelDisable(UDMA_CHANNEL_TMR0B);
    }
    IntMasterEnable();
}

//******************************************************************************
//  Function:       send_frame
//  Description:    Start the process of sending a frame to the LED strand
//  Parameters:     None
//  Return:         None
//******************************************************************************
void send_frame(void)
{
    tx_sent = 0;
    __load_buffer(&LEDBufA[0][tx_sent], &LEDBufA[1][tx_sent],
                  &LEDBufA[2][tx_sent], TxBufA);
    tx_sent += 4;
    __load_buffer(&LEDBufA[0][tx_sent], &LEDBufA[1][tx_sent],
                  &LEDBufA[2][tx_sent], TxBufB);
    tx_sent += 4;
    uDMAChannelTransferSet(UDMA_CHANNEL_TMR0B | UDMA_PRI_SELECT,
    UDMA_MODE_PINGPONG,
                           TxBufA, (void *) (UART4_BASE), UART_BUF_SIZE);
    uDMAChannelTransferSet(UDMA_CHANNEL_TMR0B | UDMA_ALT_SELECT,
    UDMA_MODE_PINGPONG,
                           TxBufB, (void *) (UART4_BASE), UART_BUF_SIZE);
    uDMAChannelEnable(UDMA_CHANNEL_TMR0B);
}

//******************************************************************************
//  Function:       init_clock
//  Description:    Initialize System Clock and SysTick
//  Parameters:     None
//  Return:         None
//******************************************************************************
void init_clock(void)
{
    //
    //Setup System Clock to 80MHz and System Tick SystemClock/FPS (60Hz)
    //
    SysCtlClockSet(
    SYSCTL_SYSDIV_2 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralClockGating(true);
    SysTickPeriodSet(SysCtlClockGet() / FPS);
    SysTickIntEnable();
    SysTickEnable();
}

//******************************************************************************
//  Function:       init_display
//  Description:    Initialize Timer 0 as full width timer with TIMER0LOAD
//  Parameters:     None
//  Return:         None
//******************************************************************************
void init_display(void)
{
    CFAL96x64x16Init();
    GrContextInit(&sContext, &g_sCFAL96x64x16);
    GrContextForegroundSet(&sContext, ClrWhite);
    GrContextFontSet(&sContext, g_psFontCm12);
}

//******************************************************************************
//  Function:       init_gpio
//  Description:    Initialize PORT J, pins 0 and 1 for use with UART4
//  Parameters:     None
//  Return:         None
//******************************************************************************
void init_gpio(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
    {
    }
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOJ);
    GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure((GPIO_PJ0_U4RX | GPIO_PJ1_U4TX));
    ButtonsInit();
}

//******************************************************************************
//  Function:       init_UART
//  Description:    Initialize UART4 for transmit with uDMA and FIFO
//  Parameters:     None
//  Return:         None
//******************************************************************************
void init_UART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART4))
    {
    }
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART4);
    UARTConfigSetExpClk(
            UART4_BASE, SysCtlClockGet(), BAUD_RATE,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTFIFOLevelSet(UART4_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
    UARTFIFOEnable(UART4_BASE);
    UARTEnable(UART4_BASE);
    UARTDMAEnable(UART4_BASE, UART_DMA_TX);
    IntEnable(INT_UART4);
}

//******************************************************************************
//  Function:       init_uDMA
//  Description:    Initialize uDMA for use with UART4TX
//  Parameters:     None
//  Return:         None
//******************************************************************************
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

//******************************************************************************
//  Function:       init_timer
//  Description:    Initialize Timer 0 as full width timer with TIMER0LOAD
//  Parameters:     None
//  Return:         None
//******************************************************************************
void init_timer(void)
{
    //Enable Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    //Wait for peripheral to finish init
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {
    }
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
    //Configure Timer0 to Full-Width, Periodic
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //Load Timer0 with value defined by TIMER0LOAD
    TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER0LOAD);
    //Enable Timer0 Interrupt
    IntEnable(INT_TIMER0A);
    //Set Timer0 Interrupt type
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //Enable Timer0
    TimerEnable(TIMER0_BASE, TIMER_A);
}

//******************************************************************************
//  Function:       Main
//  Description:    Main function for TreeArm Project
//  Parameters:     None
//  Return:         0 if everything works
//******************************************************************************
int main(void)
{
    uint8_t red_sine = red_start;       //red sine wave table index
    uint8_t green_sine = green_start;   //green sine wave table index
    uint8_t blue_sine = blue_start;     //blue sine wave table index
    int8_t red_diff = 1;              //red sine wave +- speed value
    int8_t green_diff = 1;            //green sine wave +- speed value
    int8_t blue_diff = 1;             //blue sine wave +- speed value

//
//Peripheral setup
//
    init_clock();
    init_display();
    init_uDMA();
    init_gpio();
    init_UART();
    init_timer();

    sprintf(str, "Clock: %d", SysCtlClockGet());   //format result string
    GrStringDrawCentered(&sContext, str, -1,
                         GrContextDpyWidthGet(&sContext) / 2, 15, 1); //Draw line 1
    IntMasterEnable();                                  //Enable Interrupts
//
//Main loop used to service SysTick and Timer0
//
    while (1)
    {
        if (Service_SysTick)
        {
            IntMasterDisable();                  //Disable Interrupts
            Service_SysTick = false;       //Reset Timer0 Interrupt Service Flag
            IntMasterEnable();                   //Enable Interrupts
            sprintf(str, "Frames: %d", count);   //format result string
            GrStringDrawCentered(&sContext, str, -1,
                                 GrContextDpyWidthGet(&sContext) / 2, 25, 1); //Draw line 1
            send_frame();                  //Start sending a frame to the strand
        }
        else if (Service_Timer0)
        {
            uint8_t i, buttons, buttons_diff;
            IntMasterDisable();         //Disable Interrupts
            Service_Timer0 = false;     //Reset Timer0 Interrupt Service Flag
            IntMasterEnable();          //Enable Interrupts
            buttons = ButtonsPoll(&buttons_diff, NULL);
            buttons = buttons & buttons_diff; //only want to know about buttons that just turned on
            switch (buttons)
            {
            case 0:
            {
                break;
            }
            case UP_BUTTON:
            {
                red_diff +=1;
                break;
            }
            case DOWN_BUTTON:
            {
                green_diff +=1;
                break;
            }
            case RIGHT_BUTTON:
            {
                blue_diff +=1;
                break;
            }
            case LEFT_BUTTON:
            {
                //
                //Reset colors to initial values
                //
                red_sine = red_start;
                green_sine = green_start;
                blue_sine = blue_start;
                red_diff = 1;
                green_diff = 1;
                blue_diff = 1;
                break;
            }
            case SELECT_BUTTON:
            {

                break;
            }
            default:
            {
                break;
            }
            }
            //
            //update buffer with new sine wave portions for each color
            //
            for (i = 0; i < LED_SIZE; i++)
            {
                LEDBufA[0][i] = gamma[sine_wave[(red_sine + i) % 256]];
                LEDBufA[1][i] = gamma[sine_wave[(green_sine + i) % 256]];
                LEDBufA[2][i] = gamma[sine_wave[(blue_sine + i) % 256]];
            }
            sprintf(str, "R: %d", buttons);   //format result string
            GrStringDrawCentered(&sContext, str, -1,
                                 GrContextDpyWidthGet(&sContext) / 2, 35, 1); //Draw line 1
            sprintf(str, "G: %d", green_sine);   //format result string
            GrStringDrawCentered(&sContext, str, -1,
                                 GrContextDpyWidthGet(&sContext) / 2, 45, 1); //Draw line 1
            sprintf(str, "B: %d", blue_sine);   //format result string
            GrStringDrawCentered(&sContext, str, -1,
                                 GrContextDpyWidthGet(&sContext) / 2, 55, 1); //Draw line 1
            red_sine -= red_diff;
            green_sine -= green_diff;
            blue_sine -= blue_diff;
        }
        else if(Service_Timer1)
        {
            IntMasterDisable();         //Disable Interrupts
            Service_Timer1 = false;     //Reset Timer1 Interrupt Service Flag
            IntMasterEnable();          //Enable Interrupts
            ButtonsPoll(NULL, NULL);    //Pole buttons for proper debouncing
        }
    }
    return 0;
}
