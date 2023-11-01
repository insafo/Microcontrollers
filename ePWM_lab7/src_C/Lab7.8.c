/*
 * Task7.8.c
 *
 *  Created on: 26/10/2023
 *      Author: Inacio Fonseca
 *      Adapted from TI
 */
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// FROM Mundus_lib.c
extern void display_toLeds(unsigned int counter);
extern void Gpio_select(void);
extern float changeTIMER0FREQ(int hex);
extern int readHexEncoder();
extern int readButtonPB1();
extern int readButtonPB2();

// Prototype statements In This FILE
void Setup_ePWM1(void);
interrupt void cpu_timer0_isr(void);
interrupt void ePWM1_TZ_isr(void);

int counter=0;  // binary counter for digital output
// Main
void main(void) {
    InitSysCtrl();  // Basic Core Init from DSP2833x_SysCtrl.c

    EALLOW;
    SysCtrlRegs.WDCR= 0x00AF;   // Re-enable the watchdog
    EDIS;           // 0x00AF  to NOT disable the Watchdog, Prescaler = 64

    DINT;               // Disable all interrupts

    Gpio_select();      // GPIO9, GPIO11, GPIO34 and GPIO49 as output // to 4 LEDs at Peripheral Explorer)
    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 3; // GPIO17 as /TZ6
    EDIS;

    Setup_ePWM1();     // init of ePWM1A

    InitPieCtrl(); // in the DSP2833x_PieCtrl.c
    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // in DSP2833x_PieVect.c.
    InitPieVectTable();

    EALLOW;
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.EPWM1_TZINT = &ePWM1_TZ_isr;
    EDIS;

    InitCpuTimers();    // basic setup CPU Timer0, 1 and 2

    ConfigCpuTimer(&CpuTimer0,150,100000); // 100 ms
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // Enable PIE INT1.7 | CPU - Timer 0
    PieCtrlRegs.PIEIER2.bit.INTx1 = 1; // Enable PIE INT2.1 | ePWM1-TZ
    IER |= M_INT1 | M_INT2;  // Enable INT1 and INT2 in F28335

    EINT;
    ERTM;

    CpuTimer0Regs.TCR.bit.TSS = 0;  // start timer0

    while(1) {
        while(CpuTimer0.InterruptCount < 1);
        CpuTimer0.InterruptCount = 0;

        EALLOW;
        SysCtrlRegs.WDKEY = 0x55;   // service WD #1
        EDIS;

        counter++;
        //display_toLeds(counter);
    }
}


void Setup_ePWM1(void) {
   ?????
}

interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    EALLOW;
    SysCtrlRegs.WDKEY = 0xAA;   // service WD #2
    EDIS;

    GpioDataRegs.GPATOGGLE.bit.GPIO11 = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
interrupt void ePWM1_TZ_isr(void)
{
    EALLOW;
    EPwm1Regs.TZCLR.bit.CBC = 1;
    EPwm1Regs.TZCLR.bit.INT = 1;    // reset interrupt flags
    SysCtrlRegs.WDKEY = 0x55;       // service WD #1
    EDIS;
    GpioDataRegs.GPATOGGLE.bit.GPIO9 = 1;   // toggle LED at GPIO9
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}
