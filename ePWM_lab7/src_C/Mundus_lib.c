/*
 * Mundus_lib.c
 *
 *  Created on: 26/10/2023
 *      Author: Inácio Fonseca
 */
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// ADD TO YOUR C code above main function void main()
/*
extern void display_toLeds(unsigned int counter);
extern void Gpio_select(void);
extern void Gpio_select_PWM1_2_3();
extern float changeTIMER0FREQ(int hex);
extern int readHexEncoder();
extern int readButtonPB1();
extern int readButtonPB2();
*/

/*
 * ***************** CODE **************************
 */

void display_toLeds(unsigned int counter) {
    if(counter&1) GpioDataRegs.GPASET.bit.GPIO9 = 1;
         else GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
     if(counter&2) GpioDataRegs.GPASET.bit.GPIO11 = 1;
         else GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
     if(counter&4) GpioDataRegs.GPBSET.bit.GPIO34 = 1;
         else GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
     if(counter&8) GpioDataRegs.GPBSET.bit.GPIO49 = 1;
         else GpioDataRegs.GPBCLEAR.bit.GPIO49 = 1;
}
void Gpio_select_PWM1_2_3(){
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; // ePWM1A active
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1; // ePWM1B active
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; // ePWM2A active
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1; // ePWM2B active
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1; // ePWM3A active
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1; // ePWM3B active
    EDIS;
}

void Gpio_select(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX1.all = 0;        // GPIO15 ... GPIO0 = General Puropse I/O
    GpioCtrlRegs.GPAMUX2.all = 0;        // GPIO31 ... GPIO16 = General Purpose I/O
    GpioCtrlRegs.GPBMUX1.all = 0;        // GPIO47 ... GPIO32 = General Purpose I/O
    GpioCtrlRegs.GPBMUX2.all = 0;        // GPIO63 ... GPIO48 = General Purpose I/O
    GpioCtrlRegs.GPCMUX1.all = 0;        // GPIO79 ... GPIO64 = General Purpose I/O
    GpioCtrlRegs.GPCMUX2.all = 0;        // GPIO87 ... GPIO80 = General Purpose I/O

    // For qualification in the button to stabilize the logic level
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17=1;  // PB1 with 3 samples for qualification
    GpioCtrlRegs.GPBQSEL2.bit.GPIO48=1;  // PB2 with 3 samples for qualification

    GpioCtrlRegs.GPACTRL.bit.QUALPRD2=2; // 3 samples at 4*Sysclockout
    GpioCtrlRegs.GPBCTRL.bit.QUALPRD2=2; //  1st Sample --> 4*Sysclockout delay --> 2nd Sample --> 4*Sysclockout delay --> 3rd Sample

    GpioCtrlRegs.GPADIR.all = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;    // peripheral explorer: LED LD1 at GPIO9
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;    // peripheral explorer: LED LD2 at GPIO11

    GpioCtrlRegs.GPBDIR.all = 0;        // GPIO63-32 as inputs
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;    // peripheral explorer: LED LD3 at GPIO34
    GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1; // peripheral explorer: LED LD4 at GPIO49
    GpioCtrlRegs.GPCDIR.all = 0;        // GPIO87-64 as inputs
    EDIS;
    Gpio_select_PWM1_2_3();
    // GPIO27 --> Infrared
    // Hex Encoder = GPIO12 | 13 | 14 | GPIO15
}

// Made in 2022
float changeTIMER0FREQ(int hex) {
    float nextPeriod = 100000+(250000-100000)*hex/15;
    DINT;
    EALLOW;
    CpuTimer0Regs.TCR.bit.TSS = 1;  // stop timer0
    ConfigCpuTimer(&CpuTimer0,150,nextPeriod); // CPU - Timer0 at Hex milliseconds
    CpuTimer0Regs.TCR.bit.TSS = 0;  // start timer0
    EDIS;
    EINT;
    return nextPeriod;
}

// Last change in 2020...
int readHexEncoder(){
    int v=0;
    if (GpioDataRegs.GPADAT.bit.GPIO12==1) v+=1;
    if (GpioDataRegs.GPADAT.bit.GPIO13==1) v+=2;
    if (GpioDataRegs.GPADAT.bit.GPIO14==1) v+=4;
    if (GpioDataRegs.GPADAT.bit.GPIO15==1) v+=8;
    return v;
}

// Last change in 2020...
int readButtonPB1(){
    if (GpioDataRegs.GPADAT.bit.GPIO17==1)
        return 0; else return 1;
}

// Last change in 2020...
int readButtonPB2(){
    if (GpioDataRegs.GPBDAT.bit.GPIO48==1)
        return 0; else return 1;
}


