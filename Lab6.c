#include "DSP2833x_Device.h"
/*
 * Example Timer0 by interrupt
 * Author: Inácio Fonseca
 * Adapted from TI code examples
 */


// external function prototypes
extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);

// Prototype statements for functions found within this file.
void Gpio_select(void);
interrupt void cpu_timer0_isr(void);
//###########################################################################
//						main code									
//###########################################################################
void main(void)
 {
	int counter=0;	// binary counter for digital output

	InitSysCtrl();	// Basic Core Init from DSP2833x_SysCtrl.c

	EALLOW;
   	SysCtrlRegs.WDCR= 0x00AF;	// Re-enable the watchdog 
   	EDIS;			// 0x00AF  to NOT disable the Watchdog, Prescaler = 64

	DINT;				// Disable all interrupts
	Gpio_select();		// GPIO9, GPIO11, GPIO34 and GPIO49 as output
					    // to 4 LEDs at Peripheral Explorer)
	InitPieCtrl();		// basic setup of PIE table; from DSP2833x_PieCtrl.c
	InitPieVectTable();	// default ISR's in PIE

	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	EDIS;

	InitCpuTimers();	// basic setup CPU Timer0, 1 and 2

	ConfigCpuTimer(&CpuTimer0,150,100000); // CPU - Timer0 at 100 milliseconds

	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

	IER |=1;

	EINT;
	ERTM;

	CpuTimer0Regs.TCR.bit.TSS = 0;	// start timer0

	while(1)
	{    
	  		while(CpuTimer0.InterruptCount == 0);
			CpuTimer0.InterruptCount = 0;
			
			EALLOW;
			SysCtrlRegs.WDKEY = 0x55;	// service WD #1
			EDIS;

	  		counter++;
			if(counter&1) GpioDataRegs.GPASET.bit.GPIO9 = 1;
				else GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
			if(counter&2) GpioDataRegs.GPASET.bit.GPIO11 = 1;
				else GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
			if(counter&4) GpioDataRegs.GPBSET.bit.GPIO34 = 1;
				else GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
			if(counter&8) GpioDataRegs.GPBSET.bit.GPIO49 = 1;
				else GpioDataRegs.GPBCLEAR.bit.GPIO49 = 1;
	}
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
	// GPIO27 --> Infrared
	// Hex Encoder = GPIO12 | 13 | 14 | GPIO15
}   

interrupt void cpu_timer0_isr(void)
{
	CpuTimer0.InterruptCount++;
	EALLOW;
	SysCtrlRegs.WDKEY = 0xAA;	// service WD #2
	EDIS;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
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
// Button0 GPIO17


// Done in 2020...
int readHexEncoder(){
    int v=0;
    if (GpioDataRegs.GPADAT.bit.GPIO12==1) v+=1;
    if (GpioDataRegs.GPADAT.bit.GPIO13==1) v+=2;
    if (GpioDataRegs.GPADAT.bit.GPIO14==1) v+=4;
    if (GpioDataRegs.GPADAT.bit.GPIO15==1) v+=8;
    return v;
}
// Done in 2020...
int readButtonPB1(){
    if (GpioDataRegs.GPADAT.bit.GPIO17==1)
        return 0; else return 1;
}
// Done in 2020...
int readButtonPB2(){
    if (GpioDataRegs.GPBDAT.bit.GPIO48==1)
        return 0; else return 1;
}
// Done in 2019...
void delay_loop()
{
    volatile long i;
    for (i = 0; i < 10000000; i++);
}
//===========================================================================
// End of SourceCode.
//===========================================================================
