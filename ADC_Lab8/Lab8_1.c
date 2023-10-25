//
//      Lab8_1: TMS320F28335
//      (c) Frank Bormann
//      Adapted by Inácio Fonseca
//
//###########################################################################
//
// FILE:	Lab8_1.c
// 
// TITLE:	DSP28335ControlCARD Analogue Input
//			ADCIN_A0 , ADCIN_A1  connected to variable resistors VR1, VR2
//			on Peripheral Explorer Board (3.3V...0V)
//			ADC samples at 50KHz, hardware triggered by ePWM2
//          Voltages are displayed alternately on 4 LEDs LD1...LD4
//			(GPIO9, GPIO11, GPIO34 and GPIO49)
//			CPU Timer0 ISR every 100 ms
//			Watchdog active , cleared in ISR and main-loop 
//
//###########################################################################
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//	3.0 | 30 Jun 2009 | F.B. | Lab8_1 for F28335CC + peripheral explorer
//  3.1 | 09 Nov 2009 | F.B  | Lab8_1 for F28335 and PE revision 5
//###########################################################################
#include "DSP2833x_Device.h"

// external function prototypes
extern void InitAdc(void);
extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);
// ADC ...
void display_ADC(unsigned int);
void Setup_ADC();

// Prototype statements for functions found within this file.
void Gpio_select(void);
interrupt void cpu_timer0_isr(void);
interrupt void adc_isr(void);		 // ADC  End of Sequence ISR

// Global Variables
unsigned int Voltage_VR1;
unsigned int Voltage_VR2;

//###########################################################################
//						main code									
//###########################################################################
void main(void)
{
	InitSysCtrl();	// Basic Core Init from DSP2833x_SysCtrl.c

	EALLOW;
   	SysCtrlRegs.WDCR= 0x00AF;	// Re-enable the watchdog 
   	EDIS;			// 0x00AF  to NOT disable the Watchdog, Prescaler = 64

	DINT;				// Disable all interrupts
	
	Gpio_select();		// GPIO9, GPIO11, GPIO34 and GPIO49 as output
					    // to 4 LEDs at Peripheral Explorer)

	InitPieCtrl();		// basic setup of PIE table; from DSP2833x_PieCtrl.c
	
	InitPieVectTable();	// default ISR's in PIE

	InitAdc();			// Basic ADC setup, incl. calibration
	Setup_ADC();         // Set OUR SETUP FOR ADC, see below

	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	PieVectTable.ADCINT = &adc_isr;
	EDIS;

	InitCpuTimers();	// basic setup CPU Timer0, 1 and 2

	ConfigCpuTimer(&CpuTimer0,150,100000);

	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;		// CPU Timer 0
	PieCtrlRegs.PIEIER1.bit.INTx6 = 1;		// ADC

	IER |=1;

	EINT;
	ERTM;

	CpuTimer0Regs.TCR.bit.TSS = 0;	// start timer0

	while(1)
	{    
	  	while(CpuTimer0.InterruptCount <5)
	  	{
	  		// wait for 500 ms
			EALLOW;
			SysCtrlRegs.WDKEY = 0x55;		// Service watchdog #1
			EDIS;
		}
		// Display VR1 voltage
		display_ADC(Voltage_VR1);	
		
		while(CpuTimer0.InterruptCount <10)	// wait for 1000 ms		
	  	{
	  		EALLOW;
			SysCtrlRegs.WDKEY = 0x55;		// Service watchdog #1
			EDIS;
		}
		// Display VR2 voltage
	    display_ADC(Voltage_VR2);

		CpuTimer0.InterruptCount = 0;
	}
} 

void display_ADC(unsigned int counter) {
    if(counter&1) GpioDataRegs.GPASET.bit.GPIO9 = 1;
         else GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
     if(counter&2) GpioDataRegs.GPASET.bit.GPIO11 = 1;
         else GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
     if(counter&4) GpioDataRegs.GPBSET.bit.GPIO34 = 1;
         else GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
     if(counter&8) GpioDataRegs.GPBSET.bit.GPIO49 = 1;
         else GpioDataRegs.GPBCLEAR.bit.GPIO49 = 1;
}
void Setup_ADC() {
    AdcRegs.ADCTRL1.all = 0;
    AdcRegs.ADCTRL1.bit.ACQ_PS = 7;     // 7 = 8 x ADCCLK
    AdcRegs.ADCTRL1.bit.SEQ_CASC =1;    // 1=cascaded sequencer
    AdcRegs.ADCTRL1.bit.CPS = 0;        // divide by 1
    AdcRegs.ADCTRL1.bit.CONT_RUN = 0;   // single run mode

    AdcRegs.ADCTRL2.all = 0;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;   // 1=enable SEQ1 interrupt
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 =1;  // 1=SEQ1 start from ePWM_SOCA trigger
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;   // 0= interrupt after every end of sequence

    AdcRegs.ADCTRL3.bit.ADCCLKPS = 3;   // ADC clock: FCLK = HSPCLK / 2 * ADCCLKPS
                                        // HSPCLK = 75MHz (see DSP2833x_SysCtrl.c)
                                        // FCLK = 12.5 MHz

    AdcRegs.ADCMAXCONV.all = 0x0001;    // 2 conversions from Sequencer 1

    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0; // Setup ADCINA0 as 1st SEQ1 conv.
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 1; // Setup ADCINA1 as 2nd SEQ1 conv.

    EPwm2Regs.TBCTL.all = 0xC030;   // Configure timer control register
    /*
     bit 15-14     11:     FREE/SOFT, 11 = ignore emulation suspend
     bit 13        0:      PHSDIR, 0 = count down after sync event
     bit 12-10     000:    CLKDIV, 000 => TBCLK = HSPCLK/1
     bit 9-7       000:    HSPCLKDIV, 000 => HSPCLK = SYSCLKOUT/1
     bit 6         0:      SWFSYNC, 0 = no software sync produced
     bit 5-4       11:     SYNCOSEL, 11 = sync-out disabled
     bit 3         0:      PRDLD, 0 = reload PRD on counter=0
     bit 2         0:      PHSEN, 0 = phase control disabled
     bit 1-0       00:     CTRMODE, 00 = count up mode
    */

    EPwm2Regs.TBPRD = 2999; // TPPRD +1  =  TPWM / (HSPCLKDIV * CLKDIV * TSYSCLK)
                            //           =  20 �s / 6.667 ns

    EPwm2Regs.ETPS.all = 0x0100;            // Configure ADC start by ePWM2
    /*
     bit 15-14     00:     EPWMxSOCB, read-only
     bit 13-12     00:     SOCBPRD, don't care
     bit 11-10     00:     EPWMxSOCA, read-only
     bit 9-8       01:     SOCAPRD, 01 = generate SOCA on first event
     bit 7-4       0000:   reserved
     bit 3-2       00:     INTCNT, don't care
     bit 1-0       00:     INTPRD, don't care
    */

    EPwm2Regs.ETSEL.all = 0x0A00;           // Enable SOCA to ADC
    /*
     bit 15        0:      SOCBEN, 0 = disable SOCB
     bit 14-12     000:    SOCBSEL, don't care
     bit 11        1:      SOCAEN, 1 = enable SOCA
     bit 10-8      010:    SOCASEL, 010 = SOCA on PRD event
     bit 7-4       0000:   reserved
     bit 3         0:      INTEN, 0 = disable interrupt
     bit 2-0       000:    INTSEL, don't care
    */
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

interrupt void cpu_timer0_isr(void)
{
	CpuTimer0.InterruptCount++;
	EALLOW;
	SysCtrlRegs.WDKEY = 0xAA;	// service WD #2
	EDIS;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void  adc_isr(void)
{
	Voltage_VR1 = AdcMirror.ADCRESULT0;	// store results global
  	Voltage_VR2 = AdcMirror.ADCRESULT1;
	// Reinitialize for next ADC sequence
  	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;       // Reset SEQ1
  	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;		// Clear INT SEQ1 bit
  	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge interrupt to PIE
}

//===========================================================================
// End of SourceCode.
//===========================================================================
