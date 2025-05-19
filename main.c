/*
Software for a current measurement module I created for my bachelor's thesis
--------------------------------------------------------------------------------
PIN CONNECTIONS
--------------------------------------------------------------------------------
RUN LED   ->     P1.5
ADC INPUT ->     P1.3 A3
VEREF+    ->     P1.0 A0
VEREF-    ->     P1.2 A2
RELAY CONTROL -> P1.7

XTAL+ -> P2.0
XTAL- -> P2.1

UART RX -> P2.5
UART TX -> P2.6
*/
//INCLUDE HEADERS
#include <msp430.h>
#include <stdint.h>
#include <stdio.h>

//DEFINITIONS
#define MCLK_FREQ_MHZ 1
#define WINDOW_SIZE 1000
#define IDLE_POINT 515 //using VREF/2=1.5V midpoint the ADC value =

volatile int  adcResult;
volatile int  maxInWindow;
volatile int  sampleCount;

//STUFF
void Software_Trim(void);
int fputc(int c, FILE *stream);

//INITIAL FUNCTIONS DEFINITIONS
void init(void);
void initEXTCLK(void);
void initGPIO(void);
void initADC(void);
void initUART(void);

//--------------------------------------------------------------------------------
//MAIN
//--------------------------------------------------------------------------------
int main(void){

    init();  //Calls init function that calls GPIO, ADC, EXTCLK and UART init functions
    __enable_interrupt();

    while(1){

        // reset window
        maxInWindow  = 0;
        sampleCount  = 0;

        // collect WINDOW_SIZE samples
        /* Window based sampling
        while(sampleCount < WINDOW_SIZE){
            ADCCTL0 |= ADCENC | ADCSC;             // start conversion
            __bis_SR_register(LPM0_bits);    // sleep until ISR wakes us
        }*/
        //maxInWindow is peak value printed to UART -> RS-485
        //printf("Peak=%d\r\n", maxInWindow);
        //regular sampling
        ADCCTL0 |= ADCENC | ADCSC;             // start conversion
         __bis_SR_register(LPM0_bits);
        
        //printf("Peak=%d\r\n", adcResult);
        //rtdxChannelWrite("Peak=%d\r\n", adcResult);

        __delay_cycles(500);
    }
}

//--------------------------------------------------------------------------------
//INITIALIZATION FUNCTIONS
//--------------------------------------------------------------------------------
//PRIMARY INIT
void init(void) {

    WDTCTL = WDTPW | WDTHOLD;    //kill WDT

    // Disable GPIO High-Z protection
    PM5CTL0 &= ~LOCKLPM5;

    //initEXTCLK();
    initGPIO();
    initADC();
    initUART();

    
}
//INIT EXTERNAL CLOCK NESTRĀDĀ
void initEXTCLK(void){

    CSCTL0_H = 0xA5;

    // XT1 crystal on P2.0/P2.1
    P2SEL0 &= ~(BIT0|BIT1);
    P2SEL1 |=  (BIT0|BIT1);
    // Clear fault flags
    do {
        CSCTL7 &= ~(XT1OFFG | DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);

    // Disable FLL
    __bis_SR_register(SCG0);
    CSCTL3 = SELREF__XT1CLK;                // XT1 as FLL reference
    CSCTL1 = DCOFTRIMEN | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_0;
    CSCTL2 = FLLD_0 + 30;                   // DCODIV = 1MHz
    __delay_cycles(3);
    // Enable FLL
    __bic_SR_register(SCG0);

    Software_Trim();                        // Trim DCO

    // Route clocks: MCLK/SMCLK = DCOCLKDIV, ACLK = XT1
    CSCTL4 = SELMS__DCOCLKDIV | SELA__XT1CLK;

    CSCTL0_H = 0;

    // Enable oscillator fault interrupt
    SFRIE1 |= OFIE;
}


//INIT GPIO PINS
void initGPIO(void){

    //Configure GPIO
    P1DIR |= BIT5;              //Change P1.5(RUN LED) direction to output
    P1OUT |= BIT5;              //Set P1.5 HIGH
    P1DIR |= BIT7;              //Change P1.7(RUN LED) direction to output
    P1OUT &= ~BIT7;             //Set P1.7 LOW

    // Configure GPIO to ADC A0(VEREF+) A2(VEREF-) A3(INPUT) pins
    SYSCFG2 |= ADCPCTL0 | ADCPCTL2 | ADCPCTL3;
                                   
    

}
//INIT ADC
void initADC(void){

    // Configure ADC
    ADCCTL0 |= ADCSHT_2 | ADCON;        // ADCON, S&H=16 ADC clks
    ADCCTL1 |= ADCSHP;                  // ADCCLK = MODOSC; sampling timer
    ADCCTL2 |= ADCRES;                  // 10-bit conversion results
    ADCIE |= ADCIE0;                    // Enable ADC conv complete interrupt
    ADCMCTL0 |= ADCINCH_3 | ADCSREF_2;  // A3 ADC input select; Vref=VEREF+ and VEREF-

}

//INIT UART
void initUART(void)
{
    P2SEL0 &= ~(BIT5 | BIT6);
    P2SEL1 |=  (BIT5 | BIT6);
    UCA1CTLW0 = UCSSEL__SMCLK;
    UCA1BRW   = 104;
    UCA1MCTLW = UCOS16 | UCBRS1;
    UCA1CTLW0 &= ~UCSWRST;
}

//--------------------------------------------------------------------------------
//ADC INTERRUPT SERVICE ROUTINE
//--------------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC_VECTOR))) ADC_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
    {
        case ADCIV_NONE:
            break;                              
        case ADCIV_ADCOVIFG: 
            break;                              
        case ADCIV_ADCTOVIFG:
            break;                              
        case ADCIV_ADCHIIFG:
            break;                         
        case ADCIV_ADCLOIFG:
            break;                             
        case ADCIV_ADCINIFG:
            break;                              
        case ADCIV_ADCIFG:
            adcResult = ADCMEM0;
        /*if (adcResult > maxInWindow) {
            maxInWindow = abs(adcResult-IDLE_POINT);
        }*/
        maxInWindow = adcResult;
        sampleCount++;
        __bic_SR_register_on_exit(LPM0_bits);  // wake main()            // Clear CPUOFF bit from LPM0
            break;             
        default:
            break; 
    }  
}

//--------------------------------------------------------------------------------
//CLOCK FAULT INTERRUPT ROUTINE
//--------------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=UNMI_VECTOR
__interrupt void UNMI_ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(UNMI_VECTOR))) UNMI_ISR(void)
#else
#error Compiler not supported!
#endif
{
    do {
        CSCTL7 &= ~XT1OFFG;
        SFRIFG1 &= ~OFIFG;
        P1OUT |= BIT0;
        __delay_cycles(25000);
    } while (SFRIFG1 & OFIFG);
    P1OUT &= ~BIT0;
}

//------------------------------------------------------------------------------
// DCOFTRIM ADJUSTMENT
//------------------------------------------------------------------------------
void Software_Trim(void)
{
    unsigned int oldDcoTap = 0xffff;
    unsigned int newDcoTap;
    unsigned int newDcoDelta;
    unsigned int bestDcoDelta = 0xffff;
    unsigned int csCtl0Copy, csCtl1Copy;
    unsigned int csCtl0Read, csCtl1Read;
    unsigned int dcoFreqTrim;
    unsigned char endLoop = 0;

    do {
        CSCTL0 = 0x100;                     // DCOTAP=256
        do { CSCTL7 &= ~DCOFFG; } while (CSCTL7 & DCOFFG);
        __delay_cycles(3000 * MCLK_FREQ_MHZ);
        while ((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && !(CSCTL7 & DCOFFG));

        csCtl0Read = CSCTL0;
        csCtl1Read = CSCTL1;
        newDcoTap   = csCtl0Read & 0x01ff;
        dcoFreqTrim = (csCtl1Read & 0x0070) >> 4;

        if (newDcoTap < 256) {
            newDcoDelta = 256 - newDcoTap;
            if (oldDcoTap != 0xffff && oldDcoTap >= 256) endLoop = 1;
            else { dcoFreqTrim--; CSCTL1 = (csCtl1Read & ~0x0070) | (dcoFreqTrim << 4); }
        } else {
            newDcoDelta = newDcoTap - 256;
            if (oldDcoTap < 256) endLoop = 1;
            else { dcoFreqTrim++; CSCTL1 = (csCtl1Read & ~0x0070) | (dcoFreqTrim << 4); }
        }

        if (newDcoDelta < bestDcoDelta) {
            csCtl0Copy    = csCtl0Read;
            csCtl1Copy    = csCtl1Read;
            bestDcoDelta  = newDcoDelta;
        }
        oldDcoTap = newDcoTap;
    } while (!endLoop);

    CSCTL0 = csCtl0Copy;
    CSCTL1 = csCtl1Copy;
    while (CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));
}
