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

#define TX_BUF_SIZE  64

static volatile uint8_t  txBuf[TX_BUF_SIZE];
static volatile uint8_t  txHead = 0;    // next free slot
static volatile uint8_t  txTail = 0;    // next byte to send
static volatile uint8_t  txCount = 0;   // number of bytes in buffer



volatile int  adcResult;
volatile int  maxInWindow;
volatile int  sampleCount;

// ——————————————————————————————
// fputc() override to enqueue on A1
// ——————————————————————————————
int fputc(int ch, FILE *f) {
    while(txCount == TX_BUF_SIZE);

    _disable_interrupt();

    if (txCount == 0) {
        UCA1TXBUF = (uint8_t)ch;
        // enable the interrupt so that when this byte finishes
        // shifting out, the ISR will take over on the next one
        UCA1IE |= UCTXIE;
    } else {
        // just enqueue it
        txBuf[txHead++] = (uint8_t)ch;
        if (txHead == TX_BUF_SIZE) txHead = 0;
        txCount++;
    }
    __enable_interrupt();

    return ch;

}



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

        //reset window
        maxInWindow  = 0;
        sampleCount  = 0;

        
        ADCCTL0 |= ADCENC | ADCSC;             // start conversion
        __bis_SR_register(LPM0_bits);
        __no_operation();

        printf("Peak=%d\r\n", adcResult);
        //__no_operation();
        __delay_cycles(10000);    // ~1s @ 1 MHz
        
    }
}

//--------------------------------------------------------------------------------
//INITIALIZATION FUNCTIONS
//--------------------------------------------------------------------------------
//PRIMARY INIT
void init(void) {

    WDTCTL = WDTPW | WDTHOLD;    //kill WDT


    PM5CTL0 &= ~LOCKLPM5;


    initGPIO();
    initADC();
    initUART();
    initEXTCLK();
    

    
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
        _delay_cycles(100);
    } while (SFRIFG1 & OFIFG);

    // Clear fault flags 
    CSCTL7 &= ~(XT1OFFG | DCOFFG);
    SFRIFG1 &= ~OFIFG;
    _delay_cycles(200);
    //Clear fault flags and disable fault flags
    SFRIE1 &= ~OFIE;
    SFRIFG1 &= ~OFIFG;

    // Disable FLL
    __bis_SR_register(SCG0);
    CSCTL3 = SELREF__XT1CLK;                // XT1 as FLL reference
    CSCTL1 = DCOFTRIMEN | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_0;
    CSCTL2 = FLLD_0 + 32;                   // DCODIV = 1MHz
    __delay_cycles(3);
    // Enable FLL
    __bic_SR_register(SCG0);

    // Route clocks: MCLK/SMCLK = DCOCLKDIV, ACLK = XT1
    CSCTL4 = SELMS__DCOCLKDIV | SELA__XT1CLK;

    CSCTL0_H = 0;

    
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
    //– Route P2.5→UCA1RXD, P2.6→UCA1TXD
    P2SEL1 &= ~(BIT5 | BIT6);
    P2SEL0 |=  (BIT5 | BIT6);

    //– Put eUSCI in reset, select SMCLK
    UCA1CTLW0 |= UCSWRST;            // <<-- hold in reset
    UCA1CTLW0 |= UCSSEL__SMCLK;    // clock = SMCLK

    //– Baud-rate 9600 @ 1 MHz
    UCA1BRW    = 104;               // integer divider
    UCA1MCTLW  = UCOS16 | UCBRS1;  // oversampling + modulation

    //– Take eUSCI out of reset
    UCA1CTLW0 &= ~UCSWRST;

    // Ensure TX interrupt is _off_ until first byte is written
    //UCA1IE &= ~UCTXIE;
}

//--------------------------------------------------------------------------------
//ADC INTERRUPT SERVICE ROUTINE
//--------------------------------------------------------------------------------
void __attribute__((interrupt(ADC_VECTOR))) ADC_ISR(void)
{
  if (ADCIV == ADCIV_ADCIFG) {
    adcResult   = ADCMEM0;
    maxInWindow = adcResult;
    sampleCount++;
    __bic_SR_register_on_exit(LPM0_bits);
  }
}
