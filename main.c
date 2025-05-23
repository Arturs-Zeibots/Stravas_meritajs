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
#include <stdbool.h>

//DEFINITIONS
#define WINDOW_SIZE 1000
#define IDLE_POINT 0 //using VREF/2=1.5V midpoint the ADC value = 
#define TX_BUF_SIZE  64

static volatile uint8_t  txBuf[TX_BUF_SIZE];
static volatile uint8_t  txHead = 0;    // next free slot
static volatile uint8_t  txTail = 0;    // next byte to send
static volatile uint8_t  txCount = 0;   // number of bytes in buffer

volatile int  adcResult;
volatile int  maxInWindow;
volatile int  sampleCount;
volatile int printValue;

volatile bool requestADC = false;

//INITIAL FUNCTIONS DEFINITIONS
void init(void);
void initEXTCLK(void);
void initGPIO(void);
void initADC(void);
void initUART(void);

//ADC read and wait function
int readADC(void){
    int maxDiff = 0;
    int i;
    for (i = 0; i < WINDOW_SIZE; i++) {
        ADCCTL0 |= ADCENC | ADCSC;              // enable & start
        while (!(ADCCTL0 & ADCIFG));            // wait end
        ADCCTL0 &= ~ADCIFG;
        int v = ADCMEM0;
        int d = (v > IDLE_POINT) ? (v - IDLE_POINT)
                                 : (IDLE_POINT - v);
        if (d > maxDiff) maxDiff = d;
    }
    return maxDiff;
}

//--------------------------------------------------------------------------------
//MAIN
//--------------------------------------------------------------------------------
int main(void){

    init();  //Calls init function that calls GPIO, ADC, EXTCLK and UART init functions
    __enable_interrupt();
    

    while(1){
        //wait for UART RX interrupt to call ADC conversion
	if (requestADC) {
            requestADC = false;

            //do polled ADC
            int result = readADC();

            //format into txBuf
            int len = sprintf((char*)txBuf, "I=%d\r\n", result);
            txHead  = 0;
            txTail  = 0;
            txCount = len;

            //start TX interrupt routine
            UCA1IE |= UCTXIE;
        }
    }
}

//--------------------------------------------------------------------------------
//INITIALIZATION FUNCTIONS
//--------------------------------------------------------------------------------
//PRIMARY INIT contain WDT shutoff, calls GPIO, EXTCLK, UART, ADC init functions
void init(void) {

    WDTCTL = WDTPW | WDTHOLD;    //kill WDT


    PM5CTL0 &= ~LOCKLPM5;


    initGPIO();
    initADC();
    initEXTCLK();
    initUART();
}

//INIT EXTERNAL CLOCK
void initEXTCLK(void){

    // XT1 crystal on P2.0/P2.1
    P2SEL0 |= (BIT0|BIT1);
    P2SEL1 &=  ~(BIT0|BIT1);

    CSCTL7 &= ~XT1OFFG;           // clear XT1 fault flag
    CSCTL7 &= ~(XT1OFFG|DCOFFG);  // clear both fault flags
    // choose drive strength
    CSCTL6 &= ~(XT1BYPASS | XTS);
    CSCTL6 |= XT1DRIVE_3;
    // Clear fault flags until XT1 settles
    do {
    CSCTL7 &= ~(XT1OFFG|DCOFFG);
    SFRIFG1 &= ~OFIFG;
    __delay_cycles(1000);
    } while ((SFRIFG1 & OFIFG));

    // Clear and disable fault flags 
    CSCTL7 &= ~(XT1OFFG | DCOFFG);
    SFRIFG1 &= ~OFIFG;
    SFRIE1 &= ~OFIE;
    _delay_cycles(200);

    //setup FLL
    CSCTL3 = SELREF__XT1CLK;                // XT1 as FLL reference
    CSCTL1 = DCOFTRIMEN | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_0;
    CSCTL2 = FLLD_0 + 31;                   // DCODIV = 1MHz
    __delay_cycles(3);

    // Route clocks: MCLK/SMCLK = DCOCLKDIV, ACLK = XT1
    CSCTL4 = SELMS__DCOCLKDIV | SELA__XT1CLK;

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
void initUART(void){

    //– Route P2.5→UCA1RXD, P2.6→UCA1TXD
    P2SEL1 &= ~(BIT5 | BIT6);
    P2SEL0 |=  (BIT5 | BIT6);

    //– Put eUSCI in reset, select SMCLK
    UCA1CTLW0 |= UCSWRST;            // <<-- hold in reset
    UCA1CTLW0 |= UCSSEL__SMCLK;    // clock = SMCLK

    //– Baud-rate 9600 @ 1 MHz
    UCA1BRW = 6;                              // 1000000/9600 = 104.167
    UCA1MCTLW |= UCOS16 | UCBRF_13 | 0x11;     // UCBRSx value = 0x11

    UCA1IE &= ~UCTXIE;
 
    //– Take eUSCI out of reset
    UCA1CTLW0 &= ~UCSWRST;

    //enable RX interrupt
    UCA1IE = UCRXIE;

}


//--------------------------------------------------------------------------------
//ADC INTERRUPT SERVICE ROUTINE
//--------------------------------------------------------------------------------
void __attribute__((interrupt(ADC_VECTOR))) ADC_ISR(void){
  
  if (ADCIV == ADCIV_ADCIFG) {
            adcResult = ADCMEM0;
        if (adcResult > maxInWindow) {
            maxInWindow = abs(adcResult-IDLE_POINT);
        }
        sampleCount++;
  }
}

//--------------------------------------------------------------------------------
//UART INTERRUPT SERVICE ROUTINE
//-------------------------------------------------------------------------------
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void){
    if (UCA1IFG & UCRXIFG) {
        char c = UCA1RXBUF;
        // echo
        while (!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = c;
        while (!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = '\r';
        while (!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = '\n';
        // flag ADC request on 'u'
        if (c == 'u') {
            requestADC = true;
        }
        // clear RX flag
        UCA1IFG &= ~UCRXIFG;
    }
    // TX interrupt feed txBuf into UART TX buffer
    if (UCA1IFG & UCTXIFG) {
        if (txCount) {
            UCA1TXBUF = txBuf[txTail++];
            if (txTail == TX_BUF_SIZE) txTail = 0;
            txCount--;
        } else {
            UCA1IE &= ~UCTXIE;
        }
    }
}