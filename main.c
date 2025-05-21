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

#define XT1_TIMEOUT  50000u
uint16_t timeout = XT1_TIMEOUT;


static volatile uint8_t  txBuf[TX_BUF_SIZE];
static volatile uint8_t  txHead = 0;    // next free slot
static volatile uint8_t  txTail = 0;    // next byte to send
static volatile uint8_t  txCount = 0;   // number of bytes in buffer



volatile int  adcResult;
volatile int  maxInWindow;
volatile int  sampleCount;

//INITIAL FUNCTIONS DEFINITIONS
void init(void);
void initEXTCLK(void);
void initGPIO(void);
void initADC(void);
void initUART(void);

void readADC(void);

int fputc(int ch, FILE *f) {
    while (!(UCA1IFG & UCTXIFG));   // wait for TX ready
    UCA1TXBUF = (uint8_t)ch;
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

//--------------------------------------------------------------------------------
//MAIN
//--------------------------------------------------------------------------------
int main(void){

    init();  //Calls init function that calls GPIO, ADC, EXTCLK and UART init functions
    __enable_interrupt();

    ADCCTL0 |= ADCENC; 
    UCA1RXBUF = 0x0075;
    UCA1IFG |= UCRXIFG;
    
    while(1){
        __no_operation();
        
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
    initEXTCLK();
    initUART();
}
//INIT EXTERNAL CLOCK NESTRĀDĀ
void initEXTCLK(void){

    //CSCTL0_H = 0xA5;

    // XT1 crystal on P2.0/P2.1
    P2SEL0 |= (BIT0|BIT1);
    P2SEL1 &=  ~(BIT0|BIT1);

    CSCTL7 &= ~XT1OFFG;           // clear XT1 fault flag
    CSCTL7 &= ~(XT1OFFG|DCOFFG);  // clear both fault flags
    // choose drive strength: lowest first, increase if it doesn’t start
    CSCTL6 &= ~(XT1BYPASS | XTS);
    CSCTL6 |= XT1DRIVE_3;
    // Clear fault flags
    do {
    CSCTL7 &= ~(XT1OFFG|DCOFFG);
    SFRIFG1 &= ~OFIFG;
    __delay_cycles(1000);
    } while ((SFRIFG1 & OFIFG));


    // Clear fault flags 
    CSCTL7 &= ~(XT1OFFG | DCOFFG);
    SFRIFG1 &= ~OFIFG;
    _delay_cycles(200);
    //Clear fault flags and disable fault flags
    SFRIE1 &= ~OFIE;
    SFRIFG1 &= ~OFIFG;

    CSCTL3 = SELREF__XT1CLK;                // XT1 as FLL reference
    CSCTL1 = DCOFTRIMEN | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_0;
    CSCTL2 = FLLD_0 + 31;                   // DCODIV = 1MHz
    __delay_cycles(3);

    // Route clocks: MCLK/SMCLK = DCOCLKDIV, ACLK = XT1
    CSCTL4 = SELMS__DCOCLKDIV | SELA__XT1CLK;

    //CSCTL0_H = 0;

    
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
    UCA1MCTLW |= UCOS16 | UCBRF_13 | 0x11;     // UCBRSx value = 0x11 (See UG)

    UCA1IE &= ~UCTXIE;
 
    //– Take eUSCI out of reset
    UCA1CTLW0 &= ~UCSWRST;


      UCA1IE   = UCRXIE;
    //UCA1IE = UCRXIE;
    //UCA1IE = UCTXIE;

}

void readADC(){
    maxInWindow  = 0;
    sampleCount  = 0;

    __enable_interrupt();

    while (sampleCount < WINDOW_SIZE) {
        ADCCTL0 |= ADCSC;    // start
        // no waiting here; ADC_ISR will bump sampleCount
    }

    // And then disable nested if you like:
    __disable_interrupt();
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
__interrupt void USCI_A1_ISR(void)
{
    

    if(UCA1IFG & UCRXIFG){
        //char rx = UCA1RXBUF;  
        //if (rx == 'u') {
            // ——— BEGIN ring-buffer fill ———
            readADC();
            volatile int result = maxInWindow;//readADC();
            int len = sprintf((char*)txBuf, "I=%d\r\n", result);

            //__disable_interrupt();
            // reset the TX buffer pointers/count
            txHead  = 0;
            txTail  = 0;
            txCount = len;
            // copy the message into txBuf[]
           /* volatile uint8_t i;
            for (i = 0; i < len; i++) {
                txBuf[txHead++] = msg[i];
                if (txHead == TX_BUF_SIZE) txHead = 0;
                txCount++;
            }*/
            // kick off the first byte
            UCA1IFG &= ~UCRXIFG;
            UCA1IE |= UCTXIE;  // enable TX interrupt
            //__enable_interrupt();
            // ——— END “Option 2” ———
        //
    }

    if (UCA1IFG & UCTXIFG) {
        if (txCount) {
            // send next byte from the ring buffer
            UCA1TXBUF = txBuf[txTail++];
            if (txTail == TX_BUF_SIZE) txTail = 0;
            txCount--;
        } else {
            // nothing left: turn off TX interrupts
            UCA1IE &= ~UCTXIE;
        }
    }
}

