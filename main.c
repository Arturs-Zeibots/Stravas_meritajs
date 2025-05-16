/*
Software for a current measurement module I created for my bachelor's thesis
----------------------------------------------------------------------------
PIN CONNECTIONS
----------------------------------------------------------------------------
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

#include <msp430.h>
#include <stdint.h>
#include <stdio.h>

#define WINDOW_SIZE 1000
#define IDLE_POINT 515 //using VREF/2=1.5V midpoint the ADC value =

volatile int  adcResult;
volatile int  maxInWindow;
volatile int  sampleCount;



void init(void);
void initGPIO(void);
void initADC(void);

int main(void){
    init();
    __enable_interrupt();
    while(1){

        // reset window
        maxInWindow  = 0;
        sampleCount  = 0;

        // collect WINDOW_SIZE samples
        while(sampleCount < WINDOW_SIZE){
            ADCCTL0 |= ADCENC | ADCSC;             // start conversion
            __bis_SR_register(LPM0_bits);    // sleep until ISR wakes us
        }

        //maxInWindow is max value attained in the window
        if(maxInWindow > 10){
            P1OUT &= ~BIT5;
               // turn on RELAY control as an example
        } else {
            P1OUT |= BIT5;
        }

        __delay_cycles(500000);
    }
}

void init(void) {

    WDTCTL = WDTPW | WDTHOLD;    //kill WDT

    initGPIO();
    initADC();
    
}

void initGPIO(void){

    //Configure GPIO
    P1DIR |= BIT5;              //Change P1.5(RUN LED) direction to output
    P1OUT |= BIT5;              //Set P1.5 HIGH
    P1DIR |= BIT7;              //Change P1.7(RUN LED) direction to output
    P1OUT &= ~BIT7;             //Set P1.7 LOW

    // Configure GPIO to ADC A0(VEREF+) A2(VEREF-) A3(INPUT) pins
    SYSCFG2 |= ADCPCTL0 | ADCPCTL2 | ADCPCTL3;
                                   
    // Disable GPIO High-Z protection
    PM5CTL0 &= ~LOCKLPM5;

}

void initADC(void){

    // Configure ADC
    ADCCTL0 |= ADCSHT_2 | ADCON;        // ADCON, S&H=16 ADC clks
    ADCCTL1 |= ADCSHP;                  // ADCCLK = MODOSC; sampling timer
    ADCCTL2 |= ADCRES;                  // 10-bit conversion results
    ADCIE |= ADCIE0;                    // Enable ADC conv complete interrupt
    ADCMCTL0 |= ADCINCH_3 | ADCSREF_2;  // A3 ADC input select; Vref=VEREF+ and VEREF-

}


// ADC interrupt service routine
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
        if (adcResult > maxInWindow) {
            maxInWindow = abs(adcResult-IDLE_POINT);
        }
        sampleCount++;
        __bic_SR_register_on_exit(LPM0_bits);  // wake main()            // Clear CPUOFF bit from LPM0
            break;             
        default:
            break; 
    }  
}


