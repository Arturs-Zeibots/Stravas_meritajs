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

int adcResult;

void init(void);
int adcSample(void);

int main(void){
    init();

    while(1){

        ADCCTL0 |= ADCENC | ADCSC;
        __bis_SR_register(LPM0_bits | GIE); 
        P1OUT ^= BIT5;
        __delay_cycles(500000);
        printf(adcResult);
    }
}

void init(void) {

    WDTCTL = WDTPW | WDTHOLD;    //kill WDT

    P1DIR |= BIT5;              //Change P1.5(RUN LED) direction to output
    P1OUT |= BIT5;              //Set P1.5 HIGH

    // Configure ADC A3(INPUT) pin
    SYSCFG2 |= ADCPCTL3;
                                   
    // Disable GPIO High-Z protection
    PM5CTL0 &= ~LOCKLPM5;

    // Configure ADC
    ADCCTL0 |= ADCSHT_2 | ADCON;        // ADCON, S&H=16 ADC clks
    ADCCTL1 |= ADCSHP;                  // ADCCLK = MODOSC; sampling timer
    ADCCTL2 |= ADCRES;                  // 10-bit conversion results
    ADCIE |= ADCIE0;                    // Enable ADC conv complete interrupt
    ADCMCTL0 |= ADCINCH_3 | ADCSREF_7;  // A3 ADC input select; Vref=VEREF+ and VEREF-
}

/*int adcSample(void){
    int adcSampleValue = 0;
     

    return adcSampleValue;

}*/

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
            __bic_SR_register_on_exit(LPM0_bits);              // Clear CPUOFF bit from LPM0
            break;             
        default:
            break; 
    }  
}

