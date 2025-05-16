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

#include <msp430h>

void init(void);

int main(void){
    init();
}

void init(void) {

        WDCTL = WDTPW | WDTHOLD;    //kill WDT

        P1DIR |= BIT5;              //Change P1.5(RUN LED) direction to output
        P1OUT |= BIT5;              //Set P1.5 HIGH

        PM5CTL0 &= ~LOCKLPM5;       //Unlock GPIO from High-Z

        
}
