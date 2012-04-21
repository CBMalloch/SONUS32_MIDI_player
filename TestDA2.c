/*
** TestDA2.c
**    generating a square waveform
**
** 07/10/06 v1.0 LDJ
** 11/25/07 v2.0 PIC32 porting 
*/
// configuration bit settings, Fcy=72MHz, Fpb=36MHz
#pragma config POSCMOD=XT, FNOSC=PRIPLL 
#pragma config FPLLIDIV=DIV_2, FPLLMUL=MUL_18, FPLLODIV=DIV_1
#pragma config FPBDIV=DIV_2, FWDTEN=OFF, CP=OFF, BWP=OFF

#include <p32xxxx.h>
#include <plib.h>
#include <explore.h>

void initDA( int samplerate)
{
    // init OC1 module 
    OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE,
            0, 0);

    // init Timer2 mode and period (PR2)
    OpenTimer2( T2_ON | T2_PS_1_1 |  T2_SOURCE_INT, 
                FPB/samplerate);         
    mT2SetIntPriority( 4); 
    mT2ClearIntFlag();
    mT2IntEnable( 1);     
} // initDA

int count;

void __ISR( _TIMER_2_VECTOR, ipl4) T2Interrupt( void)
{
    OC1RS = (count < 22) ? PR2 : 0;
    count++;
    if ( count >= 44)
        count = 0;

    // clear interrupt flag and exit
    mT2ClearIntFlag();
} // T2 Interrupt


main( void)
{
    initEX16();         // init and enable vectored interrupts
    initDA( 44100);     // init the PWM for 44.1kHz
    SetDCOC1PWM( PR2/2);
    
    // main loop
    while( 1);

}// main

