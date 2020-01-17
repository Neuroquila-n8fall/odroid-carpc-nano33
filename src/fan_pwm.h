#pragma once
#include <Arduino.h>

//Maximaler dutycycle also frequenz. 1919 entspricht 100%DC @ 25kHz
const int MAX_FANDUTY = 1919;

//Minimaler cycle.
const int MIN_FANDUTY = 0;

//PWM Setup
void setupPWM();
//Set Duty Cycle
void setFanDutyCycle(int pcCycle);

void setupPWM()
{

    //Code found on forums
    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) | // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                      GCLK_GENDIV_ID(4);   // Select Generic Clock (GCLK) 4
    while (GCLK->STATUS.bit.SYNCBUSY)
        ; // Wait for synchronization

    REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |         // Set the duty cycle to 50/50 HIGH/LOW
                       GCLK_GENCTRL_GENEN |       // Enable GCLK4
                       GCLK_GENCTRL_SRC_DFLL48M | // Set the 48MHz clock source
                       GCLK_GENCTRL_ID(4);        // Select GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY)
        ; // Wait for synchronization

    // Enable the port multiplexer for the 4 PWM channels: timer TCC0 outputs
    const uint8_t CHANNELS = 2;
    const uint8_t pwmPins[] = {2, 3}; //D2 and D3
    for (uint8_t i = 0; i < CHANNELS; i++)
    {
        PORT->Group[g_APinDescription[pwmPins[i]].ulPort].PINCFG[g_APinDescription[pwmPins[i]].ulPin].bit.PMUXEN = 1;
    }
    // Connect the TCC0 timer to the port outputs - port pins are paired odd PMUO and even PMUXE
    // F & E specify the timers: TCC0, TCC1 and TCC2
    //D2 is even
    PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;
    //D3 is odd
    PORT->Group[g_APinDescription[3].ulPort].PMUX[g_APinDescription[3].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F;

    // Feed GCLK4 to TCC0 and TCC1
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |       // Enable GCLK4 to TCC0 and TCC1
                       GCLK_CLKCTRL_GEN_GCLK4 |   // Select GCLK4
                       GCLK_CLKCTRL_ID_TCC0_TCC1; // Feed GCLK4 to TCC0 and TCC1
    while (GCLK->STATUS.bit.SYNCBUSY)
        ; // Wait for synchronization

    // Normal (single slope) PWM operation: timer countinuouslys count up to PER register value and then is reset to 0
    REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM; // Setup single slope PWM on TCC0
    while (TCC0->SYNCBUSY.bit.WAVE)
        ; // Wait for synchronization

    // Each timer counts up to a maximum or TOP value set by the PER register,
    // this determines the frequency of the PWM operation:
    // 20000 = 50Hz, 10000 = 100Hz, 2500  = 400Hz
    REG_TCC0_PER = MAX_FANDUTY; // Set the frequency of the PWM on TCC0 to 50Hz
    while (TCC0->SYNCBUSY.bit.PER)
        ;

    // The CCBx register value corresponds to the pulsewidth in microseconds (us)
    //Set Fans to maximum
    REG_TCC0_CCB0 = 100;
    while (TCC0->SYNCBUSY.bit.CCB0)
        ;
    REG_TCC0_CCB1 = 100;
    while (TCC0->SYNCBUSY.bit.CCB1)
        ;

    // Divide the 48MHz signal by 1 giving 48MHz (20.8ns) TCC0 timer tick and enable the outputs
    REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 | // Divide GCLK4 by 1
                      TCC_CTRLA_ENABLE;          // Enable the TCC0 output
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ; // Wait for synchronization
}

void setFanDutyCycle(int pcCycle)
{
    REG_TCC0_CCB1 = pcCycle;
    while (TCC0->SYNCBUSY.bit.CCB1)
        ;
    REG_TCC0_CCB0 = pcCycle;
    while (TCC0->SYNCBUSY.bit.CCB0)
        ;
}
