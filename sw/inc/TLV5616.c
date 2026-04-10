// TLV5616.c
// Runs on TM4C123
// Use SSI1 to send a 16-bit code to the TLV5616 and return the reply.
// Daniel Valvano
// EE445L Fall 2015
//    Jonathan W. Valvano 9/22/15

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// SSIClk (SCLK) connected to PD0
// SSIFss (FS)   connected to PD1
// SSITx (DIN)   connected to PD3

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

#define TLV5616_CMD(code) (0x1000 | ((code) & 0x0FFF))

//----------------   DAC_Init     -------------------------------------------
// Initialize TLV5616 12-bit DAC
// assumes bus clock is 80 MHz
// inputs: initial voltage output (0 to 4095)
// outputs:none
void DAC_Init(uint16_t data){
  volatile uint32_t delay;
  SYSCTL_RCGCSSI_R |= 0x02;      // activate SSI1
  SYSCTL_RCGCGPIO_R |= 0x08;     // activate Port D
  delay = SYSCTL_RCGCGPIO_R;
  (void)delay;

  GPIO_PORTD_AFSEL_R |= 0x0B;    // PD3,PD1,PD0 alternate function
  GPIO_PORTD_DEN_R |= 0x0B;      // digital enable
  GPIO_PORTD_AMSEL_R &= ~0x0B;   // disable analog
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xFFFF0F00) + 0x00002022;

  SSI1_CR1_R = 0x00000000;       // disable SSI during config
  SSI1_CPSR_R = 4;               // 20 MHz SSI clock with 80 MHz bus
  SSI1_CR0_R = 0x0000000F;       // Freescale, SPO=0, SPH=0, 16-bit
  SSI1_DR_R = TLV5616_CMD(data); // preload first sample
  SSI1_CR1_R = 0x00000002;       // enable SSI
}

// --------------     DAC_Out   --------------------------------------------
// Send data to TLV5616 12-bit DAC
// inputs:  voltage output (0 to 4095)
// 
void DAC_Out(uint16_t code){
  while((SSI1_SR_R&SSI_SR_TNF) == 0){};
  SSI1_DR_R = TLV5616_CMD(code);
}

// --------------     DAC_OutNonBlocking   ------------------------------------
// Send data to TLV5616 12-bit DAC without checking for room in the FIFO
// inputs:  voltage output (0 to 4095)
// 
void DAC_Out_NB(uint16_t code){
  SSI1_DR_R = TLV5616_CMD(code);
}
