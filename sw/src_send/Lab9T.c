// ----------------------------------------------------------------------------
// 
// File name:     Lab9.c
//
// ----------------------------------------------------------------------------
//
// Description:   This lab is designed to be used on two devices
//                This device is the transmitter with a joystick, DAC and speaker
//                The other device is the RSLK robot
// Author:        Your names
// Orig gen date: August 4, 2024
// Date:          Your date
// Goal of this lab: Audio Communication of the robot
//

// ----------------------------------------------------------------------------
// Hardware/software assigned to common
// main initialization initializes all modules
//   PD2, Timer5A, ADC1, UART0 implements TExaSdisplay
// ----------------------------------------------------------------------------
// Hardware/software assigned to transmitter
//   UART0, possible source of input data (conflicts with TExaSdisplay)
//   PC7-PC4 Port_C_Init, possible source of input data
//   Timer0A periodic interrupt to generate input
//   Fifo1 linkage from Input to Encoder
//   Timer1A periodic interrupt create a sequence of frequencies
//   Fifo2 linkage from  Encoder to DAC
//   SSI1 PD3 PD1 PD0 TLV5616 DAC output
//   SysTick ISR output to DAC
// ----------------------------------------------------------------------------


#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/Texas.h"
#include "../inc/PLL.h"
#include "../inc/ST7735.h"
#include "Transmitter.h"
#include "../inc/dsp.h"


void DisableInterrupts(void);           // Disable interrupts
void EnableInterrupts(void);            // Enable interrupts
int main(void){
  DisableInterrupts();
  PLL_Init(Bus80MHz);    // bus clock at 80 MHz
  // write this
  EnableInterrupts();
  while(1){
      // write this
  }
}


