// File **********Transmitter.c***********
// Lab 9
// Programs to implement transmitter functionality   
// EE445L Spring 2025

// Hardware/software assigned to transmitter
//   UART0, possible source of input data (conflicts with TExaSdisplay)
//   Input comes from Joystick and switches
//   Timer0A periodic interrupt to generate input
//   FreqFifo linkage from Encoder to SendData
//   Timer1A periodic interrupt create a sequence of frequencies
//   SSI1 PD3 PD1 PD0 TLV5616 DAC output
//   SysTick ISR output to DAC

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/Timer0A.h"
#include "../inc/Timer1A.h"
#include "../inc/FIFO.h"
#include "../inc/TLV5616.h"
#include "Transmitter.h"
    // write this
   
