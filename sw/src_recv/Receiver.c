// Receiver.c
// Lab 9
// Programs to implement receiver functionality   
// ECE445L Spring 2025


// ----------------------------------------------------------------------------
// Hardware/software assigned to receiver
//   Timer2A ADC0 samples sound
//   Fifo3 linkage from  ADC to Decoder
//   main-loop runs decoder
//   RSLK robot with microphone connected to ADC input

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/dsp.h"
#include "../inc/FIFO.h"
#include "Receiver.h"
// write this

