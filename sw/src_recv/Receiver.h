// Receiver.h
// Lab 9
// Programs to implement receiver functionality   
// ECE445L Spring 2025


// ----------------------------------------------------------------------------
// Hardware/software assigned to receiver
//   Timer2A ADC0 samples sound
//   Fifo3 linkage from  ADC to Decoder
//   main-loop runs decoder
//   RSLK robot with microphone connected to ADC input

#ifndef __RECEIVER_H__
#define __RECEIVER_H__
#include <stdint.h>

#define RX_CMD_STOP      0
#define RX_CMD_FORWARD   1
#define RX_CMD_LEFT      2
#define RX_CMD_RIGHT     3
#define RX_CMD_BACKWARD  4

void Receiver_Init(void);
void Receiver_Run(void);
uint8_t Receiver_GetCommand(void);
uint32_t Receiver_GetMag1(void);
uint32_t Receiver_GetMag2(void);



#endif
