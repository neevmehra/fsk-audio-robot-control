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
uint8_t  Receiver_GetCommand(void);
uint32_t Receiver_GetMag1(void);
uint32_t Receiver_GetMag2(void);

// ── Unit tests (define RUN_UNIT_TESTS to call these from main) ─────────────────
// Test_ADC: collects 64 FIFO samples and checks every value is in [-2048, 2047]
//           (call AFTER Receiver_Init so Timer2A is running)
// Test_DFT: feeds 16 hardcoded 1000 Hz samples; asserts Mag2 dominates and
//           exceeds RX_TONE_MIN_MAG (no hardware required)
// GetTests*: read accumulated pass/fail counts via debugger or OLED display
void     Receiver_Test_ADC(void);
void     Receiver_Test_DFT(void);
uint32_t Receiver_GetTestsPassed(void);
uint32_t Receiver_GetTestsFailed(void);

#endif
