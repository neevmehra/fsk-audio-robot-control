// Transmitter.h
// Lab 9 — Audio Communication Transmitter
// ECE445L Spring 2025

#ifndef __TRANSMITTER_H__
#define __TRANSMITTER_H__
#include <stdint.h>

// ── Command codes ─────────────────────────────────────────────────────────────
#define CMD_STOP     0   // silence
#define CMD_FORWARD  1   // f1 + f2 mixed
#define CMD_LEFT     2   // f1 only  (500 Hz)
#define CMD_RIGHT    3   // f2 only  (1000 Hz)
#define CMD_BACKWARD 4   // f1 and f2 alternating every 100 ms

// ── Tone frequencies ──────────────────────────────────────────────────────────
// Sample rate fs = 8000 Hz, 64-point sine table
// f1 = 500 Hz  → phase increment INC_F1 = 4  (4/64 × 8000 = 500 Hz)
// f2 = 1000 Hz → phase increment INC_F2 = 8  (8/64 × 8000 = 1000 Hz)
#define INC_F1   4
#define INC_F2   8

// ── Joystick thresholds (12-bit ADC, centre ≈ 2048) ──────────────────────────
#define JOY_CENTER  2048
#define JOY_DEAD     400   // dead-band either side of centre

// ── Public interface ──────────────────────────────────────────────────────────

// Transmitter_Init
// Initialises all transmitter peripherals:
//   Joystick:  ADC0 SS2 on AIN8/PE5 (X) and AIN9/PE4 (Y)
//   Buttons:   LaunchPad PF0 (SW2) and PF4 (SW1)
//   DAC:       TLV5616 via SSI1 on PD0/PD1/PD3
//   Timer0A:   50 Hz  → InputTask  (reads joystick → updates CurrentCommand)
//   SysTick:   8 kHz  → DACTask    (generates sine wave, writes to DAC)
void Transmitter_Init(void);

// Returns the most recently decoded command
uint8_t  Transmitter_GetCommand(void);

// Returns cumulative count of InputTask calls (for CPU utilisation measurement)
uint32_t Transmitter_GetInputSamples(void);

// Returns cumulative count of SysTick DAC writes (for CPU utilisation measurement)
uint32_t Transmitter_GetWaveSamples(void);

#endif // __TRANSMITTER_H__
