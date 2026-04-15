// Transmitter.h
// Lab 9 — Audio Communication Transmitter
// ECE445L Spring 2025

#ifndef __TRANSMITTER_H__
#define __TRANSMITTER_H__
#include <stdint.h>

// ── Command codes (must match Receiver.h RX_CMD_*) ────────────────────────────
#define CMD_STOP     0u
#define CMD_FORWARD  1u
#define CMD_LEFT     2u
#define CMD_RIGHT    3u
#define CMD_BACKWARD 4u

// ── Continuous tones (receiver DFT bins k=1..5, fs=8 kHz, N=16) ───────────────
// STOP=500 Hz … BACK=2500 Hz — phase inc = f_Hz / 125 for 64-sample LUT @ 8 kHz
#define TX_INC_STOP     4u
#define TX_INC_FORWARD  8u
#define TX_INC_LEFT    12u
#define TX_INC_RIGHT   16u
#define TX_INC_BACK    20u

// Joystick: 12-bit ADC, calibrate centre at boot; dead-band (tune if stick drifts)
#define JOY_CENTER  2048
#define JOY_DEAD     200
/* Physical forward/back drives PE4 (AIN9, stored as FiltX/dx); left/right drives PE5.
   JOY_SWAP_XY=1 corrects this so dy tracks forward/back and dx tracks left/right. */
#ifndef JOY_SWAP_XY
#define JOY_SWAP_XY  1
#endif
/* Set to 1 if that axis reads high when you expect the opposite direction.
   JOY_INVERT_Y=1: physical forward gives negative old_dx; after swap that becomes
   negative dy, which would be BACKWARD — invert to make it FORWARD. */
#ifndef JOY_INVERT_X
#define JOY_INVERT_X 0
#endif
#ifndef JOY_INVERT_Y
#define JOY_INVERT_Y 1
#endif

// ── Public interface ──────────────────────────────────────────────────────────

// Transmitter_Init
// Initialises all transmitter peripherals:
//   Joystick:  ADC0 SS2 — data[0]=AIN8/PE5 (Y), data[1]=AIN9/PE4 (X)
//   Buttons:   LaunchPad_Input: bit0=PF0, bit1=PF4 (see LaunchPad.c)
//   DAC:       TLV5616 via SSI1 on PD0/PD1/PD3
//   Timer0A:   50 Hz  → InputTask  (reads joystick → updates CurrentCommand)
//   SysTick:   8 kHz  → sine from phase increment (command → frequency)
void Transmitter_Init(void);

// Returns the most recently decoded command
uint8_t  Transmitter_GetCommand(void);

// Returns cumulative count of InputTask calls (for CPU utilisation measurement)
uint32_t Transmitter_GetInputSamples(void);

// Returns cumulative count of SysTick DAC writes (for CPU utilisation measurement)
uint32_t Transmitter_GetWaveSamples(void);

// Returns EMA-filtered raw ADC readings (12-bit, 0..4095) — useful for LCD debug
uint16_t Transmitter_GetFiltX(void);
uint16_t Transmitter_GetFiltY(void);

// Returns signed displacement from calibrated center (after SWAP/INVERT applied)
// Positive dX = RIGHT candidate; positive dY = FORWARD candidate
int32_t  Transmitter_GetDX(void);
int32_t  Transmitter_GetDY(void);

// Returns the calibrated center ADC values measured at boot
uint32_t Transmitter_GetCalCX(void);
uint32_t Transmitter_GetCalCY(void);

// ── Unit tests (define RUN_UNIT_TESTS to call these from main) ─────────────────
// Test_DAC:       validates Sine64 LUT bounds and per-command output range [0,4095]
// Test_Joystick:  validates direction-mapping logic with known (dx,dy) inputs
// GetTests*:      read pass/fail counts via debugger or LCD after tests complete
void     Transmitter_Test_DAC(void);
void     Transmitter_Test_Joystick(void);
uint32_t Transmitter_GetTestsPassed(void);
uint32_t Transmitter_GetTestsFailed(void);

#endif // __TRANSMITTER_H__
