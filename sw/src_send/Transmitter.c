// Transmitter.c
// Lab 9 — Audio Communication Transmitter
// ECE445L Spring 2025
//
// Architecture (two ISRs, no FIFO needed):
//
//   SysTick  @ 8 kHz  — DACTask:
//     Reads CurrentCommand, generates the correct sine-wave sample(s),
//     writes to TLV5616 DAC via SSI1.
//
//   Timer0A  @ 50 Hz  — InputTask:
//     Reads joystick (AIN8/PE5 = X, AIN9/PE4 = Y) and LaunchPad buttons,
//     sets CurrentCommand.  SysTick sees the new command on its very next
//     call (at most 125 µs later) — no FIFO latency.
//
// Encoding:
//   CMD_STOP    → silence (DAC held at mid-scale 2048)
//   CMD_LEFT    → 500 Hz sine  (f1, INC_F1 = 4)
//   CMD_RIGHT   → 1000 Hz sine (f2, INC_F2 = 8)
//   CMD_FORWARD → f1 + f2 mixed (half-amplitude each to avoid clipping)
//   CMD_BACKWARD→ f1 for 100 ms, then f2 for 100 ms, alternating
//                 (receiver detects the alternation as BACKWARD)
//
// Hardware:
//   TLV5616 12-bit DAC → SSI1: PD0=SCLK, PD1=FS, PD3=DIN
//   Joystick X         → AIN8 / PE5
//   Joystick Y         → AIN9 / PE4
//   SW1 (hard stop)    → PF4 (negative logic)
//   SW2 (backward btn) → PF0 (negative logic)
//   LEDs               → PF1=Red, PF2=Blue, PF3=Green (driven from main)

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/Timer0A.h"
#include "../inc/TLV5616.h"
#include "../inc/LaunchPad.h"
#include "Joystick.h"
#include "Transmitter.h"

// ─── 64-point sine table, amplitude ≈ ±1692 (fits cleanly in int16_t) ────────
static const int16_t Sine64[64] = {
       0,  176,  351,  523,  689,  848,  998, 1137,
    1264, 1376, 1473, 1552, 1615, 1659, 1685, 1692,
    1680, 1649, 1600, 1534, 1451, 1352, 1238, 1110,
     969,  818,  658,  491,  319,  143,  -33, -208,
    -382, -551, -715, -871,-1018,-1154,-1278,-1387,
   -1481,-1559,-1619,-1662,-1686,-1692,-1678,-1646,
   -1596,-1530,-1446,-1347,-1233,-1106, -966, -814,
    -654, -487, -315, -139,   37,  213,  387,  556
};

#define SINE_MID          2048u   // 12-bit DAC midpoint (0 V output swing centre)
#define TIMER0A_50HZ   1600000u   // 80 MHz / 50 Hz
#define SYSTICK_RELOAD    9999u   // 80 MHz / 8000 Hz − 1
// Backward half-period in SysTick calls: 800 × 125 µs = 100 ms per phase
#define BACKWARD_TOGGLE    800u

// ─── Shared state (written by Timer0A ISR, read by SysTick ISR) ──────────────
// uint8_t writes are atomic on Cortex-M — no critical section needed.
static volatile uint8_t  CurrentCommand = CMD_STOP;

// ─── SysTick-private state (only touched inside SysTick_Handler) ─────────────
static volatile uint8_t  Phase1       = 0;
static volatile uint8_t  Phase2       = 0;
static volatile uint32_t SysTickCount = 0;
static volatile uint32_t WaveSamples  = 0;

// ─── Timer0A-private ─────────────────────────────────────────────────────────
static volatile uint32_t InputSamples = 0;

// ─── Helper: map joystick + buttons → command ────────────────────────────────
static uint8_t ReadCommand(void){
  uint32_t joy[2];
  uint8_t  sw;

  JoyStick_In(joy);       // joy[0] = AIN8/PE5 (X-axis)
                          // joy[1] = AIN9/PE4 (Y-axis)
  sw = JoyStick_Button(); // bit0 = SW2/PF0 pressed, bit1 = SW1/PF4 pressed

  // SW1 (PF4, bit1) — hard stop, overrides everything
  if(sw & 0x02u){ return CMD_STOP; }

  // Y-axis → forward / backward
  if((int32_t)joy[1] > (int32_t)(JOY_CENTER + JOY_DEAD)){ return CMD_FORWARD;  }
  if((int32_t)joy[1] < (int32_t)(JOY_CENTER - JOY_DEAD)){ return CMD_BACKWARD; }

  // X-axis → left / right
  if((int32_t)joy[0] > (int32_t)(JOY_CENTER + JOY_DEAD)){ return CMD_RIGHT; }
  if((int32_t)joy[0] < (int32_t)(JOY_CENTER - JOY_DEAD)){ return CMD_LEFT;  }

  // SW2 (PF0, bit0) — alternate backward button (e.g. no joystick)
  if(sw & 0x01u){ return CMD_BACKWARD; }

  return CMD_STOP;
}

// ─── Timer0A ISR @ 50 Hz ─────────────────────────────────────────────────────
// Reads input devices and updates CurrentCommand.
// SysTick reads CurrentCommand on its very next call (≤ 125 µs later).
static void InputTask(void){
  CurrentCommand = ReadCommand();
  InputSamples++;
}

// ─── SysTick ISR @ 8 kHz ─────────────────────────────────────────────────────
// Synthesises one audio sample per call and pushes it to the DAC.
// Derives increment values directly from CurrentCommand each call —
// no shared Inc1/Inc2 variables, no race conditions.
void SysTick_Handler(void){
  uint8_t  i1 = 0, i2 = 0;
  int32_t  sample = (int32_t)SINE_MID;

  switch(CurrentCommand){
    case CMD_LEFT:
      i1 = INC_F1;
      break;

    case CMD_RIGHT:
      i2 = INC_F2;
      break;

    case CMD_FORWARD:
      i1 = INC_F1;
      i2 = INC_F2;
      break;

    case CMD_BACKWARD:
      // Alternate f1 / f2 every BACKWARD_TOGGLE SysTick calls (= 100 ms).
      // (SysTickCount / BACKWARD_TOGGLE) & 1 switches between 0 and 1.
      if((SysTickCount / BACKWARD_TOGGLE) & 1u){
        i2 = INC_F2;   // second half-period → f2 only
      } else {
        i1 = INC_F1;   // first  half-period → f1 only
      }
      break;

    default:            // CMD_STOP
      break;            // i1 = i2 = 0  →  DC mid-scale output (silence)
  }
  SysTickCount++;

  // Accumulate phases and build sample
  // Each tone contributes half its amplitude so two tones together never clip.
  if(i1){ Phase1 = (Phase1 + i1) & 0x3Fu;  sample += Sine64[Phase1] / 2; }
  if(i2){ Phase2 = (Phase2 + i2) & 0x3Fu;  sample += Sine64[Phase2] / 2; }

  // Clamp to 12-bit DAC range
  if(sample < 0)         sample = 0;
  else if(sample > 4095) sample = 4095;

  DAC_Out_NB((uint16_t)sample);   // non-blocking SSI write (safe at 8 kHz)
  WaveSamples++;
}

// ─── Public API ──────────────────────────────────────────────────────────────
void Transmitter_Init(void){
  // Joystick: ADC0 SS2 on AIN8/AIN9, LaunchPad PF (switches + LEDs)
  JoyStick_Init();

  // Zero all state before enabling interrupts
  CurrentCommand = CMD_STOP;
  Phase1         = 0;
  Phase2         = 0;
  SysTickCount   = 0;
  InputSamples   = 0;
  WaveSamples    = 0;

  // DAC: TLV5616 on SSI1 (PD0=SCLK, PD1=FS, PD3=DIN), initial output = mid-scale
  DAC_Init(SINE_MID);

  // Timer0A @ 50 Hz, priority 3 (lower priority than SysTick)
  Timer0A_Init(&InputTask, TIMER0A_50HZ, 3);

  // SysTick @ 8 kHz, priority 1
  // Priority 1 > Timer0A priority 3, so SysTick always runs on time.
  NVIC_ST_CTRL_R    = 0;
  NVIC_ST_RELOAD_R  = SYSTICK_RELOAD;
  NVIC_ST_CURRENT_R = 0;
  // SysTick priority lives in bits [31:29] of NVIC_SYS_PRI3_R
  NVIC_SYS_PRI3_R   = (NVIC_SYS_PRI3_R & 0x00FFFFFFu) | 0x20000000u;  // priority 1
  NVIC_ST_CTRL_R    = 0x07u;   // enable SysTick, enable interrupt, use system clock
}

uint8_t  Transmitter_GetCommand(void)      { return CurrentCommand; }
uint32_t Transmitter_GetInputSamples(void) { return InputSamples;   }
uint32_t Transmitter_GetWaveSamples(void)  { return WaveSamples;    }
