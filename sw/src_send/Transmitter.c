// Transmitter.c
// Lab 9 — Audio Communication Transmitter (UART-FSK)
// ECE445L Spring 2025
//
// Encoding — UART-like FSK at 100 baud:
//   f1 = 500 Hz  = mark  (logic 1, idle, stop bit)
//   f2 = 1000 Hz = space (logic 0, start bit)
//
//   Frame (10 bits × 10 ms = 100 ms):
//     [start: f2] [b0 b1 b2 b3 b4 b5 b6 b7] [stop: f1]
//     LSB first, exactly like UART 8N1.
//
//   The current command byte is retransmitted every frame.
//   Receiver simply watches for start-bit edge and decodes.
//
// Hardware:
//   VRx → PE4 / AIN9 → joy[1]   (X-axis: left/right)
//   VRy → PE5 / AIN8 → joy[0]   (Y-axis: forward/backward)
//   TLV5616 DAC → SSI1: PD0=SCLK, PD1=FS, PD3=DIN
//   SW1 (stop) → PF4,  SW2 (back) → PF0
//
// Architecture:
//   Timer0A @ 50 Hz — joystick → CurrentCommand
//   SysTick @ 8 kHz — UART-FSK encoder → DAC

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/Timer0A.h"
#include "../inc/TLV5616.h"
#include "../inc/LaunchPad.h"
#include "Joystick.h"
#include "CommandFifo.h"
#include "Transmitter.h"

// ─── 64-point sine table, amplitude ±1800 ────────────────────────────────────
// round(1800 * sin(k * 2*pi / 64)) for k = 0..63
// ─── 64-point sine table, amplitude ±1800 ────────────────────────────────────
// round(1800 * sin(k * 2*pi / 64)) for k = 0..63
// ─── 64-point sine table, amplitude ±1800 ────────────────────────────────────
// round(1800 * sin(k * 2*pi / 64)) for k = 0..63
static const int16_t Sine64[64] = {
      0,  176,  351,  522,  689,  849, 1000, 1142,
   1273, 1391, 1497, 1588, 1663, 1723, 1765, 1791,
   1800, 1791, 1765, 1723, 1663, 1588, 1497, 1391,
   1273, 1142, 1000,  849,  689,  522,  351,  176,
      0, -176, -351, -522, -689, -849,-1000,-1142,
  -1273,-1391,-1497,-1588,-1663,-1723,-1765,-1791,
  -1800,-1791,-1765,-1723,-1663,-1588,-1497,-1391,
  -1273,-1142,-1000, -849, -689, -522, -351, -176
};

#define SINE_MID        2048u
#define TIMER0A_50HZ 1600000u   // 80 MHz / 50 Hz
#define SYSTICK_RELOAD   9999u  // 80 MHz / 8000 Hz - 1

// UART baud: 100 baud → 80 SysTick calls per bit = 10 ms
#define BIT_PERIOD    80u
#define FRAME_BITS    11u

// ─── Shared state ─────────────────────────────────────────────────────────────
static volatile uint8_t  CurrentCommand = CMD_STOP;
static volatile uint8_t  Phase1         = 0;
static volatile uint8_t  Phase2         = 0;
static volatile uint32_t WaveSamples    = 0;
static volatile uint32_t InputSamples   = 0;

// UART TX frame state (written only in SysTick context)
static volatile uint8_t  TxBitIdx   = 0;           // 0=start, 1-8=data, 9=stop
static volatile uint8_t  TxBitTimer = BIT_PERIOD;
static volatile uint8_t  TxFrame    = CMD_STOP;     // byte being transmitted

// ─── Joystick calibration ─────────────────────────────────────────────────────
static uint32_t CalCenterX = JOY_CENTER;
static uint32_t CalCenterY = JOY_CENTER;

static uint8_t LastQueuedCmd = CMD_STOP;

static uint8_t EvenParity8(uint8_t b){
  b ^= b >> 4;
  b ^= b >> 2;
  b ^= b >> 1;
  return b & 1u;
}

static void CalibrateJoystick(void){
  uint32_t joy[2], sumX = 0, sumY = 0;
  uint8_t i;
  for(i = 0; i < 32; i++){
    JoyStick_In(joy);
    sumX += joy[1];   // VRx = AIN9/PE4
    sumY += joy[0];   // VRy = AIN8/PE5
  }
  CalCenterX = sumX >> 5;
  CalCenterY = sumY >> 5;
}

// ─── Timer0A ISR @ 50 Hz ─────────────────────────────────────────────────────
static uint8_t ReadCommand(void){
  uint32_t joy[2];
  uint8_t sw;
  JoyStick_In(joy);
  sw = JoyStick_Button();

  if(sw & 0x02u){
    return CMD_STOP;
  }

  if((int32_t)joy[0] > (int32_t)CalCenterY + (int32_t)JOY_DEAD){
    return CMD_FORWARD;
  }
  if((int32_t)joy[0] < (int32_t)CalCenterY - (int32_t)JOY_DEAD){
    return CMD_BACKWARD;
  }
  if((int32_t)joy[1] > (int32_t)CalCenterX + (int32_t)JOY_DEAD){
    return CMD_RIGHT;
  }
  if((int32_t)joy[1] < (int32_t)CalCenterX - (int32_t)JOY_DEAD){
    return CMD_LEFT;
  }

  if(sw & 0x01u){
    return CMD_BACKWARD;
  }
  return CMD_STOP;
}

static void InputTask(void){
  uint8_t c = ReadCommand();
  CurrentCommand = c;
  if(c != LastQueuedCmd){
    if(CommandFifo_Put(c)){
      LastQueuedCmd = c;
    }
  }
  InputSamples++;
}

// ─── SysTick ISR @ 8 kHz — UART-FSK encoder ─────────────────────────────────
// Each call outputs one DAC sample.  The bit period timer advances the
// UART frame one bit at a time.
//   Bit 0      = start bit → f2 (space, 1000 Hz)
//   Bits 1–8   = data bits → f1=mark=1, f2=space=0  (LSB first)
//   Bit 9      = stop bit  → f1 (mark,  500 Hz)
void SysTick_Handler(void){
  int32_t sample = (int32_t)SINE_MID;
  uint8_t mark;   // 1 → output f1 (500 Hz),  0 → output f2 (1000 Hz)

  // Determine mark/space for the current bit position
  if(TxBitIdx == 0){
    mark = 0u;                                   // start bit = space
  } else if(TxBitIdx <= 8u){
    mark = (uint8_t)((TxFrame >> (TxBitIdx - 1u)) & 1u);
  } else if(TxBitIdx == 9u){
    mark = EvenParity8(TxFrame);
  } else {
    mark = 1u;                                   // stop bit = mark
  }

  if(mark){
    Phase1 = (uint8_t)((Phase1 + INC_F1) & 0x3Fu);
    sample += Sine64[Phase1] / 2;
  } else {
    Phase2 = (uint8_t)((Phase2 + INC_F2) & 0x3Fu);
    sample += Sine64[Phase2] / 2;
  }

  TxBitTimer--;
  if(TxBitTimer == 0u){
    TxBitTimer = BIT_PERIOD;
    TxBitIdx++;
    if(TxBitIdx >= FRAME_BITS){
      uint8_t next = CMD_STOP;
      if(!CommandFifo_Get(&next)){
        next = CurrentCommand;
      }
      TxFrame  = next;
      TxBitIdx = 0;
    }
  }

  if(sample < 0){
    sample = 0;
  } else if(sample > 4095){
    sample = 4095;
  }

  DAC_Out_NB((uint16_t)sample);
  WaveSamples++;
}

// ─── Public API ──────────────────────────────────────────────────────────────
void Transmitter_Init(void){
  JoyStick_Init();

  CommandFifo_Init();
  LastQueuedCmd = CMD_STOP;
  CommandFifo_Put(CMD_STOP);

  CurrentCommand = CMD_STOP;
  Phase1 = Phase2 = 0;
  InputSamples = WaveSamples = 0;
  TxFrame    = CMD_STOP;
  TxBitIdx   = 0;
  TxBitTimer = BIT_PERIOD;

  // Measure actual joystick resting position — don't touch stick during boot
  CalibrateJoystick();

  DAC_Init(SINE_MID);
  Timer0A_Init(&InputTask, TIMER0A_50HZ, 3);

  // SysTick @ 8 kHz, priority 1 (higher than Timer0A)
  NVIC_ST_CTRL_R    = 0;
  NVIC_ST_RELOAD_R  = SYSTICK_RELOAD;
  NVIC_ST_CURRENT_R = 0;
  NVIC_SYS_PRI3_R   = (NVIC_SYS_PRI3_R & 0x00FFFFFFu) | 0x20000000u;
  NVIC_ST_CTRL_R    = 0x07u;
}

uint8_t  Transmitter_GetCommand(void)      { return CurrentCommand; }
uint32_t Transmitter_GetInputSamples(void) { return InputSamples;   }
uint32_t Transmitter_GetWaveSamples(void)  { return WaveSamples;    }
