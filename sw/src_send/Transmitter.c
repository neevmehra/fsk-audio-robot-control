// Transmitter.c
// Lab 9 — Audio Communication Transmitter (continuous tone per command)
// ECE445L Spring 2025
//
// Encoding — one steady sine per command (matches receiver DFT bins k=1..5 @ 8 kHz, N=16):
//   STOP=500 Hz, FORWARD=1000 Hz, LEFT=1500 Hz, RIGHT=2000 Hz, BACK=2500 Hz
//
// Hardware (ADC_In89): data[0]=AIN8/PE5, data[1]=AIN9/PE4
//   Default mapping: joy[1] → left/right, joy[0] → forward/back (see JOY_SWAP_XY)
//   TLV5616 DAC → SSI1: PD0=SCLK, PD1=FS, PD3=DIN
//   SW1 (stop) → PF4,  SW2 (back) → PF0
//
// Architecture:
//   Timer0A @ 50 Hz — joystick → CurrentCommand (debounced so pitch does not chatter)
//   SysTick @ 8 kHz — phase increment from command → DAC

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/Timer0A.h"
#include "../inc/TLV5616.h"
#include "../inc/LaunchPad.h"
#include "Joystick.h"
#include "Transmitter.h"

// 64-point sine, ±1800 → with SINE_MID gives ~0..4095 span when added full-scale
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
/* Use full table swing for louder speaker drive (was /2 — too quiet for many boards). */
#define SINE_SCALE_NUM  1
#define SINE_SCALE_DEN  1
#define TIMER0A_50HZ    1600000u
#define SYSTICK_RELOAD  9999u

/* Phase increment per 8 kHz sample: f_Hz = inc * 8000 / 64 = inc * 125 */
static const uint8_t TxInc[5] = { 4u, 8u, 12u, 16u, 20u };

#define STABLE_NEED  2u

static volatile uint8_t  CurrentCommand = CMD_STOP;
static volatile uint32_t WaveSamples    = 0;
static volatile uint32_t InputSamples   = 0;
static volatile int32_t  LastDX         = 0;  // signed X displacement from calibrated center
static volatile int32_t  LastDY         = 0;  // signed Y displacement from calibrated center

static volatile uint8_t TxPhase = 0;

// ─── Joystick calibration ─────────────────────────────────────────────────────
static uint32_t CalCenterX = JOY_CENTER;
static uint32_t CalCenterY = JOY_CENTER;

static uint8_t DebCand  = CMD_STOP;
static uint8_t DebCount = 0;

/* EMA of raw ADC; joy[1]=PE4, joy[0]=PE5 — matches CalCenterX/Y */
static uint16_t FiltX = JOY_CENTER;
static uint16_t FiltY = JOY_CENTER;

// Add these near the top of Transmitter.c
volatile uint32_t DebugRawX = 0;
volatile uint32_t DebugRawY = 0;

static void CalibrateJoystick(void){
  uint32_t joy[2], sumX = 0, sumY = 0;
  uint8_t i;
  for(i = 0; i < 32; i++){
    JoyStick_In(joy);
    sumX += joy[1];
    sumY += joy[0];
  }
  CalCenterX = sumX >> 5;
  CalCenterY = sumY >> 5;
}

/* Dominant axis wins so diagonals map to one direction; optional swap/invert in Transmitter.h */
static uint8_t ReadCommand(void){
  uint32_t joy[2];
  uint8_t sw;
  int32_t dx, dy;
  int32_t adx, ady;
  int32_t dead = (int32_t)JOY_DEAD;

  // --- ADD THESE TWO LINES RIGHT HERE ---
  DebugRawX = joy[1]; // X-axis (PE4)
  DebugRawY = joy[0]; // Y-axis (PE5)

  JoyStick_In(joy);
  // EMA Filtering
  FiltX = (uint16_t)(((uint32_t)FiltX * 3u + joy[1]) >> 2);
  FiltY = (uint16_t)(((uint32_t)FiltY * 3u + joy[0]) >> 2);

  // Apply Calibration
#if JOY_SWAP_XY
  dx = (int32_t)FiltY - (int32_t)CalCenterY;
  dy = (int32_t)FiltX - (int32_t)CalCenterX;
#else
  dx = (int32_t)FiltX - (int32_t)CalCenterX;
  dy = (int32_t)FiltY - (int32_t)CalCenterY;
#endif

#if JOY_INVERT_X
  dx = -dx;
#endif
#if JOY_INVERT_Y
  dy = -dy;
#endif

  LastDX = dx;
  LastDY = dy;
  adx = dx >= 0 ? dx : -dx;
  ady = dy >= 0 ? dy : -dy;

  sw = JoyStick_Button();

  // 1. High Priority: STOP Switch (SW2)
  if(sw & 0x02u) return CMD_STOP;

  // 2. Deadzone check: If center, check if the BACK switch (SW1) is pressed
  if(adx <= dead && ady <= dead){
    if(sw & 0x01u) return CMD_FORWARD; // swapped
    return CMD_STOP;
  }

  // 3. Directional Logic: Determine if we are more "Vertical" or "Horizontal"
  if(ady > adx){
    // Vertical is dominant
    if(dy > dead) return CMD_BACKWARD;   // swapped
    if(dy < -dead) return CMD_FORWARD; // swapped
  } else {
    // Horizontal is dominant
    if(dx > dead) return CMD_RIGHT;
    if(dx < -dead) return CMD_LEFT;
  }

  // 4. Default fallback: check switch one last time, else STOP
  //if(sw & 0x01u) return CMD_FORWARD;   // swapped
  return CMD_STOP;
}

static void InputTask(void){
  uint8_t r = ReadCommand();

  if(r == CurrentCommand){
    DebCand  = r;
    DebCount = 0;
  } else if(r == DebCand){
    DebCount++;
    if(DebCount >= STABLE_NEED){
      CurrentCommand = r;
      DebCount = 0;
    }
  } else {
    DebCand  = r;
    DebCount = 1u;
  }

  InputSamples++;
}

// ─── SysTick ISR @ 8 kHz — sine from phase increment (command → frequency) ──
void SysTick_Handler(void){
  int32_t sample;
  uint8_t inc;

  inc = TxInc[CurrentCommand % 5u];
  TxPhase = (uint8_t)((TxPhase + inc) & 0x3Fu);

  sample = (int32_t)SINE_MID;
  sample += (Sine64[TxPhase] * SINE_SCALE_NUM) / SINE_SCALE_DEN;

  if(sample < 0){
    sample = 0;
  } else if(sample > 4095){
    sample = 4095;
  }

  DAC_Out_NB((uint16_t)sample);
  WaveSamples++;
}

void Transmitter_Init(void){
  JoyStick_Init();

  CurrentCommand = CMD_STOP;
  TxPhase        = 0;
  InputSamples   = 0;
  WaveSamples    = 0;
  DebCand        = CMD_STOP;
  DebCount       = 0;

  CalibrateJoystick();
  FiltX = (uint16_t)CalCenterX;
  FiltY = (uint16_t)CalCenterY;

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
uint16_t Transmitter_GetFiltX(void)        { return FiltX;          }
uint16_t Transmitter_GetFiltY(void)        { return FiltY;          }
int32_t  Transmitter_GetDX(void)           { return LastDX;         }
int32_t  Transmitter_GetDY(void)           { return LastDY;         }
uint32_t Transmitter_GetCalCX(void)        { return CalCenterX;     }
uint32_t Transmitter_GetCalCY(void)        { return CalCenterY;     }

// ─── Unit Tests ───────────────────────────────────────────────────────────────
// Compile-time gated — define RUN_UNIT_TESTS in project settings to enable.
// Results accumulate in TestsPassed / TestsFailed; read via getters or debugger.

static volatile uint32_t TestsPassed = 0;
static volatile uint32_t TestsFailed = 0;

#define TEST_ASSERT(cond)  do { if(cond){ TestsPassed++; } else { TestsFailed++; } } while(0)

// Test 1 — DAC: verify Sine64 LUT range and that every (command, phase) pair
// produces a 12-bit output value that fits in [0, 4095].
void Transmitter_Test_DAC(void){
  uint8_t i;

  // 1a. Every LUT entry must stay within ±1800 (amplitude bound used in Transmitter_Init).
  for(i = 0; i < 64u; i++){
    TEST_ASSERT(Sine64[i] >= -1800 && Sine64[i] <= 1800);
  }

  // 1b. Known spot-checks: zero crossing at index 0 and 32, peak at 16, trough at 48.
  TEST_ASSERT(Sine64[0]  ==     0);
  TEST_ASSERT(Sine64[16] ==  1800);
  TEST_ASSERT(Sine64[32] ==     0);
  TEST_ASSERT(Sine64[48] == -1800);

  // 1c. Simulate 64 SysTick steps for every command and verify no output clips.
  {
    uint8_t cmd, j;
    for(cmd = 0; cmd < 5u; cmd++){
      uint8_t phase = 0u;
      uint8_t inc   = TxInc[cmd % 5u];
      for(j = 0; j < 64u; j++){
        phase = (uint8_t)((phase + inc) & 0x3Fu);
        int32_t sample = (int32_t)SINE_MID
                       + (Sine64[phase] * SINE_SCALE_NUM) / SINE_SCALE_DEN;
        TEST_ASSERT(sample >= 0 && sample <= 4095);
      }
    }
  }
}

// Pure direction-mapping logic mirroring ReadCommand() — operates on pre-computed
// (dx, dy) so no hardware is required.
static uint8_t JoyLogic(int32_t dx, int32_t dy){
  int32_t adx  = dx >= 0 ? dx : -dx;
  int32_t ady  = dy >= 0 ? dy : -dy;
  int32_t dead = (int32_t)JOY_DEAD;

  if(adx <= dead && ady <= dead){ return CMD_STOP; }
  if(ady > adx){
    if(dy >  dead){ return CMD_BACKWARD; }
    if(dy < -dead){ return CMD_FORWARD;  }
  } else {
    if(dx >  dead){ return CMD_RIGHT; }
    if(dx < -dead){ return CMD_LEFT;  }
  }
  return CMD_STOP;
}

// Test 2 — Joystick: verify direction mapping for center, four cardinal directions,
// and two diagonal cases where the dominant axis should win.
void Transmitter_Test_Joystick(void){
  int32_t D = (int32_t)JOY_DEAD;

  // Deadband: displacement within ±JOY_DEAD on both axes → STOP
  TEST_ASSERT(JoyLogic(0,       0)       == CMD_STOP);
  TEST_ASSERT(JoyLogic(D - 1,   0)       == CMD_STOP);
  TEST_ASSERT(JoyLogic(0,       D - 1)   == CMD_STOP);

  // Cardinal directions (well outside deadband)
  TEST_ASSERT(JoyLogic(0,      -(D+300)) == CMD_FORWARD);
  TEST_ASSERT(JoyLogic(0,       (D+300)) == CMD_BACKWARD);
  TEST_ASSERT(JoyLogic(-(D+300), 0)      == CMD_LEFT);
  TEST_ASSERT(JoyLogic( (D+300), 0)      == CMD_RIGHT);

  // Diagonals — dominant axis wins
  TEST_ASSERT(JoyLogic(D+200, D+500) == CMD_BACKWARD); // |dy| > |dx|
  TEST_ASSERT(JoyLogic(D+500, D+200) == CMD_RIGHT);    // |dx| > |dy|
}

uint32_t Transmitter_GetTestsPassed(void){ return TestsPassed; }
uint32_t Transmitter_GetTestsFailed(void){ return TestsFailed; }
