// Receiver.c
// Lab 9 — Audio Communication Receiver (UART-FSK decoder)
// ECE445L Spring 2025
//
// Encoding matches Transmitter.c:
//   One steady tone per command: 500 / 1000 / 1500 / 2000 / 2500 Hz
//   DFT bins k=1..5 @ fs=8 kHz, N=16 → Mag1..Mag5
//
// Timing:
//   Timer2A ISR @ 8 kHz  → feeds samples into SampleFifo
//   Main loop DFT: every 16 samples = 2 ms → pick strongest bin
//
// Hardware:
//   Microphone (MAX4466) → AIN8 / PE5  (ADC0 SS3)
//   Motors    → RSLK2 (Motor.c)
//   Bump switches → Bump.c
//   SSD1306 I2C OLED → PA6/PA7  (MUST be connected)

#include <stdint.h>
#include <stdio.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/dsp.h"
#include "../inc/FIFO.h"
#include "../inc/Bump.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/SSD1306.h"
#include "../inc/Timer2A.h"
#include "Receiver.h"

AddIndexFifo(Sample, 64, int32_t, 1, 0)

#define RX_SAMPLE_PERIOD   10000u
#define RX_DFT_LEN            16u

#define RX_TONE_MIN_MAG    6000u
#define RX_STARTUP_TICKS    300u
#define RX_STABLE_PICKS       3u
#define RX_MOTOR_DUTY      14000u
#define RX_TONE_MIN_MAG      6000u
#define RX_WIN_MARGIN        1500u   // winner must beat runner-up by this much
#define RX_STABLE_MOVE_PICKS    3u   // movement commands: require 3 in a row
#define RX_STABLE_STOP_PICKS    8u   // STOP: require more evidence

static volatile uint8_t  CurrentCommand = RX_CMD_STOP;
static volatile uint32_t LastMag1 = 0;
static volatile uint32_t LastMag2 = 0;
static volatile uint32_t LastMag3 = 0;
static volatile uint32_t LastMag4 = 0;
static volatile uint32_t LastMag5 = 0;

static uint32_t DftIndex       = 0;
static uint32_t DisplayDivider = 0;
static uint32_t RxDecodeTicks  = 0;

static uint8_t  LastPick   = 0xFFu;
static uint8_t  PickAgree  = 0;

// ─── ADC (AIN8/PE5, software-trigger, SS3) ───────────────────────────────────
static void ReceiverADC_Init(void){
  volatile uint32_t delay;
  SYSCTL_RCGCADC_R  |= 0x01u;
  SYSCTL_RCGCGPIO_R |= 0x10u;
  delay = SYSCTL_RCGCGPIO_R;
  delay = SYSCTL_RCGCGPIO_R;
  (void)delay;
  while((SYSCTL_PRADC_R  & 0x01u) == 0u){};
  while((SYSCTL_PRGPIO_R & 0x10u) == 0u){};

  GPIO_PORTE_DIR_R   &= ~0x20u;
  GPIO_PORTE_AFSEL_R |=  0x20u;
  GPIO_PORTE_DEN_R   &= ~0x20u;
  GPIO_PORTE_AMSEL_R |=  0x20u;

  ADC0_PC_R     = (ADC0_PC_R & ~0x0Fu) | 0x01u;
  ADC0_SSPRI_R  = 0x3210u;
  ADC0_ACTSS_R &= ~0x08u;
  ADC0_EMUX_R  &= ~0xF000u;
  ADC0_SSMUX3_R = 8u;
  ADC0_SSCTL3_R = 0x0006u;
  ADC0_IM_R    &= ~0x08u;
  ADC0_ACTSS_R |=  0x08u;
}

static int32_t ReceiverADC_In(void){
  ADC0_PSSI_R = 0x08u;
  while((ADC0_RIS_R & 0x08u) == 0u){};
  int32_t s = (int32_t)(ADC0_SSFIFO3_R & 0x0FFFu);
  ADC0_ISC_R  = 0x08u;
  return s;
}

// ─── Timer2A ISR @ 8 kHz ─────────────────────────────────────────────────────
static void SampleTask(void){
  int32_t s = ReceiverADC_In() - 2048;
  SampleFifo_Put(s);
}

// ─── Motor + LED output ───────────────────────────────────────────────────────
static void ApplyRobotCommand(uint8_t cmd){
  if(Bump_In()){
    Motor_Forward(0, 0);
    LaunchPad_Output(DARK);
    return;
  }
  switch(cmd){
    case RX_CMD_FORWARD:
      Motor_Forward(RX_MOTOR_DUTY, RX_MOTOR_DUTY);
      LaunchPad_Output(GREEN);
      break;
    case RX_CMD_LEFT:
      Motor_Left(RX_MOTOR_DUTY, RX_MOTOR_DUTY);
      LaunchPad_Output(BLUE);
      break;
    case RX_CMD_RIGHT:
      Motor_Right(RX_MOTOR_DUTY, RX_MOTOR_DUTY);
      LaunchPad_Output(RED);
      break;
    case RX_CMD_BACKWARD:
      Motor_Backward(RX_MOTOR_DUTY, RX_MOTOR_DUTY);
      LaunchPad_Output(YELLOW);
      break;
    default:
      Motor_Forward(0, 0);
      LaunchPad_Output(DARK);
      break;
  }
}

static uint8_t TonePick(uint32_t m1, uint32_t m2, uint32_t m3, uint32_t m4, uint32_t m5){
  uint32_t mags[5];
  uint32_t best;
  uint32_t second;
  uint8_t bestIdx;
  uint8_t i;

  mags[0] = m1; // STOP
  mags[1] = m2; // FORWARD
  mags[2] = m3; // LEFT
  mags[3] = m4; // RIGHT
  mags[4] = m5; // BACKWARD

  best = mags[0];
  second = 0;
  bestIdx = 0;

  for(i = 1; i < 5; i++){
    if(mags[i] > best){
      second = best;
      best = mags[i];
      bestIdx = i;
    } else if(mags[i] > second){
      second = mags[i];
    }
  }

  // Signal too weak: treat as STOP
  if(best < RX_TONE_MIN_MAG){
    return RX_CMD_STOP;
  }

  // Winner not dominant enough: treat as STOP
  if((best - second) < RX_WIN_MARGIN){
    return RX_CMD_STOP;
  }

  switch(bestIdx){
    case 1: return RX_CMD_FORWARD;
    case 2: return RX_CMD_LEFT;
    case 3: return RX_CMD_RIGHT;
    case 4: return RX_CMD_BACKWARD;
    default:return RX_CMD_STOP;
  }
}

static void UpdateStableCommand(uint8_t pick){
  static uint8_t stopAgree = 0;

  if(pick == RX_CMD_STOP){
    stopAgree++;
    if(stopAgree >= RX_STABLE_STOP_PICKS){
      CurrentCommand = RX_CMD_STOP;
      stopAgree = RX_STABLE_STOP_PICKS;
    }
    return;
  }

  // Any valid movement command resets stop accumulation
  stopAgree = 0;

  if(pick == LastPick){
    if(PickAgree < RX_STABLE_MOVE_PICKS){
      PickAgree++;
    }
    if(PickAgree >= RX_STABLE_MOVE_PICKS){
      CurrentCommand = pick;
    }
  } else {
    LastPick  = pick;
    PickAgree = 1u;
  }
}

static void UpdateDisplay(void){
  char line[22];
  static const char *const CmdName[5] = {
    "Stop    ", "Forward ", "Left    ", "Right   ", "Back    "
  };
  uint8_t idx = (CurrentCommand < 5u) ? CurrentCommand : 0u;

  SSD1306_SetCursor(0, 0);
  SSD1306_OutString("Cmd:");
  SSD1306_OutString(CmdName[idx]);

  SSD1306_SetCursor(0, 2);
  snprintf(line, sizeof(line), "M1:%7lu", (unsigned long)LastMag1);
  SSD1306_OutString(line);

  SSD1306_SetCursor(0, 3);
  snprintf(line, sizeof(line), "M2:%7lu", (unsigned long)LastMag2);
  SSD1306_OutString(line);

  SSD1306_SetCursor(0, 4);
  snprintf(line, sizeof(line), "M3:%7lu", (unsigned long)LastMag3);
  SSD1306_OutString(line);

  SSD1306_SetCursor(0, 5);
  snprintf(line, sizeof(line), "min:%5lu", (unsigned long)RX_TONE_MIN_MAG);
  SSD1306_OutString(line);
}

void Receiver_Init(void){
  LaunchPad_Init();
  Bump_Init();
  Motor_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_OutClear();

  SampleFifo_Init();
  DFT_Init();
  ReceiverADC_Init();

  CurrentCommand = RX_CMD_STOP;
  LastMag1 = LastMag2 = LastMag3 = LastMag4 = LastMag5 = 0;
  DftIndex       = 0;
  DisplayDivider = 0;
  RxDecodeTicks  = 0;
  LastPick       = 0xFFu;
  PickAgree      = 0;

  Timer2A_Init(&SampleTask, RX_SAMPLE_PERIOD, 1);   // 8 kHz, priority 1
}

void Receiver_Run(void){
  int32_t sample;
  while(SampleFifo_Get(&sample)){
    DFT(DftIndex & 0x0Fu, sample);
    DftIndex++;

    if((DftIndex & (RX_DFT_LEN - 1u)) == 0u){
      LastMag1 = (uint32_t)Mag1();
      LastMag2 = (uint32_t)Mag2();
      LastMag3 = (uint32_t)Mag3();
      LastMag4 = (uint32_t)Mag4();
      LastMag5 = (uint32_t)Mag5();

      RxDecodeTicks++;

      {
        uint8_t pick = TonePick(LastMag1, LastMag2, LastMag3, LastMag4, LastMag5);
        UpdateStableCommand(pick);
      }

      if(RxDecodeTicks < RX_STARTUP_TICKS){
        ApplyRobotCommand(RX_CMD_STOP);
      } else {
        ApplyRobotCommand(CurrentCommand);
      }

      // Refresh OLED every 64 DFT periods (≈ 128 ms)
      DisplayDivider++;
      if((DisplayDivider & 0x3Fu) == 0u){
        UpdateDisplay();
      }
    }
  }
}

uint8_t  Receiver_GetCommand(void){ return CurrentCommand; }
uint32_t Receiver_GetMag1(void)   { return LastMag1;       }
uint32_t Receiver_GetMag2(void)   { return LastMag2;       }

// ─── Unit Tests ───────────────────────────────────────────────────────────────
// Define RUN_UNIT_TESTS at the project level to call these from Lab9R.c main().
// Results accumulate in RxTestsPassed / RxTestsFailed; read via getters or debugger.

static volatile uint32_t RxTestsPassed = 0;
static volatile uint32_t RxTestsFailed = 0;

#define RX_TEST_ASSERT(cond)  do { if(cond){ RxTestsPassed++; } else { RxTestsFailed++; } } while(0)

// Test 3 — ADC: collect 64 samples from the FIFO populated by Timer2A and
// verify every sample is a valid offset-removed 12-bit reading: [-2048, 2047].
// Call AFTER Receiver_Init() so Timer2A is already running.
void Receiver_Test_ADC(void){
  int32_t  sample;
  uint32_t collected = 0u;
  uint32_t inRange   = 0u;
  uint32_t timeout   = 0u;

  while(collected < 64u && timeout < 800000u){
    if(SampleFifo_Get(&sample)){
      // SampleTask stores (ADC_raw - 2048), so valid range is [-2048, 2047]
      if(sample >= -2048 && sample <= 2047){ inRange++; }
      collected++;
    }
    timeout++;
  }

  // Pass if we received enough samples and all were in range
  RX_TEST_ASSERT(collected >= 60u);
  RX_TEST_ASSERT(inRange == collected);
}

// Test 4 — DFT: feed 16 samples of a pure 1000 Hz sine (= DFT bin k=2, FORWARD
// command) and verify Mag2 is the dominant bin and exceeds RX_TONE_MIN_MAG.
//
// Sample derivation: x[i] = round(1800 * sin(2*pi * 1000/8000 * i))
//   at fs=8 kHz the 1000 Hz tone completes exactly two cycles over 16 samples.
void Receiver_Test_DFT(void){
  static const int32_t tone1000Hz[16] = {
       0,  1273,  1800,  1273,     0, -1273, -1800, -1273,
       0,  1273,  1800,  1273,     0, -1273, -1800, -1273
  };
  int32_t m1, m2, m3, m4, m5;
  uint8_t i;

  DFT_Init();
  for(i = 0u; i < 16u; i++){
    DFT((uint32_t)i, tone1000Hz[i]);
  }
  // Consume all five magnitudes (each call resets its own accumulators)
  m1 = Mag1(); m2 = Mag2(); m3 = Mag3(); m4 = Mag4(); m5 = Mag5();

  // Bin k=2 must be largest
  RX_TEST_ASSERT(m2 > m1);
  RX_TEST_ASSERT(m2 > m3);
  RX_TEST_ASSERT(m2 > m4);
  RX_TEST_ASSERT(m2 > m5);

  // Signal must be strong enough to exceed the receiver's minimum detection threshold
  RX_TEST_ASSERT((uint32_t)m2 > RX_TONE_MIN_MAG);
}

uint32_t Receiver_GetTestsPassed(void){ return RxTestsPassed; }
uint32_t Receiver_GetTestsFailed(void){ return RxTestsFailed; }
