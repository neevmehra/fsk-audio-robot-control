// Receiver.c
// Lab 9 — Audio Communication Receiver
// ECE445L Spring 2025
//
// Architecture:
//
//   Timer2A  @ 8 kHz  — SampleTask:
//     Triggers ADC0 SS3, reads AIN8/PE5 (microphone), subtracts DC bias,
//     pushes the signed sample into SampleFifo.
//
//   Main loop  — Receiver_Run:
//     Drains SampleFifo, feeds a 16-point DFT, computes magnitudes at
//     f1 = 500 Hz (k = 1) and f2 = 1000 Hz (k = 2) every 16 samples,
//     decodes the command, and drives the RSLK motors.
//
// Decoding:
//   Mag1 only  → LEFT
//   Mag2 only  → RIGHT
//   Both       → FORWARD
//   Alternating LEFT/RIGHT (transmitter toggles 100 ms each) → BACKWARD
//   Neither    → STOP
//
// Threshold:
//   RX_MAG_THRESHOLD = 5000
//   Requires signal amplitude ≥ ~9 ADC counts at PE5.
//   Increase this value if the robot reacts to background noise;
//   decrease it if it fails to react when sound is played nearby.
//
// Hardware:
//   Microphone (MAX4466 + electret) → AIN8 / PE5  (ADC0 SS3)
//   Motors     → RSLK2 (Motor.c: PF2/PF3 PWM, PA2/PA3 direction)
//   Bumpers    → Bump.c (PA5, PF0, PB3, PC4)
//   LEDs       → LaunchPad PF1–PF3
//   Display    → SSD1306 I2C OLED (PA6 = SCL, PA7 = SDA)
//                MUST be connected — I2C will hang if absent.

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

// ─── Sample FIFO (64 × int32_t; producer = Timer2A ISR, consumer = main) ─────
AddIndexFifo(Sample, 64, int32_t, 1, 0)

// ─── Tuning constants ─────────────────────────────────────────────────────────
#define RX_SAMPLE_PERIOD   10000u   // Timer2A reload: 80 MHz / 8 kHz
#define RX_DFT_LEN            16u   // DFT window (one result every 2 ms)
#define RX_MAG_THRESHOLD    5000u   // min Mag² to count as "tone present"
                                    // raise if false triggers; lower if misses
#define RX_ACTION_HOLD        50u   // DFT periods to hold command after last hit
                                    // 50 × 2 ms = 100 ms coasting
#define RX_MOTOR_DUTY      14000u   // PWM duty (range 2 .. 39998)

// ─── Module state ─────────────────────────────────────────────────────────────
static volatile uint8_t  CurrentCommand   = RX_CMD_STOP;
static volatile uint32_t LastMag1         = 0;
static volatile uint32_t LastMag2         = 0;
static volatile uint32_t ActionHold       = 0;

static uint32_t DftIndex         = 0;
static uint8_t  RawPrevious      = RX_CMD_STOP;
static uint8_t  AlternatingCount = 0;
static uint32_t DisplayDivider   = 0;

// ─── ADC initialisation (PE5 / AIN8, software-trigger, SS3) ─────────────────
static void ReceiverADC_Init(void){
  volatile uint32_t delay;

  SYSCTL_RCGCADC_R  |= 0x01u;   // enable ADC0 run-mode clock
  SYSCTL_RCGCGPIO_R |= 0x10u;   // enable Port E run-mode clock
  delay = SYSCTL_RCGCGPIO_R;    // let clock stabilise (two dummy reads)
  delay = SYSCTL_RCGCGPIO_R;
  (void)delay;
  while((SYSCTL_PRADC_R  & 0x01u) == 0u){};  // wait ADC0 peripheral ready
  while((SYSCTL_PRGPIO_R & 0x10u) == 0u){};  // wait Port E peripheral ready

  // Configure PE5 as analog input (AIN8)
  GPIO_PORTE_DIR_R   &= ~0x20u;  // PE5 input
  GPIO_PORTE_AFSEL_R |=  0x20u;  // alternate function (ADC)
  GPIO_PORTE_DEN_R   &= ~0x20u;  // disable digital buffer
  GPIO_PORTE_AMSEL_R |=  0x20u;  // enable analog mode

  // ADC0 SS3: software-trigger, single-sample (AIN8), 125 kSPS max
  ADC0_PC_R     = (ADC0_PC_R & ~0x0Fu) | 0x01u;  // 125 kSPS
  ADC0_SSPRI_R  = 0x3210u;                         // SS3 = lowest priority
  ADC0_ACTSS_R &= ~0x08u;                          // disable SS3 during config
  ADC0_EMUX_R  &= ~0xF000u;                        // SS3 trigger = software
  ADC0_SSMUX3_R = 8u;                              // select AIN8 (PE5)
  ADC0_SSCTL3_R = 0x0006u;                         // IE0=1, END0=1
  ADC0_IM_R    &= ~0x08u;                          // no NVIC interrupt (we busy-wait)
  ADC0_ACTSS_R |=  0x08u;                          // enable SS3
}

// Software-trigger ADC0 SS3, busy-wait for result, return signed 12-bit value.
// Called from Timer2A ISR — conversion takes ~3 µs at 125 kSPS (2.4% ISR load).
static int32_t ReceiverADC_In(void){
  ADC0_PSSI_R = 0x08u;
  while((ADC0_RIS_R & 0x08u) == 0u){};
  int32_t s = (int32_t)(ADC0_SSFIFO3_R & 0x0FFFu);
  ADC0_ISC_R  = 0x08u;   // clear raw interrupt flag
  return s;
}

// ─── Timer2A ISR @ 8 kHz ─────────────────────────────────────────────────────
static void SampleTask(void){
  int32_t s = ReceiverADC_In() - 2048;   // subtract DC midpoint
  SampleFifo_Put(s);
}

// ─── Decode magnitudes → raw command ─────────────────────────────────────────
// f1 (Mag1 @ 500 Hz) and f2 (Mag2 @ 1000 Hz) are orthogonal DFT bins,
// so they are independent: detecting one does NOT leak into the other.
static uint8_t DecodeFromMagnitudes(uint32_t m1, uint32_t m2){
  uint8_t has1 = (m1 > RX_MAG_THRESHOLD);
  uint8_t has2 = (m2 > RX_MAG_THRESHOLD);

  if(has1 && has2){ return RX_CMD_FORWARD; }
  if(has1)        { return RX_CMD_LEFT;    }
  if(has2)        { return RX_CMD_RIGHT;   }
  return RX_CMD_STOP;
}

// ─── Backward detection ───────────────────────────────────────────────────────
// The transmitter encodes BACKWARD as alternating LEFT/RIGHT tones (100 ms each).
// We count command *transitions* — not every DFT call — so that 50 consecutive
// LEFT results (during one 100 ms half-period) count as a single event.
static uint8_t ApplyBackwardDetection(uint8_t raw){
  if(raw == RawPrevious){
    // Command unchanged — hold the existing count; no update needed.
    return (AlternatingCount >= 3u) ? RX_CMD_BACKWARD : raw;
  }

  // Command has changed — update alternation counter.
  if(((raw == RX_CMD_LEFT)  && (RawPrevious == RX_CMD_RIGHT)) ||
     ((raw == RX_CMD_RIGHT) && (RawPrevious == RX_CMD_LEFT))){
    // Strict LEFT↔RIGHT alternation: count it.
    if(AlternatingCount < 255u){ AlternatingCount++; }
  } else {
    // Any other transition (e.g. STOP, FORWARD) resets the counter.
    AlternatingCount = 0;
  }
  RawPrevious = raw;

  return (AlternatingCount >= 3u) ? RX_CMD_BACKWARD : raw;
}

// ─── Motor + LED output ───────────────────────────────────────────────────────
static void ApplyRobotCommand(uint8_t cmd){
  if(Bump_In()){
    // Any bump switch → emergency stop
    Motor_Forward(0, 0);
    LaunchPad_Output(DARK);
    ActionHold = 0;
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
    default:               // RX_CMD_STOP
      Motor_Forward(0, 0);
      LaunchPad_Output(DARK);
      break;
  }
}

// ─── OLED display (updated every ~128 ms) ────────────────────────────────────
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

  SSD1306_SetCursor(0, 5);
  snprintf(line, sizeof(line), "Bump:%d Hold:%2lu",
           (int)Bump_In(), (unsigned long)ActionHold);
  SSD1306_OutString(line);
}

// ─── Public API ──────────────────────────────────────────────────────────────
void Receiver_Init(void){
  LaunchPad_Init();                          // PF LEDs + buttons
  Bump_Init();                               // bump switches
  Motor_Init();                              // RSLK PWM + direction pins
  SSD1306_Init(SSD1306_SWITCHCAPVCC);        // I2C OLED (PA6/PA7)
  SSD1306_OutClear();

  SampleFifo_Init();
  DFT_Init();
  ReceiverADC_Init();                        // ADC0 SS3 on AIN8/PE5

  CurrentCommand   = RX_CMD_STOP;
  LastMag1         = 0;
  LastMag2         = 0;
  ActionHold       = 0;
  DftIndex         = 0;
  RawPrevious      = RX_CMD_STOP;
  AlternatingCount = 0;
  DisplayDivider   = 0;

  Timer2A_Init(&SampleTask, RX_SAMPLE_PERIOD, 1);  // 8 kHz, priority 1
}

void Receiver_Run(void){
  int32_t sample;
  while(SampleFifo_Get(&sample)){
    // Feed one sample into the running 16-point DFT
    DFT(DftIndex & 0x0Fu, sample);
    DftIndex++;

    // Every 16 samples (every 2 ms) compute magnitudes and act
    if((DftIndex & (RX_DFT_LEN - 1u)) == 0u){
      uint8_t raw;

      LastMag1 = (uint32_t)Mag1();   // magnitude² at f1 = 500 Hz; clears accumulators
      LastMag2 = (uint32_t)Mag2();   // magnitude² at f2 = 1000 Hz

      raw            = DecodeFromMagnitudes(LastMag1, LastMag2);
      CurrentCommand = ApplyBackwardDetection(raw);

      // Refresh action-hold timer
      if(CurrentCommand != RX_CMD_STOP){
        ActionHold = RX_ACTION_HOLD;
      } else if(ActionHold > 0u){
        ActionHold--;
      }

      // Drive motors (stop if hold expired)
      ApplyRobotCommand(ActionHold ? CurrentCommand : RX_CMD_STOP);

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
