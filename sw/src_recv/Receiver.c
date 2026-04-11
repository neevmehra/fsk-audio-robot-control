// Receiver.c
// Lab 9 — Audio Communication Receiver (UART-FSK decoder)
// ECE445L Spring 2025
//
// Encoding matches Transmitter.c:
//   f1 = 500 Hz  = mark  (logic 1)   — DFT bin k=1
//   f2 = 1000 Hz = space (logic 0)   — DFT bin k=2
//   100 baud (10 ms/bit)
//   Frame: start(f2) + 8 data LSB-first + even parity + stop(f1)
//
// Timing:
//   Timer2A ISR @ 8 kHz  → feeds samples into SampleFifo
//   Main loop DFT: every 16 samples = 2 ms → one frequency decision
//   WINDOWS_PER_BIT = 5  (10 ms / 2 ms)
//
// UART decoder state machine (runs in Receiver_Run):
//   IDLE  → waits for first f2 (start-bit edge)
//   SYNC  → waits 2 more DFT windows to centre on start bit, verifies f2
//   DATA  → samples 8 data bits, then one parity bit
//   STOP  → verifies stop bit (f1) + even parity, then outputs command
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

// ─── Sample FIFO (64 × int32_t) ──────────────────────────────────────────────
AddIndexFifo(Sample, 64, int32_t, 1, 0)

// ─── Constants ───────────────────────────────────────────────────────────────
#define RX_SAMPLE_PERIOD   10000u   // Timer2A reload: 80 MHz / 8 kHz
#define RX_DFT_LEN            16u   // DFT window → one decision every 2 ms
#define RX_MAG_THRESHOLD    8000u   // min Mag² to count tone as present (raise if false starts)
// Space (1000 Hz) must exceed mark (500 Hz) by this much to begin a frame — rejects broadband noise
#define RX_SPACE_OVER_MARK  6000u
#define RX_STARTUP_TICKS    300u    // ~600 ms @ 2 ms/decode; motors stay stopped while ADC/DFT settle
#define RX_MOTOR_DUTY      14000u   // PWM duty (2 .. 39998)

// UART timing: 10 ms/bit = 5 DFT windows per bit
#define WINDOWS_PER_BIT        5u
#define SYNC_WAIT              2u   // DFT windows to wait after start-edge
                                    // → samples near bit centre

// ─── UART decoder states ──────────────────────────────────────────────────────
#define STATE_IDLE   0
#define STATE_SYNC   1
#define STATE_DATA   2
#define STATE_STOP   3

// ─── Module state ─────────────────────────────────────────────────────────────
static volatile uint8_t  CurrentCommand = RX_CMD_STOP;
static volatile uint32_t LastMag1       = 0;
static volatile uint32_t LastMag2       = 0;

static uint32_t DftIndex       = 0;
static uint32_t DisplayDivider = 0;

// UART decoder
static uint8_t  UartState   = STATE_IDLE;
static uint8_t  UartTimer   = 0;    // countdown within current state
static uint8_t  UartBitCnt  = 0;    // data bits collected so far
static uint8_t  UartRxByte  = 0;    // byte being assembled
static uint8_t  UartParityRx = 0;   // received parity bit (0/1)
static uint32_t RxDecodeTicks = 0; // DFT decode calls since init (for startup mute)

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

// ─── OLED display (every ~128 ms) ────────────────────────────────────────────
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
  snprintf(line, sizeof(line), "St:%d Rx:0x%02X",
           (int)UartState, (unsigned int)CurrentCommand);
  SSD1306_OutString(line);
}

// ─── UART-FSK decoder (called every DFT window = every 2 ms) ─────────────────
// has1/has2: quick thresholds; mag1/mag2: raw Mag² — use dominance to avoid noise false starts
static void UartDecode(uint8_t has1, uint8_t has2, uint32_t mag1, uint32_t mag2){
  switch(UartState){

    // ── IDLE: real start bit = strong 1000 Hz and clearly stronger than 500 Hz bin ──
    case STATE_IDLE:
      if((mag2 > RX_MAG_THRESHOLD) && (mag2 > mag1 + RX_SPACE_OVER_MARK)){
        UartTimer  = SYNC_WAIT;
        UartState  = STATE_SYNC;
      }
      break;

    // ── SYNC: verify start bit is still present after centering ──────────────
    case STATE_SYNC:
      UartTimer--;
      if(UartTimer == 0){
        if(has2){
          // Valid start bit confirmed — prepare to receive 8 data bits
          UartRxByte = 0;
          UartBitCnt = 0;
          UartTimer  = WINDOWS_PER_BIT;
          UartState  = STATE_DATA;
        } else {
          UartState = STATE_IDLE;   // false start, go back
        }
      }
      break;

    // ── DATA: 8 data bits then parity bit ───────────────────────────────────
    case STATE_DATA:
      UartTimer--;
      if(UartTimer == 0){
        if(UartBitCnt < 8u){
          if(has1){
            UartRxByte |= (uint8_t)(1u << UartBitCnt);
          }
          UartBitCnt++;
          UartTimer = WINDOWS_PER_BIT;
        } else if(UartBitCnt == 8u){
          UartParityRx = has1 ? 1u : 0u;
          UartBitCnt   = 9u;
          UartTimer    = WINDOWS_PER_BIT;
          UartState    = STATE_STOP;
        }
      }
      break;

    // ── STOP: verify stop bit (f1) + even parity ─────────────────────────────
    case STATE_STOP:
      UartTimer--;
      if(UartTimer == 0){
        if(has1){
          uint8_t x = UartRxByte;
          x ^= x >> 4;
          x ^= x >> 2;
          x ^= x >> 1;
          uint8_t wantP = x & 1u;
          if((wantP ^ (UartParityRx & 1u)) == 0u){
            if(UartRxByte <= RX_CMD_BACKWARD){
              CurrentCommand = UartRxByte;
            }
          }
        }
        UartState = STATE_IDLE;
      }
      break;

    default:
      UartState = STATE_IDLE;
      break;
  }
}

// ─── Public API ──────────────────────────────────────────────────────────────
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
  LastMag1       = 0;
  LastMag2       = 0;
  DftIndex       = 0;
  DisplayDivider = 0;
  UartState      = STATE_IDLE;
  UartTimer      = 0;
  UartBitCnt     = 0;
  UartRxByte     = 0;
  UartParityRx   = 0;
  RxDecodeTicks  = 0;

  Timer2A_Init(&SampleTask, RX_SAMPLE_PERIOD, 1);   // 8 kHz, priority 1
}

void Receiver_Run(void){
  int32_t sample;
  while(SampleFifo_Get(&sample)){
    DFT(DftIndex & 0x0Fu, sample);
    DftIndex++;

    if((DftIndex & (RX_DFT_LEN - 1u)) == 0u){
      uint8_t has1, has2;

      LastMag1 = (uint32_t)Mag1();   // mag² at f1 = 500 Hz
      LastMag2 = (uint32_t)Mag2();   // mag² at f2 = 1000 Hz

      has1 = (LastMag1 > RX_MAG_THRESHOLD);
      has2 = (LastMag2 > RX_MAG_THRESHOLD);

      RxDecodeTicks++;

      UartDecode(has1, has2, LastMag1, LastMag2);

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
uint32_t Receiver_GetMag2(void)   { return LastMag2        ;}
