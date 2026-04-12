// Lab9T.c
// Lab 9 — Audio Communication Transmitter main
// ECE445L Spring 2025

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/PLL.h"
#include "../inc/LaunchPad.h"
#include "../inc/CortexM.h"
#include "Transmitter.h"

int main(void){
  DisableInterrupts();
  PLL_Init(Bus80MHz);
  Transmitter_Init();
  EnableInterrupts();

  // ── SysTick sanity check ────────────────────────────────────────────────
  // Wait ~100 ms (8,000,000 cycles at 80 MHz) and check if SysTick fired.
  // At 8 kHz, 100 ms = 800 SysTick calls → WaveSamples should be ~800.
  volatile uint32_t delay = 8000000;
  while(delay--);

  if(Transmitter_GetWaveSamples() == 0){
    // SysTick never fired — blink WHITE rapidly forever as a fault indicator.
    // Fix: check that Transmitter.c is in the CCS project build, do Project→Clean.
    while(1){
      LaunchPad_Output(WHITE);
      delay = 800000; while(delay--);
      LaunchPad_Output(DARK);
      delay = 800000; while(delay--);
    }
  }

  // Solid LED = current command (no 50% duty blink — that made direction look random).
  while(1){
    uint8_t cmd = Transmitter_GetCommand();
    switch(cmd){
      case CMD_FORWARD:  LaunchPad_Output(GREEN);  break;
      case CMD_LEFT:     LaunchPad_Output(BLUE);   break;
      case CMD_RIGHT:    LaunchPad_Output(RED);    break;
      case CMD_BACKWARD: LaunchPad_Output(YELLOW); break;
      default:           LaunchPad_Output(DARK);   break;
    }
  }
}
