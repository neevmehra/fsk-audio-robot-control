// Lab9T.c
// Lab 9 — Audio Communication Transmitter main
// ECE445L Spring 2025
//
// This board has: joystick (PE4/PE5), TLV5616 DAC (PD0/PD1/PD3),
// LaunchPad switches (PF0/PF4) and LEDs (PF1–PF3).
//
// The main loop only drives the LED colour to show the current command.
// All real work happens in:
//   SysTick_Handler @ 8 kHz  — DAC output
//   Timer0A_Handler @ 50 Hz  — joystick + button read

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/PLL.h"
#include "../inc/LaunchPad.h"
#include "../inc/CortexM.h"
#include "Transmitter.h"

int main(void){
  DisableInterrupts();
  PLL_Init(Bus80MHz);       // 80 MHz system clock
  Transmitter_Init();       // set up DAC, ADC, timers
  EnableInterrupts();

  while(1){
    // Mirror current command on the RGB LED for visual debugging
    switch(Transmitter_GetCommand()){
      case CMD_FORWARD:  LaunchPad_Output(GREEN);  break;
      case CMD_LEFT:     LaunchPad_Output(BLUE);   break;
      case CMD_RIGHT:    LaunchPad_Output(RED);    break;
      case CMD_BACKWARD: LaunchPad_Output(YELLOW); break;
      default:           LaunchPad_Output(DARK);   break;
    }
  }
}
