// Lab9R.c
// Lab 9 — Audio Communication Receiver main
// ECE445L Spring 2025
//
// This board is the RSLK2 robot with:
//   Microphone + MAX4466 amplifier → AIN8/PE5
//   SSD1306 OLED display (I2C)     → PA6/PA7
//   RSLK2 motors (PWM + dir)       → PF2/PF3, PA2/PA3
//   Bump switches                  → PA5, PF0, PB3, PC4
//
// All real work happens in:
//   Timer2A_Handler @ 8 kHz — ADC sample → FIFO
//   Receiver_Run (main loop) — DFT + decode + motor control

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/PLL.h"
#include "../inc/CortexM.h"
#include "Receiver.h"

int main(void){
  DisableInterrupts();
  PLL_Init(Bus80MHz);   // 80 MHz system clock
  Receiver_Init();      // ADC, motors, display, Timer2A
  EnableInterrupts();

  while(1){
    Receiver_Run();     // drain FIFO, DFT, decode, drive motors
  }
}
