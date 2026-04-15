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
#include <stdio.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/PLL.h"
#include "../inc/CortexM.h"
#include "Receiver.h"

int main(void){
  DisableInterrupts();
  PLL_Init(Bus80MHz);   // 80 MHz system clock
  Receiver_Init();      // ADC, motors, display, Timer2A
  EnableInterrupts();

#ifdef RUN_UNIT_TESTS
  // Run receiver unit tests before entering the main loop.
  // Test_DFT uses only pre-computed samples (no hardware).
  // Test_ADC drains live FIFO samples — Timer2A must be running (it is, see above).
  // Pass/fail counts are visible via Receiver_GetTestsPassed/Failed() in the debugger,
  // and briefly shown on the OLED display.
  Receiver_Test_DFT();
  Receiver_Test_ADC();
  {
    uint32_t passed = Receiver_GetTestsPassed();
    uint32_t failed = Receiver_GetTestsFailed();
    char line[22];
    SSD1306_OutClear();
    SSD1306_SetCursor(0, 0);
    SSD1306_OutString("Unit Tests");
    SSD1306_SetCursor(0, 1);
    SSD1306_OutString(failed == 0u ? "ALL PASS" : "FAIL");
    snprintf(line, sizeof(line), "Pass:%lu", (unsigned long)passed);
    SSD1306_SetCursor(0, 2); SSD1306_OutString(line);
    snprintf(line, sizeof(line), "Fail:%lu", (unsigned long)failed);
    SSD1306_SetCursor(0, 3); SSD1306_OutString(line);
    volatile uint32_t d = 40000000u; while(d--);
  }
#endif

  while(1){
    Receiver_Run();     // drain FIFO, DFT, decode, drive motors
  }
}
