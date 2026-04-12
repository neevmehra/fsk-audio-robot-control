// Lab9T.c
// Lab 9 — Audio Communication Transmitter main
// ECE445L Spring 2025

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/PLL.h"
#include "../inc/LaunchPad.h"
#include "../inc/CortexM.h"
#include "../inc/ST7735.h"
#include "Transmitter.h"

// Update ST7735 with current joystick command and raw filtered ADC values.
// Only redraws rows that changed to avoid flicker.
// Display layout (21 cols x 16 rows, size=1):
//   Row 0: "Joystick Debug"  (white, drawn once)
//   Row 1: "Cmd:"            (white, drawn once)
//   Row 2: direction string  (color-coded, updated on change)
//   Row 3: "X: "             (white, drawn once)
//   Row 4: 4-digit X value   (yellow, updated on change)
//   Row 5: "Y: "             (white, drawn once)
//   Row 6: 4-digit Y value   (yellow, updated on change)
static void LCD_UpdateJoystick(uint8_t cmd){
  static uint8_t  prevCmd = 0xFF;
  static uint16_t prevX   = 0xFFFF;
  static uint16_t prevY   = 0xFFFF;

  uint16_t fx = Transmitter_GetFiltX();
  uint16_t fy = Transmitter_GetFiltY();

  if(cmd != prevCmd){
    prevCmd = cmd;
    uint16_t dirColor;
    char    *dirStr;
    switch(cmd){
      case CMD_FORWARD:  dirStr = "FORWARD   "; dirColor = ST7735_GREEN;   break;
      case CMD_LEFT:     dirStr = "LEFT      "; dirColor = ST7735_BLUE;    break;
      case CMD_RIGHT:    dirStr = "RIGHT     "; dirColor = ST7735_RED;     break;
      case CMD_BACKWARD: dirStr = "BACKWARD  "; dirColor = ST7735_YELLOW;  break;
      default:           dirStr = "STOP      "; dirColor = ST7735_WHITE;   break;
    }
    ST7735_DrawString(0, 2, dirStr, dirColor);
  }

  if(fx != prevX){
    prevX = fx;
    ST7735_SetCursor(3, 4);
    ST7735_SetTextColor(ST7735_YELLOW);
    ST7735_OutUDec4(fx);
  }

  if(fy != prevY){
    prevY = fy;
    ST7735_SetCursor(3, 6);
    ST7735_SetTextColor(ST7735_YELLOW);
    ST7735_OutUDec4(fy);
  }
}

int main(void){
  DisableInterrupts();
  PLL_Init(Bus80MHz);
  Transmitter_Init();

  // ST7735R init — try INITR_BLACKTAB if colors look inverted on your display
  ST7735_InitR(INITR_REDTAB);
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_DrawString(0, 0, "Joystick Debug", ST7735_WHITE);
  ST7735_DrawString(0, 1, "Cmd:",           ST7735_WHITE);
  ST7735_DrawString(0, 3, "X:",             ST7735_WHITE);
  ST7735_DrawString(0, 5, "Y:",             ST7735_WHITE);

  EnableInterrupts();

  // ── SysTick sanity check ────────────────────────────────────────────────
  // Wait ~100 ms (8,000,000 cycles at 80 MHz) and check if SysTick fired.
  // At 8 kHz, 100 ms = 800 SysTick calls → WaveSamples should be ~800.
  volatile uint32_t delay = 8000000;
  while(delay--);

  if(Transmitter_GetWaveSamples() == 0){
    // SysTick never fired — show fault on LCD and blink WHITE forever.
    // Fix: check that Transmitter.c is in the CCS project build, do Project→Clean.
    ST7735_FillScreen(ST7735_BLACK);
    ST7735_DrawString(0, 0, "FAULT",        ST7735_RED);
    ST7735_DrawString(0, 1, "SysTick = 0", ST7735_WHITE);
    while(1){
      LaunchPad_Output(WHITE);
      delay = 800000; while(delay--);
      LaunchPad_Output(DARK);
      delay = 800000; while(delay--);
    }
  }

  // Solid LED = current command; LCD shows direction + raw ADC for debugging.
  while(1){
    uint8_t cmd = Transmitter_GetCommand();
    switch(cmd){
      case CMD_FORWARD:  LaunchPad_Output(GREEN);  break;
      case CMD_LEFT:     LaunchPad_Output(BLUE);   break;
      case CMD_RIGHT:    LaunchPad_Output(RED);    break;
      case CMD_BACKWARD: LaunchPad_Output(YELLOW); break;
      default:           LaunchPad_Output(DARK);   break;
    }
    LCD_UpdateJoystick(cmd);
  }
}
