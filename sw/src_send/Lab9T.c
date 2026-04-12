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

// Print a signed 4-digit decimal at the current cursor: "+NNNN" or "-NNNN"
// Clamps to ±9999 to keep fixed width.
static void LCD_OutSDec4(int32_t v){
  if(v < 0){
    ST7735_OutChar('-');
    v = -v;
  } else {
    ST7735_OutChar('+');
  }
  if(v > 9999) v = 9999;
  ST7735_OutUDec4((uint32_t)v);
}

// Update ST7735 debug display — only redraws rows that changed.
//
// Layout (21 cols x 16 rows, size=1, black background):
//   Row  0: "Joystick Debug "  white  (static, drawn once at init)
//   Row  1: "Cmd: XXXXXXXXX "  colored (updated when cmd changes)
//   Row  3: "dX:+NNNN d=NNNN"  cyan   (signed X displacement + dead zone)
//   Row  4: "dY:+NNNN        "  cyan   (signed Y displacement)
//   Row  6: "Cx:NNNN Cy:NNNN"  grey   (calibrated centers, drawn once)
//
// Reading the display:
//   Push joystick RIGHT  → watch dX sign/magnitude
//   Push joystick FORWARD → watch dY sign/magnitude
//   If the wrong axis changes, set JOY_SWAP_XY 1 in Transmitter.h
//   If the sign is backwards, set JOY_INVERT_X or JOY_INVERT_Y 1
//   If you never leave STOP, compare |dX|/|dY| against the dead zone value
static void LCD_UpdateJoystick(uint8_t cmd){
  static uint8_t  prevCmd = 0xFF;
  static int32_t  prevDX  = 0x7FFFFFFF;
  static int32_t  prevDY  = 0x7FFFFFFF;

  int32_t dx = Transmitter_GetDX();
  int32_t dy = Transmitter_GetDY();

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
    ST7735_DrawString(5, 1, dirStr, dirColor);
  }

  if(dx != prevDX){
    prevDX = dx;
    ST7735_SetCursor(0, 3);
    ST7735_SetTextColor(ST7735_CYAN);
    ST7735_OutString("dX:");
    LCD_OutSDec4(dx);
  }

  if(dy != prevDY){
    prevDY = dy;
    ST7735_SetCursor(0, 4);
    ST7735_SetTextColor(ST7735_CYAN);
    ST7735_OutString("dY:");
    LCD_OutSDec4(dy);
  }
}

int main(void){
  DisableInterrupts();
  PLL_Init(Bus80MHz);
  Transmitter_Init();

  // ST7735R init — try INITR_BLACKTAB if colors look wrong on your display
  ST7735_InitR(INITR_REDTAB);
  ST7735_FillScreen(ST7735_BLACK);

  // Static labels — drawn once
  ST7735_DrawString(0, 0, "Joystick Debug", ST7735_WHITE);
  ST7735_DrawString(0, 1, "Cmd:",           ST7735_WHITE);

  // Dead zone reference — constant, show it once
  ST7735_SetCursor(0, 5);
  ST7735_SetTextColor(ST7735_LIGHTGREY);
  ST7735_OutString("Dead:");
  ST7735_OutUDec4(JOY_DEAD);

  // Calibrated centers — set during Transmitter_Init, show once here
  ST7735_SetCursor(0, 6);
  ST7735_SetTextColor(ST7735_LIGHTGREY);
  ST7735_OutString("Cx:");
  ST7735_OutUDec4(Transmitter_GetCalCX());
  ST7735_OutString(" Cy:");
  ST7735_OutUDec4(Transmitter_GetCalCY());

  EnableInterrupts();

  // SysTick sanity check — wait ~100 ms and verify ISR fired
  volatile uint32_t delay = 8000000;
  while(delay--);

  if(Transmitter_GetWaveSamples() == 0){
    ST7735_FillScreen(ST7735_BLACK);
    ST7735_DrawString(0, 0, "FAULT",         ST7735_RED);
    ST7735_DrawString(0, 1, "SysTick = 0",  ST7735_WHITE);
    ST7735_DrawString(0, 2, "Check build &", ST7735_WHITE);
    ST7735_DrawString(0, 3, "Project>Clean", ST7735_WHITE);
    while(1){
      LaunchPad_Output(WHITE);
      delay = 800000; while(delay--);
      LaunchPad_Output(DARK);
      delay = 800000; while(delay--);
    }
  }

  // LED mirrors command; LCD shows signed displacements for axis debug.
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
