// ----------------------------------------------------------------------------
// 
// File name:     JoyStick.c
//
// ----------------------------------------------------------------------------
//
// Description:   This interface part of Lab 9
// Author:        Your names
// Orig gen date: August 4, 2024
// Date:          Jan 12, 2025
// Goal of this lab: Audio Communication
//

// ----------------------------------------------------------------------------
// Hardware/software assigned to common
// main initialization initializes all modules
//   PD2, Timer5A, ADC1, UART0 implements TExaSdisplay
// ----------------------------------------------------------------------------

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/ADCSWTrigger.h"
#include "../inc/LaunchPad.h"
#include "Joystick.h"

void JoyStick_Init(void){
  ADC_Init89();
  LaunchPad_Init();
}

void JoyStick_In(uint32_t data[2]){
  ADC_In89(data);
}

uint8_t JoyStick_Button(void){
  return LaunchPad_Input();
}
