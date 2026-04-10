/* Motor.c
 * Jonathan Valvano
 * RSLK v2.0.2
 * June 28, 2024
// Motor (not LM4F120)
// TM4C MSPM0
// PF2  PB4  Motor_PWML, ML+, IN3, PWM M1PWM6
// PF3  PB1  Motor_PWMR, MR+, IN1, PWM M1PWM7
// PA3  PB0  Motor_DIR_L,ML-, IN4, GPIO, 0 means forward, 1 means backward
// PA2  PB16 Motor_DIR_R,MR-, IN2, GPIO, 0 means forward, 1 means backward
 *
 */
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/Motor.h"

#define LEFT_DIR_MASK   0x08
#define RIGHT_DIR_MASK  0x04
#define LEFT_PWM_MASK   0x04
#define RIGHT_PWM_MASK  0x08

static uint16_t ClampDuty(uint32_t duty){
  if(duty < MOTORMIN){
    return 0;
  }
  if(duty > MOTORMAX){
    return MOTORMAX;
  }
  return (uint16_t)duty;
}

static void Motor_SetDuty(uint16_t leftDuty, uint16_t rightDuty){
  if(leftDuty){
    PWM1_3_CMPA_R = MOTOR_PERIOD - leftDuty;
    PWM1_ENABLE_R |= 0x40;
  }else{
    PWM1_ENABLE_R &= ~0x40;
  }
  if(rightDuty){
    PWM1_3_CMPB_R = MOTOR_PERIOD - rightDuty;
    PWM1_ENABLE_R |= 0x80;
  }else{
    PWM1_ENABLE_R &= ~0x80;
  }
}

void Motor_Init(void){
  SYSCTL_RCGCPWM_R |= 0x02;          // PWM1
  SYSCTL_RCGCGPIO_R |= 0x21;         // Port A and F
  while((SYSCTL_PRGPIO_R&0x21) != 0x21){};

  GPIO_PORTA_DIR_R |= LEFT_DIR_MASK | RIGHT_DIR_MASK;
  GPIO_PORTA_AFSEL_R &= ~(LEFT_DIR_MASK | RIGHT_DIR_MASK);
  GPIO_PORTA_DEN_R |= LEFT_DIR_MASK | RIGHT_DIR_MASK;
  GPIO_PORTA_AMSEL_R &= ~(LEFT_DIR_MASK | RIGHT_DIR_MASK);

  GPIO_PORTF_AFSEL_R |= LEFT_PWM_MASK | RIGHT_PWM_MASK;
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFF00FF) | 0x00005500;
  GPIO_PORTF_DEN_R |= LEFT_PWM_MASK | RIGHT_PWM_MASK;
  GPIO_PORTF_AMSEL_R &= ~(LEFT_PWM_MASK | RIGHT_PWM_MASK);

  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;
  SYSCTL_RCC_R = (SYSCTL_RCC_R&~SYSCTL_RCC_PWMDIV_M) | SYSCTL_RCC_PWMDIV_2;

  PWM1_3_CTL_R = 0;
  PWM1_3_GENA_R = 0x000000C8;        // low on load, high on CMPA down
  PWM1_3_GENB_R = 0x00000C08;        // low on load, high on CMPB down
  PWM1_3_LOAD_R = MOTOR_PERIOD - 1;
  PWM1_3_CMPA_R = MOTOR_PERIOD - 1;
  PWM1_3_CMPB_R = MOTOR_PERIOD - 1;
  PWM1_3_CTL_R = 0x01;
  PWM1_ENABLE_R &= ~0xC0;
  GPIO_PORTA_DATA_R &= ~(LEFT_DIR_MASK | RIGHT_DIR_MASK);
}

void Motor_Forward(uint32_t dutyLeft, uint32_t dutyRight){
  GPIO_PORTA_DATA_R &= ~(LEFT_DIR_MASK | RIGHT_DIR_MASK);
  Motor_SetDuty(ClampDuty(dutyLeft), ClampDuty(dutyRight));
}

void Motor_Backward(uint32_t dutyLeft, uint32_t dutyRight){
  GPIO_PORTA_DATA_R |= LEFT_DIR_MASK | RIGHT_DIR_MASK;
  Motor_SetDuty(ClampDuty(dutyLeft), ClampDuty(dutyRight));
}

void Motor_Right(uint32_t dutyLeft, uint32_t dutyRight){
  GPIO_PORTA_DATA_R = (GPIO_PORTA_DATA_R & ~LEFT_DIR_MASK) | RIGHT_DIR_MASK;
  GPIO_PORTA_DATA_R &= ~LEFT_DIR_MASK;
  Motor_SetDuty(ClampDuty(dutyLeft), ClampDuty(dutyRight));
}

void Motor_Left(uint32_t dutyLeft, uint32_t dutyRight){
  GPIO_PORTA_DATA_R = (GPIO_PORTA_DATA_R & ~RIGHT_DIR_MASK) | LEFT_DIR_MASK;
  GPIO_PORTA_DATA_R &= ~RIGHT_DIR_MASK;
  Motor_SetDuty(ClampDuty(dutyLeft), ClampDuty(dutyRight));
}



