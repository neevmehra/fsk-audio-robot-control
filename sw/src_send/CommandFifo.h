// CommandFifo.h
// Lock-free uint8_t ring (power-of-2 size) for joystick → encoder queue.
// Producer: Timer0A ISR. Consumer: SysTick ISR.

#ifndef __COMMANDFIFO_H__
#define __COMMANDFIFO_H__

#include <stdint.h>

#define CMD_FIFO_SIZE 16u

void     CommandFifo_Init(void);
int      CommandFifo_Put(uint8_t cmd);
int      CommandFifo_Get(uint8_t *cmd);
uint32_t CommandFifo_Size(void);

#endif
