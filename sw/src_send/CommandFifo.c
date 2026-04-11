// CommandFifo.c — two-index FIFO (Valvano-style), uint8_t payloads

#include <stdint.h>
#include "CommandFifo.h"

static volatile uint32_t PutI;
static volatile uint32_t GetI;
static uint8_t Buf[CMD_FIFO_SIZE];

void CommandFifo_Init(void){
  PutI = 0;
  GetI = 0;
}

int CommandFifo_Put(uint8_t cmd){
  if(((PutI - GetI) & ~(CMD_FIFO_SIZE - 1u)) != 0u){
    return 0;
  }
  Buf[PutI & (CMD_FIFO_SIZE - 1u)] = cmd;
  PutI++;
  return 1;
}

int CommandFifo_Get(uint8_t *cmd){
  if(PutI == GetI){
    return 0;
  }
  *cmd = Buf[GetI & (CMD_FIFO_SIZE - 1u)];
  GetI++;
  return 1;
}

uint32_t CommandFifo_Size(void){
  return (uint32_t)(PutI - GetI);
}
