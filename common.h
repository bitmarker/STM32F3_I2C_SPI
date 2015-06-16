#ifndef __COMMON_H
#define __COMMON_H

#include "stm32f30x.h"
#include "stm32f30x_it.h"
#include "stm32f30x_conf.h"

typedef ErrorStatus RESULT;

#define CALL_FUNC(func, args) if(func != 0) { func(args); }
#define CALL_FUNC_INL(func, args) (func == 0) ? (asm("NOP")) : (func(args))

typedef void (*WAIT_MS_FUNC)(uint16_t count);

#define DEBUG

#define EXIT(message)    \
      printf("-- Crash --\n"); printf(message); printf("\n"); \
      while(1){ asm("NOP"); }

#define TRACE(message)    \
      printf(message); printf("\n");

#endif