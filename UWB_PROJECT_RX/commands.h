/**
  ******************************************************************************
  * File Name          : commands.h
  * Description        : This file contains lists of commands which can be used.
	* Software Engineer	 : THIHA KYAW
  ******************************************************************************
  */
	
#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdio.h>                      /* Standard I/O .h-file               */
#include <ctype.h>                      /* Character functions                */
#include <string.h>                     /* String and memory functions        */
#include "stm32f7xx_hal.h"

/* Command definitions structure. */

#define CMD_COUNT   (sizeof (cmd) / sizeof (cmd[0]))
	
typedef struct scmd {
  char val[8];
  void (*func)(char *par);
} SCMD;

extern SCMD cmd[];

enum {BACKSPACE = 0x08,
      LF        = 0x0A,
      CR        = 0x0D,
      CNTLQ     = 0x11,
      CNTLS     = 0x13,
      ESC       = 0x1B,
      DEL       = 0x7F };

void execute_CMD(char *par);
HAL_StatusTypeDef uart_sendmsg(char *msg, uint16_t length); 

#endif /* __FILE_COMMANDS_H */
/************************ (C) COPYRIGHT THIHA KYAW************************/
