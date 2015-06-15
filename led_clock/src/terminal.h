#ifndef _TERMINAL_H
#define _TERMINAL_H
#include <stm32f10x.h>
#include "termStrings.h"

void initTerminal();
bool t_hadInput();

void USART1_IRQHandler(void);

void d_print(uint16_t reg);
void h_print(uint32_t reg);

void t_print(const char* str);
void t_putc(char c);

void showReg(const char* name, uint32_t reg);


#endif //_TERMINAL_H

