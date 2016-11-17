#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_usart.h>
#include <misc.h>

#include "terminal.h"
#include "termHandler.h"

#define DISABLE_TX

//define the terminal that will be used
#define TERM_UART1
//#define TERM_UART2
//#define TERM_UART3_REMAPPED

#define TERM_BUFF_LEN 64
#define TERM_ECHO
#define TERM_MAP_CR_CRNL
//#define TERM_MAP_NL_CRNL

void handleTerm();

char t_buff[TERM_BUFF_LEN];
uint8_t t_buffLen = 0;

bool t_InputFlag;

USART_TypeDef* termUART = 0;

void initTerminal()
{
#ifdef TERM_UART1
	termUART = USART1;
	//enable USART 1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN, ENABLE);
	//Enable GPIOA clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPAEN, ENABLE);
#elif defined TERM_UART2
	termUART = USART2;
	//enable USART 2 clock
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN, ENABLE);
	//Enable GPIOA clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPAEN, ENABLE);
#elif TERM_UART3_REMAPPED
	termUART = USART3;
	//enable USART 3 clock
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART3EN, ENABLE);
	//Enable GPIOC clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPCEN, ENABLE);
#else
#warning You need to define a terminal UART
#endif


	//enable Alternate function reg
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN, ENABLE);

	GPIO_InitTypeDef txPin;
	GPIO_InitTypeDef rxPin;

#ifdef TERM_UART1
	txPin.GPIO_Pin = GPIO_Pin_9;
	txPin.GPIO_Mode = GPIO_Mode_AF_PP;
	txPin.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &txPin);

	rxPin.GPIO_Pin = GPIO_Pin_10;
	rxPin.GPIO_Mode = GPIO_Mode_IPU;
	rxPin.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &rxPin);

#elif defined TERM_UART2
	txPin.GPIO_Pin = GPIO_Pin_2;
	txPin.GPIO_Mode = GPIO_Mode_AF_PP;
	txPin.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &txPin);

	rxPin.GPIO_Pin = GPIO_Pin_3;
	rxPin.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	rxPin.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &rxPin);

#elif defined TERM_UART3_REMAPPED
	txPin.GPIO_Pin = GPIO_Pin_10;
	txPin.GPIO_Mode = GPIO_Mode_AF_PP;
	txPin.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &txPin);

	rxPin.GPIO_Pin = GPIO_Pin_11;
	rxPin.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	rxPin.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &rxPin);

	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
#endif


	USART_Cmd(termUART, ENABLE);

	USART_InitTypeDef usartSetup;
	usartSetup.USART_BaudRate = 115200;
	usartSetup.USART_WordLength = USART_WordLength_8b;
	usartSetup.USART_StopBits = USART_StopBits_1;
	usartSetup.USART_Parity = USART_Parity_No;
	usartSetup.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usartSetup.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(termUART, &usartSetup);


	//Map UART interrupt
	NVIC_InitTypeDef termInt;
#ifdef TERM_UART1
	termInt.NVIC_IRQChannel = USART1_IRQn;
#elif defined TERM_UART2
	termInt.NVIC_IRQChannel = USART2_IRQn;
#elif defined TERM_UART3_REMAPPED
	termInt.NVIC_IRQChannel = USART3_IRQn;
#endif

	termInt.NVIC_IRQChannelCmd = ENABLE;
	termInt.NVIC_IRQChannelPreemptionPriority = 1;
	termInt.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&termInt);

	USART_ITConfig(termUART, USART_IT_RXNE, ENABLE);
}

bool t_hadInput()
{
	return t_InputFlag;
}



void handleTerm()
{
	t_buff[t_buffLen] = 0;

	if(!t_Handler(t_buff) && t_buffLen > 2)
	{
		t_print("Unknown command \"");
		t_print(t_buff);
		t_print("\". Type h for list of commands\n");
	}

		t_print(">>");

	t_buffLen = 0;
}

void t_print(const char* str)
{
#ifdef DISABLE_TX
	return;
#endif
	uint16_t len = t_strlen(str);

	for(int k = 0; k < len; k++)
	{
		if(str[k] == '\n')
			t_putc('\r');

		t_putc(str[k]);
	}
}

void t_putc(char c)
{
	USART_SendData(termUART, c);
	while(!USART_GetFlagStatus(termUART, USART_FLAG_TXE));
}

void showReg(const char* name, uint32_t reg)
{
#ifdef DISABLE_TX
	return;
#endif
	t_print(name);
	t_print(": ");
	h_print(reg);
}

void h_print(uint32_t reg)
{
#ifdef DISABLE_TX
	return;
#endif
	t_print("0x");
	uint8_t digit = (reg >> 28) & 0x0F;
	d_print(digit);


	digit = (reg >> 24) & 0x0F;
	d_print(digit);

	digit = (reg >> 20) & 0x0F;
	d_print(digit);

	digit = (reg >> 16) & 0x0F;
	d_print(digit);

	digit = (reg >> 12) & 0x0F;
	d_print(digit);

	digit = (reg >> 8) & 0x0F;
	d_print(digit);

	digit = (reg >> 4) & 0x0F;
	d_print(digit);

	digit = (reg >> 0) & 0x0F;
	d_print(digit);

	t_print("\n");
}

void d_print(uint16_t reg)
{

#ifdef DISABLE_TX
	return;
#endif

	uint8_t thFlag = 0, hFlag = 0, tFlag = 0;

	uint8_t thousends = reg/1000;
	reg = reg%1000;

	uint8_t hundreds = reg/100;
	reg = reg%100;

	uint8_t tens = reg/10;
	reg = reg%10;

	uint8_t ones = reg;

	if(thousends)
	{
		thFlag = 1;
		hFlag = 1;
		tFlag = 1;
	}

	if(hundreds)
	{
		hFlag = 1;
		tFlag = 1;
	}

	if(tens)
		tFlag = 1;

	thousends += 0x30;
	if(thousends > 0x39)
	{
		thousends = '.';
	}


	hundreds += 0x30;
	if(hundreds > 0x39)
	{
		hundreds = '.';
	}


	tens += 0x30;
	if(tens > 0x39)
	{
		tens = '.';
	}

	ones += 0x30;
	if(ones > 0x39)
	{
		ones = '.';
	}

	if(thFlag)
		t_putc(thousends);

	if(hFlag)
		t_putc(hundreds);

	if(tFlag)
		t_putc(tens);

	t_putc(ones);
}

#ifdef TERM_UART1
void USART1_IRQHandler(void)
#elif defined TERM_UART2
void USART2_IRQHandler(void)
#elif defined TERM_UART3_REMAPPED
void USART3_IRQHandler(void)
#endif
{
	t_InputFlag = 1;

	uint32_t data =	termUART->DR;

#ifdef TERM_ECHO
	t_putc(data);
#endif

#ifdef TERM_MAP_CR_CRNL
	if(data == '\r')
	{
		t_buff[t_buffLen++] = data;
		t_buff[t_buffLen++] = '\n';
		t_putc('\n');
	}else
#elif defined TERM_MAP_NL_CRNL
	if(data == '\n')
	{
		t_buff[t_buffLen++] = '\r';
		t_buff[t_buffLen++] = data;
		t_putc('\r');
	}else
#endif
		t_buff[t_buffLen++] = data;

	if(t_buff[t_buffLen - 1] == '\n')
	{
		handleTerm();
	}


	if(t_buffLen > TERM_BUFF_LEN)
		handleTerm();

}


