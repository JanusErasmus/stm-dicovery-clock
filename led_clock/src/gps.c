#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_usart.h>
#include <misc.h>

#include "led.h"
#include "terminal.h"

//define the terminal that will be used
//#define TERM_UART1
#define TERM_UART2
//#define TERM_UART3_REMAPPED

#define GPS_BUFF_LEN 256

char g_buff[GPS_BUFF_LEN];
uint32_t g_buffLen = 0;

USART_TypeDef* termGPS = 0;

uint8_t gpsHour = 0;
uint8_t gpsMinute = 0;

void initGPS()
{
#ifdef TERM_UART1
	termGPS = USART1;
	//enable USART 1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN, ENABLE);
	//Enable GPIOA clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPAEN, ENABLE);
#elif defined TERM_UART2
	termGPS = USART2;
	//enable USART 2 clock
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN, ENABLE);
	//Enable GPIOA clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPAEN, ENABLE);
#elif TERM_UART3_REMAPPED
	termGPS = USART3;
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


	USART_Cmd(termGPS, ENABLE);

	USART_InitTypeDef usartSetup;
	usartSetup.USART_BaudRate = 4800;
	usartSetup.USART_WordLength = USART_WordLength_8b;
	usartSetup.USART_StopBits = USART_StopBits_1;
	usartSetup.USART_Parity = USART_Parity_No;
	usartSetup.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usartSetup.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(termGPS, &usartSetup);


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
	termInt.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&termInt);

	USART_ITConfig(termGPS, USART_IT_RXNE, ENABLE);

	GPIO_InitTypeDef pin;
	pin.GPIO_Mode = GPIO_Mode_Out_PP;
	pin.GPIO_Speed = GPIO_Speed_2MHz;
	pin.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOC, &pin);
	GPIO_SetBits(GPIOC, GPIO_Pin_9);


	pin.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOC, &pin);
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);
}

bool gps_getTime(uint8_t *hour, uint8_t *min)
{
	if((23 <= gpsHour) || (gpsHour == 0))
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		return 0;
	}

	*hour = gpsHour + 2;
	*min = gpsMinute;

	GPIO_SetBits(GPIOC, GPIO_Pin_8);

	return 1;
}

void handleNMEA(char * nmeaStr)
{
//	t_print(nmeaStr);
//	t_print("\n");
	char *char_ptr[4];
	char_ptr[0] = 0;
	char_ptr[1] = 0;
	char_ptr[2] = 0;
	char_ptr[3] = 0;

	int index = 0;
	while(*nmeaStr)
	{
		if(*nmeaStr == ',')
		{
			*nmeaStr = 0;
			char_ptr[index] = &nmeaStr[1];

			if(++index >= 4)
				break;
		}

		nmeaStr++;
	}

	gpsHour = index;

	if(char_ptr[0] && t_strlen(char_ptr[0]))
	{
		char_ptr[0][4] = 0;
		gpsMinute = t_atoi(&char_ptr[0][2]);

		char_ptr[0][2] = 0;
		gpsHour = t_atoi(char_ptr[0]);

		static uint8_t flag = 0;
		if(flag)
			GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		else
			GPIO_SetBits(GPIOC, GPIO_Pin_9);

		flag = !flag;
	}


//	d_print(gpsHour);
//	t_print(":");
//	d_print(gpsMinute);
//	t_print("\n");

}

#ifdef TERM_UART1
void USART1_IRQHandler(void)
#elif defined TERM_UART2
void USART2_IRQHandler(void)
#elif defined TERM_UART3_REMAPPED
void USART3_IRQHandler(void)
#endif
{
	uint32_t data =	termGPS->DR;
	g_buff[g_buffLen] = data;

	if((g_buff[g_buffLen] == '\n') || (g_buff[g_buffLen - 1] == '\r'))
	{
		g_buff[g_buffLen] = 0;
		g_buffLen = GPS_BUFF_LEN;

		if(!t_strncmp(g_buff, "$GPGGA", 6))
		{
			handleNMEA(g_buff);
		}
	}

	if(++g_buffLen >= GPS_BUFF_LEN)
		g_buffLen = 0;

}


