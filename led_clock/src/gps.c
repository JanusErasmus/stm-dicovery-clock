#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_usart.h>
#include <misc.h>

#include "led.h"
#include "terminal.h"


extern void delay();
void longdelay()
{
	uint8_t cnt = 2;
	while(cnt--)
		delay();
}

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

uint8_t gpsON = 0;
uint8_t gpsOK = 0;

void enableUART()
{
	g_buffLen = 0;
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

	USART_Cmd(termGPS, ENABLE);

	USART_InitTypeDef usartSetup;
	usartSetup.USART_BaudRate = 4800;
	usartSetup.USART_WordLength = USART_WordLength_8b;
	usartSetup.USART_StopBits = USART_StopBits_1;
	usartSetup.USART_Parity = USART_Parity_No;
	usartSetup.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usartSetup.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(termGPS, &usartSetup);

	USART_ITConfig(termGPS, USART_IT_RXNE, ENABLE);
}

void disableUART()
{
#ifdef TERM_UART1
	termGPS = USART1;
	//enable USART 1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN, DISABLE);
	//Enable GPIOA clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPAEN, DISABLE);
#elif defined TERM_UART2
	termGPS = USART2;
	//enable USART 2 clock
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN, DISABLE);
	//Enable GPIOA clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPAEN, DISABLE);
#elif TERM_UART3_REMAPPED
	termGPS = USART3;
	//enable USART 3 clock
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART3EN, DISABLE);
	//Enable GPIOC clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPCEN, DISABLE);
#else
#warning You need to define a terminal UART
#endif
}

void initGPS()
{
	enableUART();

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

	GPIO_InitTypeDef pin;
	pin.GPIO_Mode = GPIO_Mode_Out_PP;
	pin.GPIO_Speed = GPIO_Speed_2MHz;
	pin.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOC, &pin);
	GPIO_ResetBits(GPIOC, GPIO_Pin_9);


	pin.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOC, &pin);
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);

	pin.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOC, &pin);
	GPIO_ResetBits(GPIOC, GPIO_Pin_0);

	delay();
}

void gps_igt()
{
	disableUART();
	GPIO_SetBits(GPIOC, GPIO_Pin_0);
	delay();
	GPIO_ResetBits(GPIOC, GPIO_Pin_0);
	enableUART();
}


bool gps_getTime(uint8_t *hour, uint8_t *min)
{
	uint8_t H =  gpsHour + 2;
	if(H >= 24)
		H -= 24;

	*hour = H;
	*min = gpsMinute;

	if(!gpsON)
	{
		gps_igt();
	}
	gpsON = 0;

	if(!gpsOK)
		return 0;

	gpsOK = 0;
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);

	return 1;
}

uint8_t parseNMEA(char *string, char ** argv, uint8_t argc)
{
	int index = 0;
	while(*string)
	{
		if(*string == ',')
		{
			*string = 0;
			argv[index] = &string[1];

			if(++index >= argc)
				break;
		}

		string++;
	}

	return index;
}

void handleTime(char * gpgga)
{
	char *argv[7];
	uint8_t argc = parseNMEA(gpgga, argv, 7);

	if(argc < 5)
		return;

	argv[0][4] = 0;
	gpsMinute = t_atoi(&argv[0][2]);

	argv[0][2] = 0;
	gpsHour = t_atoi(argv[0]);

	static uint8_t flag = 0;
	if(flag)
		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
	else
		GPIO_SetBits(GPIOC, GPIO_Pin_9);

	flag = !flag;

	d_print(gpsHour);
	t_print(":");
	d_print(gpsMinute);
	t_print("\n");
}

void handleSatelites(char * gpgsv)
{
	char *argv[9];
	uint8_t argc = parseNMEA(gpgsv, argv, 9);

	if(argc < 8)
		return;

//	t_print(argv[2]);  //Satellites  in view
//	t_print(" ");
//	t_print(argv[3]);	//PRN number
//	t_print(" ");
	t_print(argv[4]);	//elivation
//	t_print(" ");
//	t_print(argv[5]);	//azimuth
//	t_print(" ");
//	t_print(argv[6]);	//SNR
	t_print("\n");
	if( t_atoi(argv[4]) > 1)
	{
		gpsOK = 1;
		GPIO_SetBits(GPIOC, GPIO_Pin_8);
	}
	else
	{
		gpsOK = 0;
		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
	}
}

void handleNMEA(char * nmeaStr)
{
	if(!t_strncmp(nmeaStr, "$GPGGA", 6))
	{
		handleTime(nmeaStr);
	}

	if(!t_strncmp(nmeaStr, "$GPGSV", 6))
	{
		handleSatelites(nmeaStr);
	}
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
		gpsON = 1;

//		g_buff[20] = 0;
//		t_print(g_buff);
//		t_print("\n");
//		return;

		handleNMEA(g_buff);
	}

	if(++g_buffLen >= GPS_BUFF_LEN)
		g_buffLen = 0;

}


