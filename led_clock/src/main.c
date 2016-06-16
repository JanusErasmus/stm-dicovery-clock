#include <stm32f10x.h>
#include <misc.h>
#include <stm32f10x_iwdg.h>

#include "terminal.h"
#include "led.h"
#include "timer2.h"
#include "rtc.h"
#include "gps.h"

void delay();

uint8_t feedingFlag  = 0;

typedef int size_t;
void* memcpy( void * destination, const void * source, size_t num );

void hourAction(uint8_t hour, uint8_t minute)
{
	//t_print("Time\n");
	uint8_t h = 0, m = 0;

	if(gps_getTime(&h, &m))
		rtc_set(h, m);
}

void halfMinuteAction(uint8_t hour, uint8_t minute)
{
	uint8_t h = 0, m = 0;

	if(!gps_getTime(&h, &m))
	{
		rtc_getTime(&h,&m);
	}

	led_set(h, m);
}

void startWatchdog()
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(6); //26214.4ms
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);

	IWDG_Enable();
}

/// Main function.  Called by the startup code.
int main(void)
{

	startWatchdog();

	//when interrupts are going to be used
	NVIC_Configuration();

	initTerminal();

	t_print("LED Clock - Build: ");
	t_print(__DATE__);
	t_print(" ");
	t_print(__TIME__);
	t_print("\n");


	initLED();
	initGPS();
	timer2Init();
	initRTC();

	rtc_setHourAlarm(1, hourAction);
	rtc_setSecondAlarm(30, halfMinuteAction);

	t_print(">>");

	GPIO_InitTypeDef pin;
	pin.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	pin.GPIO_Speed = GPIO_Speed_2MHz;

	pin.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &pin);
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);

	uint8_t h = 0, m = 0;
//	for(uint8_t k = 0; k < 10; k++)
//	{
//		led_set(h, m);
		delay();
//		h+=11;
//		m+=11;
//	}

		if(!gps_getTime(&h, &m))
		{
			rtc_getTime(&h,&m);
		}

	led_set(h, m);

	while(1)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0))
		{
			t_print("User pushing button\n");
		}
		delay();
	}

	return 0;
}

void* memcpy( void * destination, const void * source, int num )
{
	uint8_t* d = destination;
	const uint8_t* s = source;

	for(int k= 0; k < num; k++)
		d[k] = s[k];

	return 0;
}

void delay()
{
	volatile int t = 2500000;

	while(t >0 )
	{
		t--;
	}
}




