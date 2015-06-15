#include <stdint.h>

#include "led.h"


	uint8_t Htens = 8;
	uint8_t Hones = 8;
	uint8_t Mtens = 8;
	uint8_t Mones = 8;

GPIO_TypeDef * port_base[] =
{
		GPIOA,
		GPIOB,
		GPIOC,
		GPIOD
};

#define STM32_GPIO_PORTA	0
#define STM32_GPIO_PORTB	1
#define STM32_GPIO_PORTC	2
#define STM32_GPIO_PORTD	3

#define STM32_GPIO(_port, _pin, _mode, _speed)		((_port << 24) | (_pin << 16) | (_mode << 8) | _speed)
#define STM32_GPIO_PORT(_x)			(_x >> 24)
#define STM32_GPIO_PIN(_x)			(1 << ((_x >> 16) & 0xFF))
#define STM32_GPIO_MODE(_x)			((_x >> 8) & 0xFF)
#define STM32_GPIO_SPEED(_x)		(_x & 0xFF)

#define LED_PIN_A	STM32_GPIO(STM32_GPIO_PORTC,  6, GPIO_Mode_Out_PP, GPIO_Speed_2MHz)
#define LED_PIN_B	STM32_GPIO(STM32_GPIO_PORTC,  7, GPIO_Mode_Out_PP, GPIO_Speed_2MHz)
#define LED_PIN_0	STM32_GPIO(STM32_GPIO_PORTB, 15, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define LED_PIN_1	STM32_GPIO(STM32_GPIO_PORTB, 14, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define LED_PIN_2	STM32_GPIO(STM32_GPIO_PORTA,  8, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define LED_PIN_3	STM32_GPIO(STM32_GPIO_PORTA, 11, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define LED_PIN_4	STM32_GPIO(STM32_GPIO_PORTA, 12, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define LED_PIN_5	STM32_GPIO(STM32_GPIO_PORTC, 10, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define LED_PIN_6	STM32_GPIO(STM32_GPIO_PORTC, 11, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define LED_PIN_7	STM32_GPIO(STM32_GPIO_PORTC, 12, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define LED_PIN_8	STM32_GPIO(STM32_GPIO_PORTD, 2, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define LED_PIN_9	STM32_GPIO(STM32_GPIO_PORTB, 13, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define LED_PIN_10	STM32_GPIO(STM32_GPIO_PORTB, 6, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define LED_PIN_11	STM32_GPIO(STM32_GPIO_PORTB, 7, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define LED_PIN_12	STM32_GPIO(STM32_GPIO_PORTB, 8, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define LED_PIN_13	STM32_GPIO(STM32_GPIO_PORTB, 9, GPIO_Mode_Out_OD, GPIO_Speed_2MHz)

uint32_t pins[] =
{
		LED_PIN_0,
		LED_PIN_1,
		LED_PIN_2,
		LED_PIN_3,
		LED_PIN_4,
		LED_PIN_5,
		LED_PIN_6,
		LED_PIN_7,
		LED_PIN_8,
		LED_PIN_9,
		LED_PIN_10,
		LED_PIN_11,
		LED_PIN_12,
		LED_PIN_13,
};

typedef struct segment
{
	uint16_t A[10];
	uint16_t B[10];
}sSegments;

sSegments segments[] =
{		//										A										 |								B													|
		// 0	|   1	|   2	|   3	|   4	|   5	|   6	|   7	|  8	|   9	 |   0		|   1	|   2	|   3	|   4	|   5	|   6	|   7	|  8	|   9	 |
/*D1*/	{{0x00  , 0x00  , 0x06  , 0x06  , 0x00  , 0x00  , 0x00  , 0x00  , 0x06  , 0x00  }, {0x00  , 0x09  , 0x07  , 0x0B  , 0x00  , 0x00  , 0x00  , 0x00  , 0x0F  , 0x00  }},
/*D2*/	{{0x68  , 0x00  , 0x38  , 0x30  , 0x50  , 0x70  , 0x78  , 0x00  , 0x78  , 0x70  }, {0x70  , 0x30  , 0x50  , 0x70  , 0x30  , 0x60  , 0x60  , 0x70  , 0x70  , 0x70  }},
/*D3*/	{{0x380 , 0x300 , 0x180 , 0x380 , 0x300 , 0x280 , 0x280 , 0x380 , 0x380 , 0x380 }, {0x680 , 0x00  , 0x700 , 0x300 , 0x180 , 0x380 , 0x780 , 0x00  , 0x780 , 0x380}},
/*D4*/	{{0x3400, 0x00  , 0x1C00, 0x1800, 0x2800, 0x3800, 0x3C00, 0x0000, 0x3C00, 0x3800}, {0x3800, 0x1800, 0x2800, 0x3800, 0x1800, 0x3000, 0x3000, 0x3800, 0x3800, 0x3800}}
};



void init_gpio(uint32_t pin)
{
	GPIO_InitTypeDef pinType;
	pinType.GPIO_Pin = STM32_GPIO_PIN(pin);
	pinType.GPIO_Mode = STM32_GPIO_MODE(pin);
	pinType.GPIO_Speed = STM32_GPIO_SPEED(pin);

	GPIO_Init(port_base[STM32_GPIO_PORT(pin)], &pinType);
}

void set_gpio(uint32_t pin)
{
	GPIO_SetBits(port_base[STM32_GPIO_PORT(pin)], STM32_GPIO_PIN(pin));
}

void reset_gpio(uint32_t pin)
{
	GPIO_ResetBits(port_base[STM32_GPIO_PORT(pin)], STM32_GPIO_PIN(pin));
}

void initLED()
{
	//Enable GPIOC clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPAEN, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPBEN, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPCEN, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPDEN, ENABLE);

	init_gpio(LED_PIN_A);
	init_gpio(LED_PIN_B);

	for(uint8_t k =0; k < 14; k++)
	{
		init_gpio(pins[k]);
		set_gpio(pins[k]);
	}
}

void led_switch(uint16_t bits)
{
	for(uint8_t k =0; k < 14; k++)
	{
		if(bits & (1 << k))
			reset_gpio(pins[k]);
	}
}

void led_set(uint8_t hours, uint8_t minutes)
{
	Htens = hours/10;
	Hones = hours%10;
	Mtens = minutes/10;
	Mones = minutes%10;
}

void led_animate()
{
	static uint8_t toggle = 0;

	//switch all LED off
	for(uint8_t k =0; k < 14; k++)
	{
		set_gpio(pins[k]);
	}

	if(toggle)
	{
		toggle = 0;
		set_gpio(LED_PIN_A);
		reset_gpio(LED_PIN_B);

		led_switch(segments[0].A[Htens] | segments[1].A[Hones] | segments[2].A[Mtens] | segments[3].A[Mones]);
	}
	else
	{
		toggle = 1;
		reset_gpio(LED_PIN_A);
		set_gpio(LED_PIN_B);


		led_switch(segments[0].B[Htens] | segments[1].B[Hones] | segments[2].B[Mtens] | segments[3].B[Mones]);
	}
}
