
#include "main.h"

static uint8_t Htens = 8;
static uint8_t Hones = 8;
static uint8_t Mtens = 8;
static uint8_t Mones = 8;

void led_set_time(int hours, int minutes)
{
	Htens = hours/10;
	Hones = hours%10;
	Mtens = minutes/10;
	Mones = minutes%10;
}

typedef struct
{
	uint32_t pin;
	GPIO_TypeDef *port;
}led_pin_t;

static const led_pin_t pins[] =
{
		{LED_PIN_0_Pin,  LED_PIN_0_GPIO_Port},
		{LED_PIN_1_Pin,  LED_PIN_1_GPIO_Port},
		{LED_PIN_2_Pin,  LED_PIN_2_GPIO_Port},
		{LED_PIN_3_Pin,  LED_PIN_3_GPIO_Port},
		{LED_PIN_4_Pin,  LED_PIN_4_GPIO_Port},
		{LED_PIN_5_Pin,  LED_PIN_5_GPIO_Port},
		{LED_PIN_6_Pin,  LED_PIN_6_GPIO_Port},
		{LED_PIN_7_Pin,  LED_PIN_7_GPIO_Port},
		{LED_PIN_8_Pin,  LED_PIN_8_GPIO_Port},
		{LED_PIN_9_Pin,  LED_PIN_9_GPIO_Port},
		{LED_PIN_10_Pin, LED_PIN_10_GPIO_Port},
		{LED_PIN_11_Pin, LED_PIN_11_GPIO_Port},
		{LED_PIN_12_Pin, LED_PIN_12_GPIO_Port},
		{LED_PIN_13_Pin, LED_PIN_13_GPIO_Port}
};

typedef struct segment
{
	uint16_t A[10];
	uint16_t B[10];
}sSegments;

static const sSegments segments[] =
{		//										A										 |								B													|
		// 0	|   1	|   2	|   3	|   4	|   5	|   6	|   7	|  8	|   9	 |   0		|   1	|   2	|   3	|   4	|   5	|   6	|   7	|  8	|   9	 |
/*D1*/	{{0x00  , 0x00  , 0x06  , 0x06  , 0x00  , 0x00  , 0x00  , 0x00  , 0x06  , 0x00  }, {0x00  , 0x09  , 0x07  , 0x0B  , 0x00  , 0x00  , 0x00  , 0x00  , 0x0F  , 0x00  }},
/*D2*/	{{0x68  , 0x00  , 0x38  , 0x30  , 0x50  , 0x70  , 0x78  , 0x00  , 0x78  , 0x70  }, {0x70  , 0x30  , 0x50  , 0x70  , 0x30  , 0x60  , 0x60  , 0x70  , 0x70  , 0x70  }},
/*D3*/	{{0x380 , 0x300 , 0x180 , 0x380 , 0x300 , 0x280 , 0x280 , 0x380 , 0x380 , 0x380 }, {0x680 , 0x00  , 0x700 , 0x300 , 0x180 , 0x380 , 0x780 , 0x00  , 0x780 , 0x380}},
/*D4*/	{{0x3400, 0x00  , 0x1C00, 0x1800, 0x2800, 0x3800, 0x3C00, 0x0000, 0x3C00, 0x3800}, {0x3800, 0x1800, 0x2800, 0x3800, 0x1800, 0x3000, 0x3000, 0x3800, 0x3800, 0x3800}}
};

static void set_gpio(uint32_t pin)
{
	HAL_GPIO_WritePin(pins[pin].port, pins[pin].pin, GPIO_PIN_SET);
}

static void reset_gpio(uint32_t pin)
{
	HAL_GPIO_WritePin(pins[pin].port, pins[pin].pin, GPIO_PIN_RESET);
}

static void led_switch(uint16_t bits)
{
	for(uint8_t k =0; k < 14; k++)
	{
		if(bits & (1 << k))
			reset_gpio(k);
		else
			set_gpio(k);
	}
}

void led_animate()
{
	static uint8_t toggle = 0;

	HAL_GPIO_WritePin(LED_PIN_B_GPIO_Port, LED_PIN_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PIN_A_GPIO_Port, LED_PIN_A_Pin, GPIO_PIN_RESET);
//	//switch all LED off
//	for(uint8_t k =0; k < 14; k++)
//	{
//		set_gpio(k);
//	}

	if(toggle)
	{
		led_switch(segments[0].A[Htens] | segments[1].A[Hones] | segments[2].A[Mtens] | segments[3].A[Mones]);
		toggle = 0;
		HAL_GPIO_WritePin(LED_PIN_A_GPIO_Port, LED_PIN_A_Pin, GPIO_PIN_SET);
	}
	else
	{
		led_switch(segments[0].B[Htens] | segments[1].B[Hones] | segments[2].B[Mtens] | segments[3].B[Mones]);
		toggle = 1;
		HAL_GPIO_WritePin(LED_PIN_B_GPIO_Port, LED_PIN_B_Pin, GPIO_PIN_SET);
	}
}
