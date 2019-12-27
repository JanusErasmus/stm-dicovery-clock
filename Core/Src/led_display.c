
#include "main.h"

static uint8_t Htens = 8;
static uint8_t Hones = 8;
static uint8_t Mtens = 8;
static uint8_t Mones = 8;
static uint8_t seg_0 = 8;
static uint8_t seg_1 = 8;
static uint8_t seg_2 = 8;

void led_set_time(int hours, int minutes)
{
	Htens = hours/10;
	Hones = hours%10;
	Mtens = minutes/10;
	Mones = minutes%10;
}

void led_set_temperature(double temp)
{
	//printf("Set Temperature %f\n", temp);
	seg_0 = (int)(temp * 10) % 10;
	seg_1 = (int)temp % 10;
	seg_2 = temp / 10;

	if(seg_0 > 9)
		seg_0 = 9;

	if(seg_1 > 9)
		seg_1 = 9;

	if(seg_2 > 9)
		seg_2 = 9;

	//printf("Seg0 %d\n", seg_0);
	//printf("Seg1 %d\n", seg_1);
	//printf("Seg2 %d\n", seg_2);

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

static const uint8_t SegmentNumbers[] =
{
		0b11100111, //0
		0b10000010, //1
		0b11010101, //2
		0b11010110, //3
		0b10110010, //4
		0b01110110, //5
		0b01110111, //6
		0b10000110, //7
		0b11110111, //8
		0b11110110, //9
		0b11010111	//E
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
	static int toggle = 0;

	HAL_GPIO_WritePin(LED_PIN_B_GPIO_Port, LED_PIN_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PIN_A_GPIO_Port, LED_PIN_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_SEG_0_GPIO_Port, LED_SEG_0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_SEG_1_GPIO_Port, LED_SEG_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_SEG_2_GPIO_Port, LED_SEG_2_Pin, GPIO_PIN_RESET);

	switch(toggle)
	{
	case 0: //A part of clock
		led_switch(segments[0].A[Htens] | segments[1].A[Hones] | segments[2].A[Mtens] | segments[3].A[Mones]);
		HAL_GPIO_WritePin(LED_PIN_A_GPIO_Port, LED_PIN_A_Pin, GPIO_PIN_SET);
		break;
	case 1: //B part of clock
		led_switch(segments[0].B[Htens] | segments[1].B[Hones] | segments[2].B[Mtens] | segments[3].B[Mones]);
		HAL_GPIO_WritePin(LED_PIN_B_GPIO_Port, LED_PIN_B_Pin, GPIO_PIN_SET);
		break;
	case 2: //digit 0 of temperature
		led_switch(SegmentNumbers[seg_0]);
		HAL_GPIO_WritePin(LED_SEG_0_GPIO_Port, LED_SEG_0_Pin, GPIO_PIN_SET);

		break;
	case 3: //digit 1 of temperature
		led_switch(SegmentNumbers[seg_1] | 0x08);
		HAL_GPIO_WritePin(LED_SEG_1_GPIO_Port, LED_SEG_1_Pin, GPIO_PIN_SET);
		break;
	case 4: //digit 2 of temperature
		led_switch(SegmentNumbers[seg_2]);
		HAL_GPIO_WritePin(LED_SEG_2_GPIO_Port, LED_SEG_2_Pin, GPIO_PIN_SET);
		toggle = -1;
		break;
	}

	toggle++;
}
