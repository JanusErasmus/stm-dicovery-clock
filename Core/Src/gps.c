#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"

#define RX_BUFF_LEN 512

static int _hours = 0;
static int _minutes = 0;
static uint8_t raw_bytes[RX_BUFF_LEN];
static int head = 0;
static int tail = 0;
static int sentence_len = 0;
static char sentence[RX_BUFF_LEN];
static uint32_t last_sentence = 0;
int gps_ok = 0;

void gps_handle_byte(uint8_t byte)
{
	raw_bytes[head] = byte;
	head = (head + 1) % RX_BUFF_LEN;
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

static void handleTime(char * gpgga)
{
	char *argv[7];
	uint8_t argc = parseNMEA(gpgga, argv, 7);

	if(argc < 5)
		return;

	argv[0][6] = 0;
	int gpsSecond = atoi(&argv[0][4]);

	argv[0][4] = 0;
	_minutes = atoi(&argv[0][2]);

	argv[0][2] = 0;
	_hours = atoi(argv[0]);

//	static uint8_t flag = 0;
//	if(flag)
//		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
//	else
//		GPIO_SetBits(GPIOC, GPIO_Pin_9);
//	flag = !flag;

	printf("GPS: %02d:%02d:%02d\n", _hours, _minutes, gpsSecond);
	HAL_GPIO_TogglePin(GPIOC, LD3_Pin);
	last_sentence = HAL_GetTick();
}


void handleSatelites(char * gpgsv)
{
	char *argv[9];
	uint8_t argc = parseNMEA(gpgsv, argv, 9);

	if(argc < 8)
		return;

//	printf(argv[2]);  //Satellites  in view
//	printf(" ");
//	printf(argv[3]);	//PRN number
//	printf(" ");
	printf("E: %s\n", argv[4]);	//elivation
//	printf(" ");
//	printf(argv[5]);	//azimuth
//	printf(" ");
//	printf(argv[6]);	//SNR

	if( atoi(argv[4]) > 1)
	{
		gps_ok = 1;
		HAL_GPIO_WritePin(GPIOC, LD4_Pin, GPIO_PIN_SET);
	}
	else
	{
		gps_ok = 0;
		HAL_GPIO_WritePin(GPIOC, LD4_Pin, GPIO_PIN_RESET);
	}
}

static void handle_sentence(char *nmea)
{
	//printf("GPS: %s\n", nmea);

	if(!strncmp(nmea, "$GPGGA", 6))
	{
		//printf("T: %s\n", nmea);
		handleTime(nmea);
	}

	if(!strncmp(nmea, "$GPGSV", 6))
	{
		//printf("S: %s\n", nmea);
		handleSatelites(nmea);
	}
}

static void gps_igt()
{
	  HAL_GPIO_WritePin(GPS_IGT_GPIO_Port, GPS_IGT_Pin, GPIO_PIN_SET);
	  HAL_Delay(800);
	  HAL_GPIO_WritePin(GPS_IGT_GPIO_Port, GPS_IGT_Pin, GPIO_PIN_RESET);
}


void debug_gps(uint8_t argc, char **argv)
{
	printf("GPS: Toggle IGT\n");
	gps_igt();
}

void gps_run()
{
	while(head != tail)
	{
		sentence[sentence_len] = raw_bytes[tail];
		if(raw_bytes[tail] == '\n')
		{
			sentence[sentence_len] = 0;
			sentence[sentence_len - 1] = 0;
			sentence_len = -1;
			handle_sentence(sentence);
		}

		tail = (tail + 1) % RX_BUFF_LEN;

		if(++sentence_len > (RX_BUFF_LEN - 2))
			sentence_len = 0;
	}

	//when we have not received any sentences for the last 30s toggle GPS IGT pin
	uint32_t tick = HAL_GetTick();
	if((tick > 30000) && last_sentence < (tick - 30000))
	{
		last_sentence = tick;
		printf("GPS: Toggle IGT\n");
		gps_igt();
	}
}

int gps_get_time(int *hours, int *minutes)
{
	if(!gps_ok)
		return 0;

	gps_ok = 0;
	HAL_GPIO_WritePin(GPIOC, LD4_Pin, GPIO_PIN_RESET);

	*hours = _hours;
	*minutes = _minutes;
	return 1;
}
