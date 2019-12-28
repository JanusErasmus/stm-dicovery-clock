#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"

#define RX_BUFF_LEN 512

static int _fix = 0;
static int _sv = 0;
static int _hours = 0;
static int _minutes = 0;
static int _seconds = 0;
static uint8_t raw_bytes[RX_BUFF_LEN];
static int head = 0;
static int tail = 0;
static int sentence_len = 0;
static char sentence[RX_BUFF_LEN];
static uint32_t last_sentence = 0;
int gps_ok = 0;
int gps_lock = 0;

void gps_handle_byte(uint8_t byte)
{
	raw_bytes[head] = byte;
	head = (head + 1) % RX_BUFF_LEN;
}

static uint8_t parseNMEA(char *string, char argv[32][16], uint8_t argc)
{
	int index = 0;
	char *arg = string;
	char *ptr = strchr(arg, ',');
	while(ptr)
	{
		//replace the found comma with a null
		*ptr = 0;
		strncpy(argv[index++], arg, 16);
		arg = ptr + 1;
		ptr = strchr(arg, ',');

		if(index > argc)
			return index;
	}
	return index;
}

static void handleTime(char * gpgga)
{
	char argv[32][16] = {0};
	uint8_t argc = parseNMEA(gpgga, argv, 32);
	if(argc < 7)
		return;

	_fix = atoi(argv[6]);
	_sv = atoi(argv[7]);
	printf("GPS: %s FIX: %d SV: %d\n", argv[1], _fix, _sv);


	argv[1][6] = 0;
	_seconds = atoi(&argv[1][4]);

	argv[1][4] = 0;
	_minutes = atoi(&argv[1][2]);

	argv[1][2] = 0;
	_hours = atoi(argv[1]);

	printf(" %02d:%02d:%02d\n", _hours, _minutes, _seconds);

	HAL_GPIO_TogglePin(GPIOC, LD3_Pin);
	last_sentence = HAL_GetTick();
}


void handleSatelites(char * gpgsv)
{
	char argv[32][16] = {0};
	uint8_t argc = parseNMEA(gpgsv, argv, 32);

	if(argc < 4)
		return;

	int total = atoi(argv[1]);
	int msg_id = atoi(argv[2]);
	int sv = atoi(argv[3]);

	//Print header on first message
	if(msg_id == 1)
	{
		gps_ok = 0;
		printf("GPS SV[%d] Msg[%d]:\n          E     A     S\n", sv, total);
	}

	if(sv == 0)
		return;

	int index = 4;
	while(index < argc)
	{
		int value = atoi(argv[index++]);
		printf(" [% 3d] ", value); //SV PRN number
		int elevation = atoi(argv[index++]);
		printf("% 5d ", elevation); //Elevation
		int azimuth = atoi(argv[index++]);
		printf("% 5d ", azimuth); //Azimuth
		int snr = atoi(argv[index++]);
		printf("% 5d\n", snr); //SNR

		//when all values can be filled in, we probably have valid time
		if(snr && azimuth && elevation)
			gps_ok = 1;
	}

	//update gps_ok state
	if(msg_id == total)
	{
		gps_lock = gps_ok;
		if(gps_lock)
			HAL_GPIO_WritePin(GPIOC, LD4_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOC, LD4_Pin, GPIO_PIN_RESET);
	}
}

void handleDOP(char * gpgsa)
{
	char argv[32][16] = {0};
	uint8_t argc = parseNMEA(gpgsa, argv, 32);

	if(argc < 15)
		return;

	int fix = atoi(argv[2]);
	int pdop = atoi(argv[14]);
	int hdop = atoi(argv[15]);

	printf("DOP: %s fix: %d P:%d H:%d\n", argv[1], fix, pdop, hdop);
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
		//printf("=> %s\n", nmea);
		handleSatelites(nmea);
	}

//	if(!strncmp(nmea, "$GPGSA", 6))
//	{
//		//printf("=> %s\n", nmea);
//		handleDOP(nmea);
//	}
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

int gps_get_time(int *hours, int *minutes, int *seconds)
{
	if(!gps_lock)
		return 0;

	*hours = _hours;
	*minutes = _minutes;
	*seconds = _seconds;
	return 1;
}

int gps_get_satellites(int *fix, int *sv)
{
	*fix = _fix;
	*sv = _sv;
	return 1;
}
