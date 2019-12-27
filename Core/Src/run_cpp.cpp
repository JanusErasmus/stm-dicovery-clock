#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"

#include "Utils/utils.h"
#include "Utils/crc.h"
#include "interface_nrf24.h"

#define MINIMUM_REPORT_RATE 600000// 1800000

static const uint8_t netAddress[] = {0x23, 0x1B, 0x25};
static const uint8_t serverAddress[] = {0x12, 0x3B, 0x45};
static bool reportToServer = false;

extern "C"{

extern RTC_HandleTypeDef hrtc;
void sample_adc(double *temperature, double *voltages);

void debug_nrf(uint8_t argc, char **argv)
{
	printf("Reporting to server...\n");
	reportToServer = true;
}

}

enum nodeFrameType_e
{
	DATA = 0,
	COMMAND = 1,
	ACKNOWLEDGE = 2
};

//Node send 32 bytes of data, with the last byte being the 8-bit CRC
typedef struct {
	uint8_t nodeAddress;	//1
	uint8_t frameType;		//1
	uint32_t timestamp;		//4  6
	uint8_t inputs;			//1  7
	uint8_t outputs;		//1  8
	uint16_t voltages[4];	//8  16
	uint16_t temperature;	//2  18
	uint8_t reserved[13]; 	//13 31
	uint8_t crc;			//1  32
}__attribute__((packed, aligned(4))) nodeData_s;


void report(const uint8_t *address, bool sampled)
{
	nodeData_s pay;
	memset(&pay, 0, 32);
	pay.nodeAddress = NODE_ADDRESS;
	pay.timestamp = HAL_GetTick();

	double temperature, voltages;
	sample_adc(&temperature, &voltages);

	pay.temperature = voltages * 100000;

	//let the server know when the sample was taken as a sample
	if(sampled)
		pay.voltages[2] = 1;

	pay.crc = CRC_8::crc((uint8_t*)&pay, 31);

	//report status in voltages[0-1]
	printf("TX result %d\n", InterfaceNRF24::get()->transmit(address, (uint8_t*)&pay, 32));
}

void reportNow(bool sampled)
{
	report(serverAddress, sampled);
}

bool NRFreceivedCB(int pipe, uint8_t *data, int len)
{
	if(pipe != 0)
	{
		printf(RED("%d NOT correct pipe\n"), pipe);
		return false;
	}


	if(CRC_8::crc(data, 32))
	{
		printf(RED("CRC error\n"));
		return false;
	}

	bool reportNow = false;
	nodeData_s down;
	memcpy(&down, data, len);
	printf("NRF RX [0x%02X]\n", down.nodeAddress);

	//Check of this is not my data
	if(down.nodeAddress != NODE_ADDRESS)
	{
		if(down.nodeAddress == 0xFF)
		{
			reportNow = true;
		}
		else
			return false;
	}

	if(down.frameType == ACKNOWLEDGE)
	{
		printf("Main: " GREEN("ACK\n"));
		return false;
	}

	printf("RCV Type# %d\n", (int)down.frameType);
	//printf(" PAYLOAD: %d\n", len);
	//diag_dump_buf(data, len);

	int hour = (down.timestamp >> 8) & 0xFF;
	int min = (down.timestamp) & 0xFF;
	printf("Set time %d:%d\n", hour, min);

	RTC_TimeTypeDef sTime;
	sTime.Hours = hour;
	sTime.Minutes = min;
	sTime.Seconds = 0;
	HAL_StatusTypeDef result = HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	if(result != HAL_OK)
		printf("Could not set Time!!! %d\n", result);


	int month = (down.timestamp >> 24) & 0xFF;
	int day = (down.timestamp >> 16) & 0xFF;
	printf("Set date %d:%d\n", month, day);

	RTC_DateTypeDef sDate;
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	sDate.Month = month;
	sDate.Date = day;
	result = HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	if(result != HAL_OK)
		printf("Could not set Date!!! %d\n", result);

	//Broadcast pipe
	if(reportNow)
	{
		reportToServer = true;
	}

	//command to node
	if(down.frameType == COMMAND)
	{
		printf("Set Outputs %d\n", down.outputs);
	}

	return false;
}

extern "C" {
void init_cpp(SPI_HandleTypeDef *spi)
{
	InterfaceNRF24::init(spi, netAddress, 3);
	InterfaceNRF24::get()->setRXcb(NRFreceivedCB);
}

static uint32_t lastSample = 0;

void run_cpp()
{
	if(reportToServer)
	{
		lastSample = HAL_GetTick() + MINIMUM_REPORT_RATE;
		reportToServer = false;
		reportNow(false);
	}
	InterfaceNRF24::get()->run();

	if(lastSample < HAL_GetTick())
	{
		lastSample = HAL_GetTick() + MINIMUM_REPORT_RATE;
		reportNow(true);
	}

}
}
