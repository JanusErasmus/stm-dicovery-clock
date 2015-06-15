#include <stm32f10x.h>

#include "terminal.h"
#include "termHandler.h"
#include "termStrings.h"
#include "version.h"
#include "led.h"
#include "rtc.h"

typedef void (*func_t)(char * argv[], int argc);

void help(char * argv[], int argc);
void version(char * argv[], int argc);
void reset(char * argv[], int argc);

typedef struct
{
	char* cmd;
	char* desc;
	func_t f;

}termCommands;

termCommands cmdTable[] = {
		{"h", "Shows this help", help},
		{"ver", "Bootloader Version", version},
		{"reset", "Reset Processor", reset},
		{"time", "Set/Display RTC time <HH MM dd mm yyyy>", rtc_setTime},

		{0,0,0}
};

bool t_Handler(char* cmd)
{
	bool stat = 0;

	char * argv[6];
	int argc = 6;


	t_stripWhite(cmd);
	t_parseArgs(cmd, argv, &argc);

	uint8_t k = 0;
	while(cmdTable[k].f)
	{
		if(!t_strcmp(cmdTable[k].cmd, cmd))
		{
			stat = 1;
			cmdTable[k].f(argv, argc);
		}


		k++;
	}

	return stat;
}

void version(char * argv[], int argc)
{
	t_print("Boot loader Version:");
	h_print(VERSION_NUMBER);
	t_print("Build Date: ");
	t_print(__DATE__);
	t_print(" ");
	t_print(__TIME__);
	t_print("\n");
}


typedef struct
{
  __IO uint32_t resetRegister;
}SPECIAL_RESET_REG;

void reset(char * argv[], int argc)
{
	t_print("Processor will RESET\n");

	uint32_t resetReg = 0xE000ED00 + 0x0C;
	((SPECIAL_RESET_REG *) resetReg)->resetRegister = (0x5FA << 16) | (1 << 2);
}


void help(char * argv[], int argc)
{
	t_print("Help:\n");
	uint8_t k = 0;
	while(cmdTable[k].f)
	{
		t_print(" ");
		t_print(cmdTable[k].cmd);
		t_print(" - ");
		t_print(cmdTable[k].desc);
		t_print("\n");

		k++;
	}
}
