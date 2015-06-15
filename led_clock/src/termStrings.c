#include "termStrings.h"

char* t_strcmp(const char* str1, const char* str2)
{
	int k=0;
	 while(str1[k] == str2[k])
	 {
		 k++;

		 if(str1[k] == 0 && str2[k] == 0)
			 return 0;
		 else if (str1[k] == 0 || str2[k] == 0)
			break;
	 }

	 return (char*)&str1[k];
}

uint16_t t_strlen(const char* str)
{
	uint16_t len = 0;

	while(str[len] != 0)
	{
		if(len++ > 65000)
		{
			len = 0;
			break;
		}
	}

	return len;
}

void t_parseArgs(char * str, char * argv[], int * argc)
{
	uint16_t len = 0;
	int argMax = *argc;

	*argc = 0;

	while((str[len] != 0) && (*argc < argMax))
	{
		if(str[len] == ' ')
		{
			str[len] = 0;

			argv[(*argc)++] = &str[len + 1];
		}

		if((str[len] == '\n') || (str[len] == '\r'))
					break;

		len++;
	}
}

void t_stripWhite(char * str)
{
	uint16_t len = 0;

	while(str[len] != 0)
	{
		if((str[len] == '\n') || (str[len] == '\r'))
			str[len] = 0;

		if(len++ > 65000)
		{
			len = 0;
			break;
		}

	}
}

uint16_t t_atoi(const char * str)
{
	uint16_t num = 0;

	uint16_t len = t_strlen(str);

	char * charNum = str;

	if(len == 4)
	{
		num += (charNum[0] - 0x30) * 1000;
		len--;
		charNum++;
	}

	if(len == 3)
	{
		num += (charNum[0] - 0x30) * 100;
		len--;
		charNum++;
	}

	if(len == 2)
	{
		num += (charNum[0] - 0x30) * 10;
		len--;
		charNum++;
	}

	if(len == 1)
	{
		num += (charNum[0] - 0x30);
	}

	return num;
}

