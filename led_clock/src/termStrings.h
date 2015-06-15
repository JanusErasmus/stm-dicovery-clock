#ifndef TERMSTRINGS_H_
#define TERMSTRINGS_H_
#include <stdint.h>

uint16_t t_strlen(const char* str);
char* t_strcmp(const char* str1, const char* str2);
void t_parseArgs(char * str, char * argv[], int * argc);
void t_stripWhite(char * str);


uint16_t t_atoi(const char * str);

#endif /* TERMSTRINGS_H_ */
