#ifndef UTILITY_H_INCLUDED
#define UTILITY_H_INCLUDED

#include <stdint.h>

#define UNDEFINED -1;

/*
 * 
 */
int safeStrtoi(char *s, int base);

char * intToLongHexString(int value);

char * intToShortHexString(int value);

void pad(char *s, int n, int c);

void dec_to_str(char *str, uint32_t val, size_t digits);

char ** str_split(char* a_str, const char a_delim);

char *strdup (const char *s);

#endif
