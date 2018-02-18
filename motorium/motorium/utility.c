#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <assert.h>
#include <string.h>

#include <stdio.h>

#include "utility.h"

int safeStrtoi(char *s, int base) {
    errno = 0;
    char *endptr;
    int retVal = UNDEFINED;

    long val = strtol(s, &endptr, base);
    if(errno == ERANGE) {
        switch(val) {
            case LONG_MIN:
                /* undefined - for now dont differentiate ...*/
                break;
            case LONG_MAX:
                /* undefined - for now do not diferentiate ...*/
                break;
            default:
                assert(1==0); /*not possible*/
        }
    } else if (errno != 0) {
        /*wtf?*/
        assert(1==0); /*die.*/
    } else if (*endptr != '\0') {
        /*garbage at the end of the string*/
		retVal = 99;
    } else {
        retVal = (int)val;
    }
    return retVal;
}

char * intToLongHexString(int value) {
	static const char hex_digits[] = "0123456789ABCDEF";
	
	char *retVal = (char *)calloc(5,sizeof(char));	
		
	retVal[0] = '0';
	retVal[1] = 'x';
	retVal[2] = hex_digits[(value >> 4) & 0xf];
	retVal[3] = hex_digits[value & 0xf];
	retVal[4] = '\0';
	
	return retVal;
}

char * intToShortHexString(int value) {
	static const char hex_digits[] = "0123456789ABCDEF";
	
	char *retVal = (char *)calloc(3,sizeof(char));
	
	retVal[0] = hex_digits[(value >> 4) & 0xf];
	retVal[1] = hex_digits[value & 0xf];
	retVal[2] = '\0';
	
	return retVal;
}

void pad(char *s, int n, int c) {
	char *p = s + n - strlen(s);
	strcpy(p,s);
	p--;
	while(p >= s) {p[0] = c; p--; }
}

void dec_to_str(char *str, uint32_t val, size_t digits) {
	size_t i=1u;
	
	for(; i<= digits;i++)
	{
		str[digits-i] = (char)((val % 10u) + '0');
		val/=10u;
	}
	str[i-1u] = '\0';
}

char ** str_split(char* a_str, const char a_delim) {
	
	char ** result 		= 0;
	size_t	count		= 0;
	char*	tmp			= a_str;
	char*	last_comma	= 0;
	
	char delim[2];
	delim[0] = a_delim;
	delim[1] = 0;
	
	//count the elements
	while(*tmp) {
		if (a_delim == *tmp) {
			count ++;
			last_comma = tmp;
		}
		tmp++;
	}
	
	if(count == 0){
		return result;
	}
	
	//add space for trailing token
	count += last_comma < (a_str + strlen(a_str) - 1);
	
	// add space for terminating null string so caller knows where the list of returned strings ends
	count ++;
	result = malloc(sizeof(char*) * count);
	
	if(result) {
		size_t idx = 0;
		char* token = strtok(a_str, delim);
		while(token) {
			assert(idx < count);
			*(result + idx++) = strdup(token);
			token = strtok(0,delim);
		}
		if(idx != count - 1) {
			printf("--%s--\n", a_str);
		}
		assert(idx == count - 1);
		*(result + idx) = 0;
	}
	
	return result;
}

char *strdup (const char *s) {
	char *d = malloc(strlen(s) + 1);
	if (d == NULL) return NULL;
	strcpy(d,s);
	return d;
}




