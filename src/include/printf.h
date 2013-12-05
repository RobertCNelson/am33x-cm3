#ifndef _PRINTF_H_
#define _PRINTF_H_

#include <types.h>

int vprintf(char const *fmt, va_list ap);
int printf(const char *fmt, ...);

#endif
