#include <string.h>

void *memset(void *s, int c, size_t n)
{
        char *cs = s;
        while (n--)
                *cs++ = c;
        return s;
}

size_t strlen(const char *s)
{
	size_t ret = 0;
	while (*s++)
		ret++;
	return ret;
}

