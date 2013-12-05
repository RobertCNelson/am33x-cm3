#include <puts.h>
#include <string.h>

extern unsigned char _logbuf_start;
extern unsigned char _logbuf_end;

static unsigned char *logbuf_pos;

int putchar(int c)
{
	if (!logbuf_pos) {
		logbuf_pos = &_logbuf_start;
		memset(logbuf_pos, 0, &_logbuf_end - &_logbuf_start);
	}
	*logbuf_pos++ = c;
	if (logbuf_pos == &_logbuf_end)
		logbuf_pos = &_logbuf_start;
	return 0;
}

int puts(const char *str)
{
	const char *start = str;
	while (*str)
		putchar(*str++);
	return str - start;
}

int putsn(const char *str, size_t maxlen)
{
	const char *start = str;
	while (*str && maxlen--)
		putchar(*str++);
	return str - start;
}
