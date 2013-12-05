#include <printf.h>
#include <puts.h>
#include <debug.h>

static int debug_level = LVL_DEBUG;
static int debug_idx;

static const char *levels[] = {
	" ",
	" ERR",
	" WARN",
	" INFO",
	" DEBUG",
};

int debug_printf(int lvl, const char *fmt, ...)
{
	unsigned long flags;
	int ret;
	va_list ap;

	if (lvl > debug_level)
		return 0;

	/* Save IRQ state and disable IRQs */
	asm volatile("mrs %0, primask\n"
		     "cpsid i" : "=r"(flags) : : "memory", "cc");

	ret = printf("[%08x]%s: ", debug_idx++, levels[lvl]) + 1;
	va_start(ap, fmt);
	ret += vprintf(fmt, ap);
	va_end(ap);
	putchar('\n');

	/* Restore IRQ state */
	asm volatile("msr primask, %0" : : "r" (flags) : "memory", "cc");

	return ret;
}

void debug_set_level(int lvl)
{
	debug_level = lvl;
}

