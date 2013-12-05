#ifndef _DEBUG_H_
#define _DEBUG_H_

#define LVL_NONE	0
#define LVL_ERR		1
#define LVL_WARN	2
#define LVL_INFO	3
#define LVL_DEBUG	4

extern int debug_printf(int lvl, const char *fmt, ...);
extern void debug_set_level(int lvl);

#define err(arg...) debug_printf(LVL_ERR, ##arg)
#define warn(arg...) debug_printf(LVL_WARN, ##arg)
#define info(arg...) debug_printf(LVL_INFO, ##arg)
#define debug(arg...) debug_printf(LVL_DEBUG, ##arg)

#endif
