/*-
 * Copyright (c) 1986, 1988, 1991, 1993
 *	The Regents of the University of California.  All rights reserved.
 * (c) UNIX System Laboratories, Inc.
 * All or some portions of this file are derived from material licensed
 * to the University of California by American Telephone and Telegraph
 * Co. or Unix System Laboratories, Inc. and are reproduced herein with
 * the permission of UNIX System Laboratories, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)subr_prf.c	8.3 (Berkeley) 1/21/94
 */

#include <ctype.h>
#include <string.h>
#include <puts.h>
#include <printf.h>
#include <stddef.h>

#define BIT(n)	(1 << (n))

#define FLAG_Q	BIT(0)
#define FLAG_L	BIT(1)
#define FLAG_C	BIT(2)
#define FLAG_H	BIT(3)
#define FLAG_J	BIT(4)
#define FLAG_T	BIT(5)
#define FLAG_Z	BIT(6)
#define LADJUST	BIT(7)
#define SHARP	BIT(8)
#define NEG	BIT(9)
#define SIGN	BIT(10)
#define DOT	BIT(11)
#define UPPER	BIT(12)

static char const hex2ascii_data[] = "0123456789abcdefghijklmnopqrstuvwxyz";

/*
 * Put a NULL-terminated ASCII number (base <= 36) in a buffer in reverse
 * order; return an optional length and a pointer to the last character
 * written in the buffer (i.e., the first character of the string).
 */
static char *ksprintn(char *nbuf, unsigned int num, int base, int *lenp,
			bool upper)
{
	char *p, c;

	p = nbuf;
	*p = '\0';
	do {
		c = hex2ascii_data[num % base];
		*++p = upper ? toupper(c) : c;
	} while (num /= base);
	if (lenp)
		*lenp = p - nbuf;
	return p;
}

static int put_pad(char pad, int count)
{
	int ret = count;
	while (count--)
		putchar(pad);
	return ret;
}

int vprintf(char const *fmt, va_list ap)
{
	/* 
	 * Max number conversion buffer length: a unsigned long long in
	 * base 2, plus NULL byte.
	 */
	char nbuf[sizeof(int) * 8 + 1];
	const char *p, *percent;
	int ch, n;
	unsigned int num;
	int base, tmp, width;
	int dwidth;
	char padc;
	bool stop = false;
	int ret = 0;
	unsigned long flags;

	num = 0;

	for (;;) {
		padc = ' ';
		width = 0;
		while ((ch = (unsigned char) *fmt++) != '%' || stop) {
			if (ch == '\0')
				return ret;
			putchar(ch);
			ret++;
		}
		percent = fmt - 1;
		flags = 0;
		dwidth = 0;
reswitch:	switch (ch = (unsigned char) *fmt++) {
		case '.':
			flags |= DOT;
			goto reswitch;
		case '#':
			flags |= SHARP;
			goto reswitch;
		case '+':
			flags |= SIGN;
			goto reswitch;
		case '-':
			flags |= LADJUST;
			goto reswitch;
		case '%':
			putchar(ch);
			ret++;
			break;
		case '*':
			if (!(flags & DOT)) {
				width = va_arg(ap, int);
				if (width < 0) {
					flags ^= LADJUST;
					width = -width;
				}
			} else
				dwidth = va_arg(ap, int);
			goto reswitch;
		case '0':
			if (!(flags & DOT)) {
				padc = '0';
				goto reswitch;
			}
		case '1' ... '9':
			for (n = 0;; ++fmt) {
				n = n * 10 + ch - '0';
				ch = *fmt;
				if (ch < '0' || ch > '9')
					break;
			}
			if (flags & DOT)
				dwidth = n;
			else
				width = n;
			goto reswitch;
		case 'c':
			putchar(va_arg(ap, int));
			ret++;
			break;
		case 'd':
		case 'i':
			base = 10;
			flags |= SIGN;
			goto handle_sign;
		case 'h':
			if (flags & FLAG_H) {
				flags &= ~FLAG_H;
				flags |= FLAG_C;
			} else
				flags |= FLAG_H;
			goto reswitch;
		case 'j':
			flags |= FLAG_J;
			goto reswitch;
		case 'l':
			if (flags & FLAG_L) {
				flags &= ~FLAG_L;
				flags |= FLAG_Q;
			} else
				flags |= FLAG_L;
			goto reswitch;
		case 'n':
			if (flags & FLAG_J)
				*(va_arg(ap, int *)) = ret;
			else if (flags & FLAG_Q)
				*(va_arg(ap, long long *)) = ret;
			else if (flags & FLAG_L)
				*(va_arg(ap, long *)) = ret;
			else if (flags & FLAG_Z)
				*(va_arg(ap, size_t *)) = ret;
			else if (flags & FLAG_H)
				*(va_arg(ap, short *)) = ret;
			else if (flags & FLAG_C)
				*(va_arg(ap, char *)) = ret;
			else
				*(va_arg(ap, int *)) = ret;
			break;
		case 'o':
			base = 8;
			goto handle_nosign;
		case 'p':
			base = 16;
			if (width == 0)
				flags |= SHARP;
			else
				flags &= ~SHARP;
			flags &= ~SIGN;
			num = (unsigned long) va_arg(ap, void *);
			goto number;
		case 'q':
			flags |= FLAG_Q;
			goto reswitch;
		case 's':
			p = va_arg(ap, char *) ? : "(null)";
			n = flags & DOT ? min(dwidth, strlen(p)) : strlen(p);
			width = max(width - n, 0);

			if (!(flags & LADJUST))
				ret += put_pad(padc, width);
			ret += putsn(p, n);
			if (flags & LADJUST)
				ret += put_pad(padc, width);
			break;
		case 't':
			flags |= FLAG_T;
			goto reswitch;
		case 'u':
			base = 10;
			goto handle_nosign;
		case 'X':
			flags |= UPPER;
		case 'x':
			base = 16;
			goto handle_nosign;
		case 'z':
			flags |= FLAG_Z;
			goto reswitch;
handle_nosign:
			flags &= ~SIGN;
			if (flags & FLAG_J)
				num = va_arg(ap, unsigned int);
			else if (flags & FLAG_Q)
				num = va_arg(ap, unsigned long long);
			else if (flags & FLAG_T)
				num = va_arg(ap, long);
			else if (flags & FLAG_L)
				num = va_arg(ap, unsigned long);
			else if (flags & FLAG_Z)
				num = va_arg(ap, size_t);
			else if (flags & FLAG_H)
				num = (unsigned short) va_arg(ap, int);
			else if (flags & FLAG_C)
				num = (unsigned char) va_arg(ap, int);
			else
				num = va_arg(ap, unsigned int);
			goto number;
handle_sign:
			if (flags & FLAG_J)
				num = va_arg(ap, int);
			else if (flags & FLAG_Q)
				num = va_arg(ap, long long);
			else if (flags & FLAG_T)
				num = va_arg(ap, long);
			else if (flags & FLAG_L)
				num = va_arg(ap, long);
			else if (flags & FLAG_Z)
				num = va_arg(ap, ssize_t);
			else if (flags & FLAG_H)
				num = (short) va_arg(ap, int);
			else if (flags & FLAG_C)
				num = (char) va_arg(ap, int);
			else
				num = va_arg(ap, int);
number:
			if (flags & SIGN && (int) num < 0) {
				flags |= NEG;
				num = -(int) num;
			}
			p = ksprintn(nbuf, num, base, &tmp, !!(flags & UPPER));
			if (flags & SHARP && num != 0) {
				if (base == 8)
					tmp++;
				else if (base == 16)
					tmp += 2;
			}
			if (flags & NEG)
				tmp++;

			if (!(flags & LADJUST) && padc != '0' && width
			    && (width -= tmp) > 0)
			    	ret += put_pad(padc, width);
			if (flags & NEG) {
				putchar('-');
				ret++;
			}
			if (flags & SHARP && num != 0) {
				if (base == 8) {
					putchar('0');
					ret++;
				} else if (base == 16)
					ret += puts("0x");
			}
			if (!(flags & LADJUST) && width && (width -= tmp) > 0)
			    	ret += put_pad(padc, width);

			while (*p) {
				putchar(*p--);
				ret++;
			}

			if (flags & LADJUST && width && (width -= tmp) > 0)
			    	ret += put_pad(padc, width);
			break;
		default:
			ret += putsn(percent, fmt - percent);
			/*
			 * Since we ignore an formatting argument it is no
			 * longer safe to obey the remaining formatting
			 * arguments as the arguments will no longer match
			 * the format specs.
			 */
			stop = true;
			break;
		}
	}
}

int printf(const char *fmt, ...)
{
	va_list ap;
	int ret;

	va_start(ap, fmt);
	ret = vprintf(fmt, ap);
	va_end(ap);

	return ret;
}
