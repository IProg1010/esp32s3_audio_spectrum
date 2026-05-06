/* Copy me if you can. */

/*
 * usage:
 * #include "arg.h"
 * char *argv0;
 * int main(int argc, char **argv) {
 *     char *f;
 *     ARGBEGIN {
 *         case 'b': printf("option -b\n"); break;
 *         case 'f': printf("option -f %s\n", (f=ARGF())? f : "no arg"); break;
 *         default: printf("bad option %c\n", ARGC()); break;
 *     } ARGEND
 *     return 0;
 * }
 *
 * ARGC() returns the current option character.
 * ARGF() returns the current option argument:
 *        the rest of the option string if not empty, or the next argument
 *        in argv if any, or NULL. ARGF() must be called only once for each
 *        option argument.
 * EARGF(code) same as ARGF() but instead of returning NULL runs `code' and,
 *        if that returns, calls abort(2). A typical value for code is usage(),
 *        as in EARGF(usage()).
 * ARGSTOP() stop processing arguments, current and following options will be
 *           passed as arguments
 *
 * ARGLONG mark the start of a block that will handle long options
 *
 * After ARGBEGIN, argv0 is a copy of argv[0]
 *
 * After ARGEND, argv points at a NULL terminated list of the remaining
 * argc arguments
 *
 * changelog:
 * 2026-01-26 cleanup and documentation
 * 2025-06-19 fix ARGEND macro and add ARGSTOP macro
 */

#ifndef ARG_H
#define ARG_H

#define ARGBEGIN {							\
	char *a, **argp, **args;					\
	for (argv0 = *argv++, argc--, args = argp = argv;		\
		(a = *argp) != NULL;	/* while != NULL */		\
		*argp ? argp++ : 0)	/* inc only if argp != NULL */	\
		if (a[0] == '-' && a[1] == '-' && a[2] == '\0')		\
			for (argc--, argp++;	/* skip the '--' arg */	\
			     (*(args++) = *(argp++)) != NULL;);		\
		else if (a[0] == '-' && a[1] != '-' && a[1] != '\0')	\
			for (argc--, a++; *a; *a ? a++ : 0)		\
				switch (a[0])

#define ARGLONG else if (a[0] == '-' && a[1] == '-' && a[2] != '\0')
#define ARGSTOP() for (argc++; (*(args++) = *(argp++)) != NULL;)
#define ARGEND  else *(args++) = *argp; /* else copy the argument */ }

#define ARGC()		(a[0])
#define ARGF()		(a[1]? a + 1 :\
			argp[1]? (argc--, argp++, argp[0]) :\
			NULL)
#define EARGF(x)	(a[1]? a + 1 :\
			argp[1]? (argc--, argp++, argp[0]) :\
			((x), abort(), NULL))

#define ARGLC()		(&a[2])
#define ARGLF()		(argp ? (argp++, *argp) : NULL)
#define EARGLF(x)	(argp ? (argp++, *argp) : ((x), abort(), NULL))

#endif /* ARG_H */
