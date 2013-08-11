/*
 * $Id: s_parse.c,v 1.5 2010/09/28 19:33:21 slava Exp $
 *
 * Copyright (C) 2000-2010 by Slava Monich
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met: 
 *
 *   1.Redistributions of source code must retain the above copyright 
 *     notice, this list of conditions and the following disclaimer. 
 *   2.Redistributions in binary form must reproduce the above copyright 
 *     notice, this list of conditions and the following disclaimer 
 *     in the documentation and/or other materials provided with the 
 *     distribution. 
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) ARISING 
 * IN ANY WAY OUT OF THE USE OR INABILITY TO USE THIS SOFTWARE, EVEN 
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 *
 * The views and conclusions contained in the software and documentation 
 * are those of the authors and should not be interpreted as representing 
 * any official policies, either expressed or implied.
 */

#include "s_util.h"
#include <math.h>

/*==========================================================================*
 *              P A R S E
 *==========================================================================*/

/**
 * Determine whether given string starts with specified non-empty character
 */
STATIC Bool STRING_StartsWithChar(Str s, char c) 
{
    if (s) {
        ASSERT(!IsSpace(c));
        while (*s && IsSpace(*s)) s++;
        return BoolValue(*s == c);
    }
    return False;
}

/**
 * Parse a short, returning True on success.
 */
#ifdef __KERNEL__
#define BOOL_StrnCmp(_s1,_s2,_len) strncmp(_s1, _s2, _len)
#else /* __KERNEL__ */
#define BOOL_StrnCmp(_s1,_s2,_len) StrnCaseCmp(_s1, _s2, _len)
#endif /* __KERNEL__ */

Bool PARSE_Bool(Str s, Bool * b) 
{
    if (s) {
        size_t n, len;
        Str s1;

        while (IsSpace(*s) && *s) s++;
        n = StrLen(s);

        len = StrLen(TRUE_STRING);
        if (n >= len && BOOL_StrnCmp(s, TRUE_STRING, len) == 0) {
            for (s1 = s+len; *s1 && IsSpace(*s1); s1++) NOTHING;
            if (!*s1) {
                if (b) *b = True;
                return True;
            }
        }

        len = StrLen(FALSE_STRING);
        if (n >= len && BOOL_StrnCmp(s, FALSE_STRING, len) == 0) {
            for (s1 = s+len; *s1 && IsSpace(*s1); s1++) NOTHING;
            if (!*s1) {
                if (b) *b = False;
                return True;
            }
        }
    }
    return False;
}

/**
 * Parse a byte, returning True on success.
 */
Bool PARSE_Byte(Str s, char * n, int base) 
{
    long number;
    if (PARSE_Long(s, &number, base)) {
        if (number >= (long)CHAR_MIN && number <= (long)CHAR_MAX) {
            if (n) *n = (char)number;
            return True;
        }
    }
    return False;
}

Bool PARSE_UByte(Str s, unsigned char * n, int base) 
{
    unsigned long number;
    if (PARSE_ULong(s, &number, base)) {
        if (number <= UCHAR_MAX) {
            if (n) *n = (unsigned char)number;
            return True;
        }
    }
    return False;
}

/**
 * Parse a short, returning True on success.
 */
Bool PARSE_Short(Str s, short * n, int base) 
{
    long number;
    if (PARSE_Long(s, &number, base)) {
        if (number >= SHRT_MIN && number <= SHRT_MAX) {
            if (n) *n = (short)number;
            return True;
        }
    }
    return False;
}

Bool PARSE_UShort(Str s, unsigned short * n, int base) 
{
    unsigned long number;
    if (PARSE_ULong(s, &number, base)) {
        if (number <= USHRT_MAX) {
            if (n) *n = (unsigned short)number;
            return True;
        }
    }
    return False;
}

/**
 * Parse an integer, returning True on success.
 */
Bool PARSE_Int(Str s, int * n, int base) 
{
    long number;
    if (PARSE_Long(s, &number, base)) {
        if (number >= INT_MIN && number <= INT_MAX) {
            if (n) *n = (int)number;
            return True;
        }
    }
    return False;
}

Bool PARSE_UInt(Str s, unsigned int * n, int base) 
{
    unsigned long number;
    if (PARSE_ULong(s, &number, base)) {
        if (number <= UINT_MAX) {
            if (n) *n = (unsigned int)number;
            return True;
        }
    }
    return False;
}

/**
 * Parse a long, returning True on success.
 */
Bool PARSE_Long(Str s, long * n, int base) 
{
    if (s) {
        Char * endptr = NULL;
        long number;
        while (*s && IsSpace(*s)) s++;
        number = StrTol(s, &endptr, base);
        if (endptr && endptr != s) {
            if ((number != LONG_MAX && number != LONG_MIN)
#ifdef HAVE_ERRNO
                || (errno != ERANGE)
#endif /* HAVE_ERRNO */
                ) {
                while (*endptr && IsSpace(*endptr)) endptr++;
                if (!*endptr) {
                    if (n) *n = number;
                    return True;
                }
            }
        }
    }
    return False;
}

Bool PARSE_ULong(Str s, unsigned long * n, int base) 
{
    if (s && !STRING_StartsWithChar(s,'-')) {
        Char * endptr = NULL;
        unsigned long number;
        while (*s && IsSpace(*s)) s++;
        number = StrToul(s, &endptr, base);
        if (endptr && endptr != s) {
            if ((number != ULONG_MAX)
#ifdef HAVE_ERRNO
                || (errno != ERANGE)
#endif /* HAVE_ERRNO */
                ) {
                while (*endptr && IsSpace(*endptr)) endptr++;
                if (!*endptr) {
                    if (n) *n = number;
                    return True;
                }
            }
        }
    }
    return False;
}

/**
 * Parse a double, returning True on success.
 */
#ifndef __KERNEL__
Bool PARSE_Double(Str s, double * d) 
{
    if (s) {
        Char * endptr = NULL;
        double value;
        while (*s && IsSpace(*s)) s++;
        value = StrTod(s, &endptr);
        if (endptr && endptr != s) {
            if ((value != HUGE_VAL)
#ifdef HAVE_ERRNO
                || (errno != ERANGE)
#endif /* HAVE_ERRNO */
                ) {
                ASSERT(value == value);
                while (*endptr && IsSpace(*endptr)) endptr++;
                if (!*endptr) {
                    if (d) *d = value;
                    return True;
                }
            }
        }
    }
    return False;
}
#endif /* __KERNEL__ */

/**
 * Parse a float, returning True on success.
 */
#ifndef __KERNEL__
Bool PARSE_Float(Str s, float* f) 
{
    double d;
    if (PARSE_Double(s, &d)) {
        float value = (float)d;
        /* The check below (exponent of all 1s) is true for all NaN and 
         * Inf values */
        if (((*((I32u*)(void*)(&value))) & 0x7f800000) != 0x7f800000) {
            if (f) *f = value;
            return True;
        }
    }
    return False;
}
#endif /* __KERNEL__ */

#if (! defined(HAVE_STRTOLL) || ! defined(HAVE_STRTOULL))
#define MINUS_INT64_MIN (((uint64_t)INT64_MAX)+1)

/* flag values */
#define FL_UNSIGNED   1       /* strtoul called */
#define FL_NEG        2       /* negative sign found */
#define FL_OVERFLOW   4       /* overflow occured */
#define FL_READDIGIT  8       /* we've read at least one correct digit */

static uint64_t strtoxll (
        Str nptr,
        Str* endptr,
        int ibase,
        int flags )
{
    Str p;
    Char c;
    uint64_t number;
    uint64_t maxval;
    unsigned digval;
    
    p = nptr;                       /* p is our scanning pointer */
    number = 0;                     /* start with zero */
    
    c = *p++;                       /* read char */
    while ( IsSpace(c) ) {
        c = *p++;                   /* skip whitespace */
    }

    if (c == '-') {
        flags |= FL_NEG;            /* remember minus sign */
        c = *p++;
    } else if (c == '+') {
        c = *p++;                   /* skip sign */
    }

    if (ibase < 0 || ibase == 1 || ibase > 36) {
        /* bad base! */
        if (endptr)
            /* store beginning of string in endptr */
            *endptr = nptr;
        return 0;                  /* return 0 */

    } else if (ibase == 0) {
        /* determine base free-lance, based on first two chars of
           string */
        if (c != '0')
            ibase = 10;
        else if (*p == 'x' || *p == 'X')
            ibase = 16;
        else
            ibase = 8;
    }
    
    if (ibase == 16) {
        /* we might have 0x in front of number; remove if there */
        if (c == '0' && (*p == 'x' || *p == 'X')) {
            ++p;
            c = *p++;       /* advance past prefix */
        }
    }

    /* if our number exceeds this, we will overflow on multiply */
    maxval = UINT64_MAX / ibase;

    
    for (;;) {      /* exit in middle of loop */
        /* convert c to value */
        if ( IsDigit((unsigned char)c) ) {
            digval = c - '0';
        } else if ( IsAlpha((unsigned char)c) ) {
            digval = ToUpper(c) - 'A' + 10;
        } else {
            break;
        }
         
        if (digval >= (unsigned)ibase) {
            break;          /* exit loop if bad digit found */
        }

        /* record the fact we have read one digit */
        flags |= FL_READDIGIT;
        
        /* we now need to compute number = number * base + digval,
           but we need to know if overflow occured.  This requires
           a tricky pre-check. */
        
        if (number < maxval || (number == maxval &&
           (uint64_t)digval <= (UINT64_MAX % ibase))) {
            /* we won't overflow, go ahead and multiply */
            number = number * ibase + digval;
        } else {
            /* we would have overflowed -- set the overflow flag */
            flags |= FL_OVERFLOW;
        }
        
        c = *p++;               /* read next digit */
    }

    --p;                            /* point to place that stopped scan */

    if (!(flags & FL_READDIGIT)) {
        /* no number there; return 0 and point to beginning of
           string */
        if (endptr) {
            /* store beginning of string in endptr later on */
            p = nptr;
        }
        number = 0;            /* return 0 */
    } else if ( (flags & FL_OVERFLOW) ||
                ( !(flags & FL_UNSIGNED) &&
                  (((flags & FL_NEG) && (number > MINUS_INT64_MIN)) ||
                    ( !(flags & FL_NEG) && (number > INT64_MAX))))) {
        /* overflow or signed overflow occurred */
#ifdef HAVE_ERRNO
        errno = ERANGE;
#endif /* HAVE_ERRNO */
        if ( flags & FL_UNSIGNED ) {
            number = UINT64_MAX;
        } else if ( flags & FL_NEG ) {
            number = MINUS_INT64_MIN;
        } else {
            number = INT64_MAX;
        }
    }

    if (endptr != NULL) {
        /* store pointer to char that stopped the scan */
        *endptr = p;
    }
    
    if (flags & FL_NEG)
        /* negate result if there was a neg sign */
        number = (int64_t)(-(int64_t)number);
    
    return number;                  /* done. */
}

#endif /* (! defined(HAVE_STRTOLL) || ! defined(HAVE_STRTOULL)) */

#ifndef HAVE_STRTOLL
#define strtoll(s,endptr,base) \
    ((int64_t)strtoxll(s,endptr,base,0))
#endif /* HAVE_STRTOLL */

#ifndef HAVE_STRTOULL
#define strtoull(s,endptr,base) \
   strtoxll(s,endptr,base,FL_UNSIGNED)
#endif /* HAVE_STRTOULL */

/**
 * Parse a 64-bit long, returning True on success.
 */
Bool PARSE_Long64(Str s, int64_t * n, int base)
{
    if (s) {
        Char * endptr = NULL;
        int64_t number = strtoll(s, &endptr, base);
        if (endptr && endptr != s) {
            if ((number != INT64_MAX && number != INT64_MIN)
#ifdef HAVE_ERRNO
                || (errno != ERANGE)
#endif /* HAVE_ERRNO */
                ) {
                while (*endptr && IsSpace(*endptr)) endptr++;
                if (!*endptr) {
                    if (n) *n = number;
                    return True;
                }
            }
        }
    }
    return False;
}

Bool PARSE_ULong64(Str s, uint64_t * n, int base)
{
    if (s && !STRING_StartsWithChar(s,'-')) {
        Char * endptr = NULL;
        uint64_t number = strtoull(s, &endptr, base);
        if (endptr && endptr != s) {
            if ((number != UINT64_MAX)
#ifdef HAVE_ERRNO
                || (errno != ERANGE)
#endif /* HAVE_ERRNO */
                ) {
                while (*endptr && IsSpace(*endptr)) endptr++;
                if (!*endptr) {
                    if (n) *n = number;
                    return True;
                }
            }
        }
    }
    return False;
}

/*
 * HISTORY:
 *
 * $Log: s_parse.c,v $
 * Revision 1.5  2010/09/28 19:33:21  slava
 * o removed unnecessary cast
 *
 * Revision 1.4  2010/09/28 19:26:21  slava
 * o 64-bit Unix build issues
 *
 * Revision 1.3  2009/04/09 21:54:57  slava
 * o use size_t instead of int where it's appropriate
 *
 * Revision 1.2  2007/11/30 21:29:34  slava
 * o fixed gcc 4.1.2 warning on Linux ("dereferencing type-punned pointer will
 *   break strict-aliasing rules")
 *
 * Revision 1.1  2006/10/12 18:30:39  slava
 * o splitting s_util.c into multiple files, because it's getting huge.
 *   Moved parsing utilities to s_parse.c
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "gmake -C .."
 * End:
 */
