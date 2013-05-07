/*
 * $Id: s_str.c,v 1.36 2010/11/19 09:24:59 slava Exp $
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
#include "s_mem.h"

/*==========================================================================*
 *              S T R I N G
 *==========================================================================*/

#define MAX_DOUBLE_FRACTION_DIGITS 340

/**
 * Finds the index of the first occurrence of the character c in the string s.
 * If the string does not contain this character, returns -1
 */
int STRING_IndexOf(Str s, Char c)
{
    Str found = StrChr(s, c);
    return (found ? (int)(found - s) : -1);
}

/**
 * Finds the index of the last occurrence of the character c in the string s.
 * If the string does not contain this character, returns -1
 */
int STRING_LastIndexOf(Str s, Char c)
{
    Str found = StrrChr(s, c);
    return (found ? (int)(found - s) : -1);
}

/**
 * Returns hash code for a string. The algorithm is borrowed from 
 * java.lang.String implementation. 
 *
 * NOTE: hash code can be negative
 */
int STRING_HashCode(Str s)
{
    int h = 0;    
    if (s) {
        while (*s) {
            h = (h<<5) - h + *s++;
        }
    }
    return h;
}

/**
 * Returns case-insensitive hash code for a string. The algorithm is borrowed 
 * from java.lang.String implementation
 *
 * NOTE: hash code can be negative
 */
int STRING_HashCodeNoCase(Str s)
{
    int h = 0;    
    if (s) {
        while (*s) {
            Char c = *s++; /* ToUpper is a macro, may have side effects */
            h = (h<<5) - h + ToUpper(c);
        }
    }
    return h;
}

/**
 * Duplicates a single-byte string.
 */
char * STRING_Dup8(const char * s) 
{
    char * copy = NULL;
    if (s) {
        copy = (char*)MEM_Alloc(strlen(s) + 1);
        if (copy) {
            strcpy(copy,s);
        }
    }
    return copy;
}

/**
 * Duplicates a Unicode string.
 */
wchar_t * STRING_DupU(const wchar_t * ws) 
{
    wchar_t * copy = NULL;
    if (ws) {
        /* inline variant of wcslen... on some systems it's unavailable */
        const wchar_t * p = ws;
        while (*p++) NOTHING;
        copy = (wchar_t*)MEM_Alloc(sizeof(wchar_t)*(p-ws));
        if (copy) {
            wchar_t * dst = copy;
            while ((*dst++ = *ws++) != 0) NOTHING;
        }
    }
    return copy;
}

/**
 * Formats the string into the given buffer (most commonly allocated 
 * on caller's stack). If formatted string fits into the provided buffer,
 * returns pointer to the given buffer. If formatted string is too long 
 * to fit into the provided buffer, allocates a new one and returns pointer
 * to it. If memory allocation fails, returns pointer to the buffer that
 * contains truncated string. Can only return NULL if the first parameter 
 * is NULL.  
 */
Char * STRING_FormatVa(Char * buf, int bufsize, Str format, va_list va)
{
    int size;
    Char * buffer;

    if (buf) {
        size = bufsize;
        buffer = buf;
    } else {
        size = MAX(100,bufsize);
        buffer = (Char*)MEM_Alloc(size * sizeof(Char));
    }

    while (buffer)  {
        Char * newbuf;
        int nchars;

        /* try to print in the allocated space. */
#ifdef va_copy
        va_list va2;
        va_copy(va2, va);
        nchars = Vsnprintf(buffer, size, format, va2);
        va_end(va2);
#else   /* va_copy */
        /* NOTE: reusing va_list is risky and may not work... */
        nchars = Vsnprintf(buffer, size, format, va);
#endif  /* va_copy */

        /* if that worked, return the string. */
        if (nchars > -1 && nchars < size) {
            break;
        }

        /* else try again with more space. */
        if (nchars > -1) {
            size = nchars+1;  /* precisely what is needed */
        } else {
            size *= 2;        /* twice the old size */
        }

        newbuf = (Char*)MEM_Alloc(size * sizeof(Char));
        if (newbuf && buffer && (buffer != buf)) MEM_Free(buffer);

        /* If memory allocation fails, break the loop and return prev buffer */
        if (!newbuf) {
            break;
        }
    }
    return buffer;
}

/**
 * Same as the above except that it takes variable argument list
 */
Char * STRING_Format(Char * buf, int bufsize, Str format, ...) 
{
    Char* result;
    va_list va;
    va_start(va, format);
    result = STRING_FormatVa(buf, bufsize, format, va);
    va_end(va);
    return result;
}

/**
 * Returns True if string s1 starts with the sequence of characters 
 * represented by string s2. If s2 is empty, always return True.
 * The comparison is case sensitive
 */
Bool STRING_StartsWith8(const char * s1, const char * s2)
{
    while (*s2) {
        if (*s1++ != *s2++) {
            return False;
        }
    }
    return True;
}

Bool STRING_StartsWithU(const wchar_t * s1, const wchar_t * s2)
{
    while (*s2) {
        if (*s1++ != *s2++) {
            return False;
        }
    }
    return True;
}

/**
 * Case insensitive version of STRING_StartsWith
 */
#ifndef __KERNEL__
Bool STRING_StartsWithNoCase8(const char * s1, const char * s2)
{
    while (*s2) {
        if (*s1++ != *s2++) {
            /* switch to more expensive case-insensitive algorithm */
            size_t len1 = strlen(s1--)+1;
            size_t len2 = strlen(s2--)+1;
            if (len1 >= len2) {
                if (strncasecmp(s1,s2,len2) == 0) {
                    return True;
                }
            }
            return False;
        }
    }
    return True;
}
#endif /* __KERNEL__ */

/**
 * Returns True if string s1 ends with the sequence of characters 
 * represented by string s2. If s2 is empty, always return True.
 * The comparison is case sensitive
 */
Bool STRING_EndsWith8(const char * s1, const char * s2)
{
    size_t len2 = strlen(s2);
    if (len2 > 0) {
        size_t len1 = strlen(s1);
        if (len1 >= len2) {
            if (strcmp(s2, s1 + len1 - len2) == 0) {
                return True;
            }
        }
        return False;
    }
    return True;
}

Bool STRING_EndsWithU(const wchar_t * s1, const wchar_t * s2)
{
    size_t len2 = wcslen(s2);
    if (len2 > 0) {
        size_t len1 = wcslen(s1);
        if (len1 >= len2) {
            if (wcscmp(s2, s1 + len1 - len2) == 0) {
                return True;
            }
        }
        return False;
    }
    return True;
}

/**
 * Case insensitive version of STRING_EndsWith
 */
#ifndef __KERNEL__
Bool STRING_EndsWithNoCase8(const char * s1, const char * s2)
{
    size_t len2 = strlen(s2);
    if (len2 > 0) {
        size_t len1 = strlen(s1);
        if (len1 >= len2) {
            if (strcasecmp(s2, s1 + len1 - len2) == 0) {
                return True;
            }
        }
        return False;
    }
    return True;
}
#endif /* __KERNEL__ */

/**
 * Formats a double value into the shortest possible string without
 * losing precision.
 */
#ifndef __KERNEL__
Str STRING_FormatDouble(StrBuf * sb, double d)
{
    /* Try to achieve exact representation */
    size_t len;
    int precision = 2;
    for (;;) {
        Char format[16];
        Sprintf(format, TEXT("%%.%df"), precision);
        if (STRBUF_Format(sb, format, d)) {
            double tmp;
            if (PARSE_Double(STRBUF_Text(sb), &tmp)) {
                if (tmp == d || precision >= MAX_DOUBLE_FRACTION_DIGITS) {
                    break;
                } else {
                    precision *= 2;
                    continue;
                }
            }
        }
        return NULL;
    }

    /* strip trailing zeros */
    while ((len = STRBUF_Length(sb)) >= 2 &&
            STRBUF_CharAt(sb,len-1) == '0' &&
            STRBUF_CharAt(sb,len-2) != '.') {
        STRBUF_SetLength(sb, len-1);
    }
    return STRBUF_Text(sb);
}
#endif /* __KERNEL__ */

/**
 * Formats a single precision floating point value into the shortest possible
 * string without losing precision.
 */
#ifndef __KERNEL__
Str STRING_FormatFloat(StrBuf * sb, float f)
{
    /* Try to achieve exact representation */
    size_t len;
    int precision = 2;
    for (;;) {
        Char format[16];
        Sprintf(format, TEXT("%%.%df"), precision);
        if (STRBUF_Format(sb, format, (double)f)) {
            float tmp;
            if (PARSE_Float(STRBUF_Text(sb), &tmp)) {
                if (tmp == f || precision >= (MAX_DOUBLE_FRACTION_DIGITS/2)) {
                    break;
                } else {
                    precision += 4;
                    continue;
                }
            }
        }
        return NULL;
    }

    /* strip trailing zeros */
    while ((len = STRBUF_Length(sb)) >= 2 &&
            STRBUF_CharAt(sb,len-1) == '0' &&
            STRBUF_CharAt(sb,len-2) != '.') {
        STRBUF_SetLength(sb, len-1);
    }
    return STRBUF_Text(sb);
}
#endif /* __KERNEL__ */

/**
 * Primitive strcasecmp substitution for Linux kernel environment 
 */
#ifdef _LINUX_KERNEL
int strcasecmp(const char * dst, const char * src)
{
    int f,l;
    do {
        if (((f = (unsigned char)(*(dst++))) >= 'A') && (f <= 'Z')) {
            f -= ('A' - 'a');
        }
        if (((l = (unsigned char)(*(src++))) >= 'A') && (l <= 'Z')) {
            l -= ('A' - 'a');
        }
    } while (f && (f == l));
    return(f - l);
}
#else /* ! _LINUX_KERNEL */

/**
 * Converts Unicode string to multibyte. The result is allocated from 
 * the heap. The caller is responsible for deallocating the result of
 * conversion. Returns NULL if the argument is NULL or memory allocation 
 * fails.
 */
char * STRING_ToMultiByte(const wchar_t * ws)
{
    char * s = NULL;
    if (ws) {
        size_t n = wcstombs(NULL, ws, 0);
        if (n != ((size_t)-1)) {
            s = MEM_NewArray(char, n+1);
            if (s) {
                size_t k = wcstombs(s, ws, n+1);
                if (k == ((size_t)-1)) {
                    /* coversion to the default locale failed */
                    MEM_Free(s);
                    s = NULL;
                } else {
                    ASSERT(k <= n);
                    s[n] = 0;
                }
            }
        } else {
            for (n = 0; ws[n]; n++) NOTHING;
            s = MEM_NewArray(char, n+1);
            if (s) {
                size_t i;
                for (i=0; i<n; i++) {
                    ASSERT((ws[i] & (~((wchar_t)0x7f))) == 0);
                    s[i] = (char)ws[i];
                }
                s[i] = 0;
            }
        }
    }
    return s;
}

#ifdef _NO_WCHAR_H
#  define wcsncpy _slib_wcsncpy
static wchar_t * wcsncpy(wchar_t * dest, const wchar_t * src, size_t n)
{
    wchar_t * start = dest;
    while (n && ((*dest++ = *src++) != 0)) n--;
    if (n) *dest++ = 0;
    return start;
}
#endif /* _NO_WCHAR_H */

/**
 * Converts Unicode string of specified length to multibyte string. The 
 * result is allocated from the heap. The caller is responsible for 
 * deallocating the result of conversion. Returns NULL if the argument
 * is NULL or memory allocation fails.
 */
char * STRING_ToMultiByteN(const wchar_t * ws, size_t count)
{
    char * s = NULL;
    wchar_t * ws2 = MEM_NewArray(wchar_t, count+1);
    if (ws2) {
        wcsncpy(ws2, ws, count);
        ws2[count] = 0;
        s = STRING_ToMultiByte(ws2);
        MEM_Free(ws2);
    }
    return s;
}

/**
 * Converts multibyte string to Unicode. The result is allocated from 
 * the heap. The caller is responsible for deallocating the result of
 * conversion. Returns NULL if the argument is NULL or memory allocation 
 * fails.
 */
wchar_t * STRING_ToUnicode(const char * s)
{
    wchar_t * ws = NULL;
    if (s) {
        size_t n = mbstowcs(NULL, s, 0);
        if (n != ((size_t)-1)) {
            ws = MEM_NewArray(wchar_t, n+1);
            if (s) {
                mbstowcs(ws, s, n+1);
                ws[n] = 0;
            }
        } else {
            n = strlen(s);
            ws = MEM_NewArray(wchar_t, n+1);
            if (ws) {
                size_t i;
                for (i=0; i<n; i++) {
                    ASSERT((s[i] & (~((char)0x7f))) == 0);
                    ws[i] = s[i];
                }
                ws[i] = 0;
            }
        }
    }
    return ws;
}

#  if defined(__CW32__) && defined(UNICODE)
/**
 * CodeWarrior incorrectly defines swprintf function. This is the
 * right swprintf implementation for the CoreWarrior build.
 */
int _cw_swprintf(wchar_t * dst, const wchar_t * format, ...)
{
    int len;
    va_list va;
    StrBuf sb;
    STRBUF_InitBuf(&sb, dst, INT_MAX);
    va_start(va, format);
    STRBUF_FormatVa(&sb, format, va);
    len = STRBUF_Length(&sb);
    STRBUF_Destroy(&sb);
    va_end(va);
    return len;
}
#  endif /* !(__CW32__ && UNICODE) */

#endif /* ! _LINUX_KERNEL */

/**
 * Helper for STRING_Split
 */
STATIC int STRING_DoSplit(Str s, StrBuf* sb, Vector* v, Str del, Bool emptyOK)
{
    int count = 0;
    Str p;

    for (p=s; *p; p++) {
        for (; *p && StrChr(del,*p); p++) {
            if (emptyOK) {
                /* empty substring */
                count++;
                if (v) {
                    Char * empty = STRING_Dup(TEXT(""));
                    if (empty) {
                        if (VECTOR_Add(v, empty)) {
                            continue;
                        }
                        MEM_Free(empty);
                    }
                    return -1;
                }
            }
        }

        if (*p) {
            /* read the substring */
            STRBUF_Clear(sb);
            for (; *p && !StrChr(del,*p); p++) {
                if (!STRBUF_AppendChar(sb, *p)) {
                    return -1;
                }
            }
            count++;
            if (v) {
                Char * substring = STRBUF_Dup(sb);
                if (substring) {
                    if (VECTOR_Add(v, substring)) {
                        if (*p) {
                            /* p++ at the end of the loop eats up the */
                            /* next delimiter */
                            if (!p[1] && emptyOK) {
                                /* empty substring at the end */
                                count++;
                                if (v) {
                                    Char * empty = STRING_Dup(TEXT(""));
                                    if (empty) {
                                        if (VECTOR_Add(v, empty)) {
                                            continue;
                                        }
                                        MEM_Free(empty);
                                    }
                                    return -1;
                                }
                            }
                            continue;
                        } else {
                            break;
                        }
                    }
                    MEM_Free(substring);
                }
                return -1;
            }
        } else {
            break;
        }
    }

    return count;
}

/**
 * Splits string into substrings, appending allocated substrings to the
 * vector. The caller is responsible for deallocating the substrings. If
 * the last argument is True, if allocates empty substrings, otherwise
 * it skips them. Returns number of sibstrings found in the input string,
 * or (-1) on memory allocation failure. If vector is NULL, it simply counts
 * number of substrings.
 */
int STRING_Split(Str s, Vector * v, Str delim, Bool emptyOK)
{
    int count, origLen = v ? VECTOR_Size(v) : 0;
    StrBuf32 buf;
    STRBUF_InitBufXXX(&buf);

    count = STRING_DoSplit(s, &buf.sb, v, delim, emptyOK);
    if (count < 0) {
        VectorFree f = v->free;
        v->free = vectorFreeValueProc;
        VECTOR_Truncate(v, origLen);
        v->free = f;
    }

    STRBUF_Destroy(&buf.sb);
    return count;
}

/*
 * HISTORY:
 *
 * $Log: s_str.c,v $
 * Revision 1.36  2010/11/19 09:24:59  slava
 * o added STRING_FormatVa
 *
 * Revision 1.35  2010/07/04 13:44:25  slava
 * o UNICODE build issues
 *
 * Revision 1.34  2010/07/03 09:42:42  slava
 * o added UTF-8 utilities
 *
 * Revision 1.33  2009/12/26 12:21:32  slava
 * o working on 64-bit issues
 *
 * Revision 1.32  2009/04/21 18:43:56  slava
 * o more 64-bit issues resolved
 *
 * Revision 1.31  2009/04/13 18:25:07  slava
 * o reduced number of warnings in 64-bit build
 *
 * Revision 1.30  2009/02/09 22:58:25  slava
 * o fixed STRING_Split to handle delimiter at the end of the parsed string
 *
 * Revision 1.29  2008/11/29 15:16:33  slava
 * o fixed annoying compilation warning in STRING_ToUTF8 (right shift by too
 *   large amount, data loss)
 *
 * Revision 1.28  2007/07/12 19:34:34  slava
 * o changed STRING_ToUTF8 to return NULL if the argument is NULL. This
 *   makes it consistent with other functions defined in this file (such
 *   as STRING_ToMultiByte, STRING_ToUnicode etc.)
 *
 * Revision 1.27  2007/05/20 22:09:40  slava
 * o fixed Unicode build
 *
 * Revision 1.26  2007/01/27 02:42:32  slava
 * o added STRING_ToUTF8 function
 *
 * Revision 1.25  2007/01/26 23:30:02  slava
 * o STRING_ToMultiByte should check the return value from the second call
 *   to wcstombs. Even if the first call succeeds, the second one can still
 *   fail.
 *
 * Revision 1.24  2006/11/20 18:44:18  slava
 * o added STRING_Split function
 *
 * Revision 1.23  2006/10/20 04:36:13  slava
 * o updated comment
 *
 * Revision 1.22  2006/10/05 17:11:00  slava
 * o added STRING_FormatFloat function
 *
 * Revision 1.21  2006/10/02 04:21:44  slava
 * o STRING_Dup8 was allocating (at least) two times more memory than
 *   necessary in Unicode build
 *
 * Revision 1.20  2006/04/13 12:15:39  slava
 * o fixed Unicode build
 *
 * Revision 1.19  2006/03/27 21:52:33  slava
 * o STRING_DupU is now a function present in both Unicode and single-byte
 *   builds, and STRING_Dup is a macro pointing to either STRING_Dup8 or
 *   STRING_DupU depending on whether or not UNICODE macro is defined.
 *   This change is source code compatible with the older builds (but
 *   not binary compatible)
 *
 * Revision 1.18  2006/03/19 22:05:24  slava
 * o MacOS X compilation issues
 *
 * Revision 1.17  2005/12/17 00:45:39  slava
 * o fixed a bug in STRING_ToMultiByteN. The new implementation is less
 *   efficient but always works as expected.
 *
 * Revision 1.16  2005/03/09 08:01:20  slava
 * o eliminated dependency on wcslen
 *
 * Revision 1.15  2005/02/20 20:31:30  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.14  2005/01/24 18:24:19  slava
 * o added STRING_ToMultiByteN function
 *
 * Revision 1.13  2005/01/02 00:09:01  slava
 * o added STRING_StartsWithNoCase and STRING_EndsWithNoCase functions which
 *   are case-insensitive versions of STRING_StartsWith and STRING_EndsWith,
 *   respectively.
 *
 * Revision 1.12  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.11  2004/04/08 01:44:17  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.10  2004/03/25 03:28:32  slava
 * o no floating point numbers in kernel mode
 *
 * Revision 1.9  2004/03/15 18:52:45  slava
 * o added STRING_FormatDouble which formats a double value into the shortest
 *   possible %f style string without losing precision.
 *
 * Revision 1.8  2003/12/04 04:48:55  slava
 * o replaced _LINUX_KERNEL_ with _LINUX_KERNEL for consistency with other
 *   platform defines such as _UNIX, _WIN32, _NT_KERNEL etc.
 *
 * Revision 1.7  2003/12/01 13:17:20  slava
 * o provided primitive strcasecmp substitution for Linux kernel environment
 *
 * Revision 1.6  2003/11/30 02:49:58  slava
 * o port to Linux kernel mode environment
 *
 * Revision 1.5  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.4  2003/05/18 14:17:04  slava
 * o handle the case if wcstombs or mbstowcs fails
 *
 * Revision 1.3  2003/02/05 00:47:30  slava
 * o added STRING_IndexOf and STRING_LastIndexOf
 *
 * Revision 1.2  2001/12/28 00:39:59  slava
 * o removed unnecessary include
 *
 * Revision 1.1  2001/12/23 18:41:16  slava
 * o moved STRING functions to s_str.c
 * o added STRING_ToMultiByte and STRING_ToUnicode
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
