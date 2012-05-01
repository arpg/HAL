/*
 * $Id: s_strbuf.c,v 1.63 2011/02/16 18:25:50 slava Exp $
 *
 * Copyright (C) 2000-2011 by Slava Monich
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

#include "s_strbuf.h"
#include "s_util.h"
#include "s_mem.h"

/*==========================================================================*
 *              S T R I N G    B U F F E R 
 *==========================================================================*/

/**
 * Allocates StrBuf from the heap.
 */
StrBuf * STRBUF_Create()
{
    StrBuf * sb = MEM_New(StrBuf);
    if (sb) {
        STRBUF_Init(sb);
        return sb;
    }
    return NULL;
}

/**
 * Delets StrBuf previously allocated by STRBUF_Create()
 */
void STRBUF_Delete(StrBuf * sb)
{
    if (sb) {
        STRBUF_Destroy(sb);
        MEM_Free(sb);
    }
}

/**
 * Initialize the string buffer
 */
void STRBUF_Init(StrBuf * sb)
{
    STRBUF_InitBuf(sb, NULL, 0);
}

/**
 * Initialize string buffer with external buffer of fixed size (typically
 * allocated on stack) 
 */
void STRBUF_InitBuf2(StrBuf * sb, Char* buf, size_t len, size_t bufsize)
{
    ASSERT(sb);
    if (sb) {
        if (buf && bufsize > 0) {
            ASSERT(len < bufsize);
            sb->s = sb->ext = buf;
            sb->alloc = bufsize;
            sb->len = MIN(bufsize-1,len);
            sb->s[sb->len] = 0;
            ASSERT(StrLen(buf) == sb->len);
        } else {
            memset(sb, 0, sizeof(*sb));
        }
    }
}

/**
 * Initialize string buffer with external buffer of fixed size (typically
 * allocated on stack) 
 */
void STRBUF_InitBuf(StrBuf * sb, Char* buf, size_t bufsize)
{
    STRBUF_InitBuf2(sb, buf, 0, bufsize);
}

/**
 * Destroy the contents of the string buffer.
 */
void STRBUF_Destroy(StrBuf * sb)
{
    if (sb) {
        if (sb->alloc) {
            ASSERT(sb->s);
            if (sb->s != sb->ext) MEM_Free(sb->s);
            sb->alloc = 0;
        }
        sb->len = 0;
        sb->s = NULL;
    }
}

/**
 * Returns length of the string.
 */
size_t STRBUF_Length(const StrBuf * sb)
{
    ASSERT(sb);
    if (sb) {
        ASSERT(sb->len == 0 || StrLen(sb->s) == sb->len);
        return sb->len;
    }
    return 0;
}

/**
 * Makes sure that length of the string does not exceed maxlen
 */
void STRBUF_SetLength(StrBuf * sb, size_t maxlen)
{
    ASSERT(sb);
    if (sb && sb->len > maxlen) {
        sb->len = maxlen;
        sb->s[maxlen] = 0;
    }
}

/**
 * Tests if two strings are equal. If any of them is NULL, the result
 * is False.
 */
Bool STRBUF_Equals(const StrBuf * sb1, const StrBuf * sb2)
{
    if (sb1 && sb2 && sb1->s && sb2->s) {

        /* try some quick checks */
        if (sb1 == sb2) {
            return True;
        } else if (sb1->s == sb2->s) {
            return True;
        } else if (sb1->len != sb2->len) {
            return False;
        }

        /* strings may be equal; compare them */
        return BoolValue(StrCmp(sb1->s, sb2->s) == 0);
    }
    ASSMSG("STRBUF_Equals: invalid arguments!");
    return False;
}

/**
 * Tests if two strings are equal. If any of them is NULL, the result
 * is False.
 */
Bool STRBUF_EqualsTo(const StrBuf * sb, Str s)
{
    if (sb && s) {
        if (sb->len == 0) {
            return BoolValue(!s[0]);
        } else if (sb->s == s) {
            return True;
        } else {
            return BoolValue(StrCmp(sb->s, s) == 0);
        }
    }
    ASSMSG("STRBUF_EqualsTo: invalid arguments!");
    return False;
}

/**
 * Clear the string buffer.
 */
void STRBUF_Clear(StrBuf * sb)
{
    ASSERT(sb);
    if (sb) {
        sb->len = 0;
        if (sb->alloc) sb->s[0] = 0;
    }
}

/**
 * Erase some characters from the string, starting from index 
 * 'from' (inclusive), ending at 'to' (exclusive)
 */
void STRBUF_Erase(StrBuf * sb, size_t from, size_t to)
{
    ASSERT(sb);
    if (sb) {
        ASSERT(from <= to);
        ASSERT(to <= sb->len);
        if (to > sb->len) to = sb->len;
        if (from < to) {
            memmove(sb->s+from, sb->s+to, sizeof(Char)*(sb->len-to+1));
            sb->len -= (to - from);
            ASSERT(!sb->s[sb->len]);
        }
    }
}

/**
 * Removes leading and trailing whitespaces. Returns True if at least one
 * whitespace has been removed, False if string didn't change.
 */
Bool STRBUF_Trim(StrBuf * sb)
{
    Bool trimmed = False;
    if (sb->len > 0) {
        size_t pos;

        /* trim trailing whitespaces */
        pos = sb->len;
        while (pos > 0 && IsSpace(sb->s[pos-1])) pos--;
        if (pos < sb->len) {
            trimmed = True;
            STRBUF_Erase(sb, pos, sb->len);
        }

        /* trim leading whitespaces */
        pos = 0;
        while (pos < sb->len && IsSpace(sb->s[pos])) pos++;
        if (pos > 0) {
            trimmed = True;
            STRBUF_Erase(sb, 0, pos);
        }
    }
    return trimmed;
}

/**
 * Returns character at specified position.
 */
Char STRBUF_CharAt(const StrBuf * sb, size_t pos)
{
    ASSERT(sb);
    if (sb) {
        ASSERT(pos <= sb->len);
        if (pos <= sb->len) {
            return sb->s[pos];
        }
    }
    return 0;
}

/**
 * Returns first character of the string, 0 if string is empty
 */
Char STRBUF_FirstChar(const StrBuf * sb)
{
    ASSERT(sb);
    if (sb && sb->len > 0) {
        return sb->s[0];
    }
    return 0;
}

/**
 * Returns last character of the string, 0 if string is empty
 */
Char STRBUF_LastChar(const StrBuf * sb)
{
    ASSERT(sb);
    if (sb && sb->len > 0) {
        return sb->s[sb->len-1];
    }
    return 0;
}

/**
 * Returns index of the specified character, -1 if character not found
 */
int STRBUF_IndexOf(const StrBuf * sb, Char ch)
{
    ASSERT(sb);
    if (sb && sb->len > 0) {
        Str found = StrChr(sb->s, ch);
        if (found) {
            int pos = (int)(found - sb->s);
            ASSERT(pos >= 0);  /* if negative, string is WAY too long */
            return pos;
        }
    }
    return -1;
}

/**
 * Returns last index of the specified character, -1 if character not found
 */
int STRBUF_LastIndexOf(const StrBuf * sb, Char ch)
{
    ASSERT(sb);
    if (sb && sb->len > 0) {
        Str found = StrrChr(sb->s, ch);
        if (found) {
            int pos = (int)(found - sb->s);
            ASSERT(pos >= 0);  /* if negative, string is WAY too long */
            return pos;
        }
    }
    return -1;
}

/**
 * Finds the first occurrence of s in sb. Returns the position where
 * the occurrence was found, or -1 if none was found, or if the string
 * buffer is empty.
 */
int STRBUF_Find(const StrBuf * sb, Str s)
{
    return STRBUF_FindFrom(sb, 0, s);
}

/**
 * Finds the first occurrence of s in sb, starting with the specified
 * position. Returns the position where the occurrence was found, or -1
 * none was found, the position is invalid or if the string buffer is 
 * empty.
 */
int STRBUF_FindFrom(const StrBuf * sb, int pos, Str s)
{
    if (sb->len > 0 && ((size_t)pos) < sb->len && s) {
        if (!*s) {
            return 0;  /* like strstr does */
        } else {
            Str found = StrStr(sb->s + pos, s);
            if (found) {
                ASSERT((size_t)(found - sb->s) < sb->len);
                pos = (int)(found - sb->s);
                ASSERT(pos >= 0); /* if negative, string is WAY too long */
                return pos;
            }
        }
    }
    return -1;
}

/**
 * Finds the last occurrence of s in sb. Returns the position where
 * the occurrence was found, or -1 if none was found, or if either
 * string is empty.
 */
int STRBUF_FindLast(const StrBuf * sb, Str s)
{
    if (sb->len > 0 && s && *s) {
        size_t slen = StrLen(s);
        if (sb->len >= slen) {
            Str ptr = sb->s + (sb->len-slen);
            while (ptr >= sb->s) {
                Str s1 = ptr;
                Str s2 = s;
                while (*s1 && *s2 && !(*s1-*s2)) s1++, s2++;
                if (!*s2) {
                    return (int)(ptr - sb->s);
                }
                ptr--;
            }
        }
    }
    return -1;
}

#ifndef __KERNEL__

/**
 * Tests if two strings are equal, ignoring the case. If any of them is NULL,
 * the result is False.
 */
Bool STRBUF_EqualsNoCase(const StrBuf * sb1, const StrBuf * sb2)
{
    if (sb1 && sb2 && sb1->s && sb2->s) {

        /* try some quick checks */
        if (sb1 == sb2) {
            return True;
        } else if (sb1->s == sb2->s) {
            return True;
        } else if (sb1->len != sb2->len) {
            return False;
        }

        /* strings may be equal; compare them */
        return BoolValue(StrCaseCmp(sb1->s, sb2->s) == 0);
    }
    ASSMSG("STRBUF_EqualsNoCase: invalid arguments!");
    return False;
}

/**
 * Tests if two strings are equal, ignoring the case. If any of them is NULL,
 * the result is False.
 */
Bool STRBUF_EqualsToNoCase(const StrBuf * sb, Str s)
{
    if (sb && s) {
        if (sb->len == 0) {
            return BoolValue(!s[0]);
        } else if (sb->s == s) {
            return True;
        } else {
            return BoolValue(StrCaseCmp(sb->s, s) == 0);
        }
    }
    ASSMSG("STRBUF_EqualsToNoCase: invalid arguments!");
    return False;
}

/**
 * Finds the first occurrence of s in sb, ignoring the case. Returns the
 * position where the occurrence was found, or -1 if none was found, or if
 * the string buffer is empty. The string comparison is case-insentive.
 */
int STRBUF_FindNoCase(const StrBuf * sb, Str s)
{
    return STRBUF_FindFromNoCase(sb, 0, s);
}

/**
 * Finds the first occurrence of s in sb, starting with the specified
 * position, ignoring the case. Returns the position where the occurrence
 * was found, or -1 none was found, the position is invalid or if the
 * string buffer is empty.  The string comparison is case-insentive.
 */
int STRBUF_FindFromNoCase(const StrBuf * sb, int pos, Str s)
{
    if (sb->len > 0 && ((size_t)pos) < sb->len && s) {
        if (!*s) {
            return 0;  /* like strstr does */
        } else {
            size_t len = StrLen(s);
            if (len+pos <= sb->len) {
                size_t i, max = sb->len - len;
                for (i=pos; i<=max; i++) {
                    if (StrnCaseCmp(sb->s + i, s, len) == 0) {
                        return (int)i;
                    }
                }
            }
        }
    }
    return -1;
}

/**
 * Finds the last occurrence of s in sb. Returns the position where
 * the occurrence was found, or -1 if none was found, or if either
 * string is empty. The string comparison is case-insentive.
 */
int STRBUF_FindLastNoCase(const StrBuf * sb, Str s)
{
    if (sb->len > 0 && s && *s) {
        size_t len = StrLen(s);
        if (sb->len >= len) {
            Str ptr = sb->s + (sb->len-len);
            while (ptr >= sb->s) {
                if (StrnCaseCmp(ptr, s, len) == 0) {
                    return (int)(ptr - sb->s);
                }
                ptr--;
            }
        }
    }
    return -1;
}

#endif /* __KERNEL__ */

/**
 * Replaces all occurrences of c1 with c2, returns number of replaced 
 * characters
 */
int STRBUF_Replace(StrBuf * sb, Char c1, Char c2)
{
    int n = 0;
    ASSERT(sb && c1 && c2);
    if (sb && c1 && c2 && (c1 != c2)) {
        size_t i;
        for (i=0; i<sb->len; i++) {
            if (sb->s[i] == c1) {
                sb->s[i] = c2;
                n++;
            }
        }
    }
    return n;
}

/**
 * Returns all occurrences of substring 'from' to 'to'. Returns
 * the number of replacements made, or -(n+1) if an error occured,
 * where n is the number of successful replacements (we don't
 * undo the damage).
 */
int STRBUF_ReplaceStr(StrBuf * sb, Str from, Str to)
{
    int n = 0;
    Char* found;
    size_t len1 = StrLen(from);
    size_t len2 = StrLen(to);
    if (len1 > len2) {
        /* start from the end to reduce the amount of data to copy */
        size_t pos = sb->len;
        while (pos > 0) {
            size_t savelen = sb->len;
            sb->len = pos;
            pos = STRBUF_FindLast(sb, from);
            sb->len = savelen;
            if (pos >= 0) {
                /* get rid of extra characters */
                STRBUF_Erase(sb, pos+len2, pos+len1);
                memcpy(sb->s+pos, to, sizeof(Char)*len2);
                n++;
            }
        }
    } else if (len1 < len2) {
        int pos = 0;
        while (pos >= 0) {
            pos = STRBUF_FindFrom(sb, pos, from);
            if (pos >= 0) {
                /* insert extra characters */
                STRBUF_Insert(sb, to+len1, pos+len1);
                memcpy(sb->s+pos, to, sizeof(Char)*len1);
                pos = (int)(pos + len2);
                n++;
            }
        }
    } else if (len1) {
        size_t off = 0;
        while ((found = StrStr(sb->s+off,from)) != NULL) {
            n++;
            memcpy(found, to, sizeof(Char)*len2);
            off = (found - sb->s) + len2;
            if (off > sb->len-len1) {
                break;
            }
        }
    }
    return n;
}

/**
 * Tests if this string starts with the specified prefix.
 */
STATIC Bool STRBUF_StartsWithOffset(const StrBuf * sb, size_t off, Str s)
{
    ASSERT(sb);
    if (sb) {
        if (off <= sb->len) {
            size_t i = off;
            while (i<sb->len && *s) {
                if (sb->s[i++] != *s++) {
                    return False;
                }
            }
            return BoolValue(!(*s));
        }
    }
    return False;
}

/**
 * Tests if this string starts with the specified prefix.
 */
Bool STRBUF_StartsWith(const StrBuf * sb, Str s)
{
    return STRBUF_StartsWithOffset(sb, 0, s);
}

/**
 * Tests if this string ends with the specified suffix.
 */
Bool STRBUF_EndsWith(const StrBuf * sb, Str s)
{
    ASSERT(sb);
    if (s && sb) {
        size_t len = StrLen(s);
        if (len <= sb->len) {
            return STRBUF_StartsWithOffset(sb, sb->len - len, s);
        }
    }
    return False;
}

#ifndef __KERNEL__
/**
 * Tests if this string starts with the specified prefix. Ignores the case.
 */
STATIC Bool STRBUF_StartsWithOffsetNoCase(const StrBuf* sb, size_t off, Str s)
{
    ASSERT(sb);
    if (sb) {
        if (off <= sb->len) {
            size_t i = off;
            while (i<sb->len && *s) {
                if (sb->s[i++] != *s++) {
                    /* switch to more expensive case-insensitive algorithm */
                    size_t len1 = sb->len-(--i);
                    size_t len2 = StrLen(s--)+1;
                    if (len1 >= len2) {
                        if (StrnCaseCmp(sb->s+i,s,len2) == 0) {
                            return True;
                        }
                    }
                    return False;
                }
            }
            return BoolValue(!(*s));
        }
    }
    return False;
}

/**
 * Tests if this string starts with the specified prefix. Ignores the case.
 */
Bool STRBUF_StartsWithNoCase(const StrBuf * sb, Str s)
{
    return STRBUF_StartsWithOffsetNoCase(sb, 0, s);
}

/**
 * Tests if this string ends with the specified suffix. Ignores the case.
 */
Bool STRBUF_EndsWithNoCase(const StrBuf * sb, Str s)
{
    ASSERT(sb);
    if (s && sb) {
        size_t len = StrLen(s);
        if (len <= sb->len) {
            return STRBUF_StartsWithOffsetNoCase(sb, sb->len - len, s);
        }
    }
    return False;
}
#endif /* __KERNEL__ */

/**
 * Converts all of the characters in this string to upper case
 */
void STRBUF_ToUpperCase(StrBuf * sb)
{
    size_t i;
    for (i=0; i<sb->len; i++) {
        sb->s[i] = (Char)ToUpper(sb->s[i]);
    }
}

/**
 * Converts all of the characters in this string to lower case
 */
void STRBUF_ToLowerCase(StrBuf * sb)
{
    size_t i;
    for (i=0; i<sb->len; i++) {
        sb->s[i] = (Char)ToLower(sb->s[i]);
    }
}

/**
 * Retruns string represented by this string buffer. If no memory has ever
 * been allocated by this string buffer, return statically allocated empty
 * string.
 */
Str STRBUF_GetString(const StrBuf * sb)
{
    ASSERT(sb);
    if (sb && sb->len && sb->s) {
        return sb->s;
    } 
    return TEXT("");
}

/**
 * Reallocate the buffer if necessary, to ensure that it may contain at 
 * least a string of specified length (i.e. minlen+1 characters).
 */
Bool STRBUF_Alloc(StrBuf * sb, size_t minlen)
{
    size_t minsize = minlen+1;
    size_t oldsize = sb->alloc;
    ASSERT(minlen != ((size_t)-1));
    if (minsize > oldsize) {
        Char* newbuf;
        size_t newsize = (oldsize * 3)/2 + 1;
        if (newsize < minsize) newsize = minsize;

        /* cannot realloc external buffer */
        if (!sb->s || (sb->s && (sb->s == sb->ext))) {
            newbuf = MEM_NewArray(Char,newsize);
            if (newbuf) {
                if (sb->len > 0) {
                    memcpy(newbuf, sb->s, sizeof(Char)*(sb->len + 1));
                } else {
                    newbuf[0] = 0;
                }
            } 
        } else {
#ifdef NO_REALLOC
            newbuf = MEM_NewArray(Char, newsize);
            if (newbuf && sb->s) {
                memcpy(newbuf, sb->s, sb->alloc * sizeof(Char));
                MEM_Free(sb->s);
            }
#else
            newbuf = (Char*)MEM_Realloc(sb->s, newsize * sizeof(Char));
#endif /* NO_REALLOC */
        }

        /* failed to allocate new buffer - bail out */
        if (!newbuf) return False;

        /* otherwise switch to the new buffer */
        sb->s = newbuf;
        sb->alloc = newsize;
    }
    return True;
}

/**
 * Allocates a NULL terminated string initialized with the current contents
 * of the string buffer. The caller is responsible for deallocating the
 * returned string with MEM_Free. Returns NULL on failure.
 */
Char * STRBUF_Dup(const StrBuf * sb)
{
    Char * copy = NULL;
    if (sb) {
        copy = MEM_NewArray(Char,sb->len+1);
        if (copy) {
            if (sb->len > 0) {
                StrCpy(copy, sb->s);
                ASSERT(StrLen(copy) == sb->len);
            } else {
                copy[0] = 0;
            }
        }
    }
    return copy;
}

/**
 * Copies a string into the buffer
 */
Bool STRBUF_Copy(StrBuf * sb, Str s)
{
    ASSERT(s);
    if (s) {
        STRBUF_Clear(sb);
        return STRBUF_Append(sb, s);
    }
    return False;
}

/**
 * Replace contents of the string buffer with the specified string.
 */
Bool STRBUF_CopyN(StrBuf * sb, Str s, size_t size)
{
    ASSERT(s);
    if (s) {
        STRBUF_Clear(sb);
        return STRBUF_AppendN(sb, s, size);
    }
    return False;
}

/**
 * Append a string to the buffer
 */
Bool STRBUF_Append(StrBuf * sb, Str s)
{
    if (s) {
        size_t len = StrLen(s);
        if (STRBUF_Alloc(sb, sb->len + len)) {
            StrCpy(sb->s + sb->len, s);
            sb->len += len;
            ASSERT(StrLen(sb->s) == sb->len);
            return True;
        }
    }
    return False;
}

/**
 * Append the contents of sb2 to sb1. This is faster than STRBUF_Append
 * because it does not need to calculate the length of the second string.
 * Also, memcpy is probably slightly faster than strcpy.
 */
Bool STRBUF_AppendBuf(StrBuf * sb1, const StrBuf * sb2)
{
    if (sb2) {
        /* allocate necessary amount of space */
        if (STRBUF_Alloc(sb1, sb1->len + sb2->len)) {
            memcpy(sb1->s + sb1->len, sb2->s, sizeof(Char)*sb2->len);
            sb1->len += sb2->len;
            sb1->s[sb1->len] = 0;
            ASSERT(StrLen(sb1->s) == sb1->len);
            return True;
        }
    }
    return False;
}

/**
 * Append no more than N characters to the buffer
 */
Bool STRBUF_AppendN(StrBuf * sb, Str s, size_t max)
{
    if (s && max > 0) {

        /* calculate the number of bytes we are about to copy... */
        size_t n = 0;
        Str sp = s;
        while (n < max && *sp++) n++;

        /* allocate necessary amount of space */
        if (STRBUF_Alloc(sb, sb->len + n)) {
            memcpy(sb->s + sb->len, s, sizeof(Char)*n);
            sb->len += n;
            sb->s[sb->len] = 0;
            ASSERT(StrLen(sb->s) == sb->len);
            return True;
        }
    }

    /* always return True if appending 0 characters */
    return BoolValue(max == 0);
}

/**
 * Append an integer (string representation of it) to the buffer
 */
Bool STRBUF_AppendInt(StrBuf * sb, int i)
{
    Char buf[32];
    Char* s = STRING_Format(buf, COUNT(buf), TEXT("%d"), i);
    Bool success = STRBUF_Append(sb, s);
    if (s != buf) MEM_Free(s);
    return success;
}

/**
 * Append a character to the buffer.
 */
Bool STRBUF_AppendChar(StrBuf * sb, Char c)
{
    if (STRBUF_Alloc(sb, sb->len+1)) {
        sb->s[sb->len++] = c;
        sb->s[sb->len] = 0;
        return True;
    }
    return False;
}

/**
 * Append an boolean (string representation of it) to the buffer
 */
Bool STRBUF_AppendBool(StrBuf * sb, Bool b)
{
    Str s = (b ? TRUE_STRING : FALSE_STRING);
    return STRBUF_Append(sb, s);
}

/**
 * Append an double (string representation of it) to the buffer
 */
#ifndef __KERNEL__
Bool STRBUF_AppendDouble(StrBuf * sb, double d)
{
    Char buf[64];
    Char* s = STRING_Format(buf, COUNT(buf), TEXT("%f"), d);
    Bool success = STRBUF_Append(sb, s);
    if (s != buf) MEM_Free(s);
    return success;
}

/**
 * Append time to the buffer in the form "Wed Jun 30 21:49:08 1993 EST"
 */
Bool STRBUF_AppendTime(StrBuf * sb, Time t)
{
    Str s = TIME_ToString(t);
    if (s) {
        if (STRBUF_Append(sb, s)) {
            Bool success = True;
#if !defined(_WIN32_WCE) && !defined(__SYMBIAN32__)
            static Bool tzisset = False;
            if (!tzisset) {
                tzisset = True;
                tzset();
            }
            if (tzname[0]) {
                if (STRBUF_AppendChar(sb, ' ')) {
#  ifdef UNICODE
                    Char * tmp = STRING_ToUnicode(tzname[0]);
                    if (tmp) {
                        success = STRBUF_Append(sb, tmp);
                        MEM_Free(tmp);
                    }
#  else  /* !UNICODE */
                    success = STRBUF_Append(sb, tzname[0]);
#  endif /* !UNICODE */
                }
            }
#endif /* !_WIN32_WCE */
            return success;
        }
    }
    return False;
}

/**
 * Formats time in the form "Wed Jun 30 21:49:08 1993 EST"
 */
Bool STRBUF_FormatTime(StrBuf * sb, Time t)
{
    STRBUF_Clear(sb);
    return STRBUF_AppendTime(sb, t);
}
#endif /* !__KERNEL__ */

/**
 * Appends the specified character to the string until its length
 * reaches specified minimum length. If len parameter is less than 
 * the string length, the string is not modified (including the case 
 * if len is negative) 
 */
Bool STRBUF_Inflate(StrBuf * sb, size_t len, Char fill)
{
    Bool success = True;
    if (sb->len < len) {
        success = STRBUF_Alloc(sb, sb->len + len);
        if (success) {
            memset(sb->s + sb->len, fill, sizeof(Char)*(len - sb->len));
            sb->s[len] = 0;
            sb->len = len;
        }
    }
    return success;
}

/**
 * Inserts string at specified position. The length of the string is
 * given as a parameter.
 */
Bool STRBUF_InsertN(StrBuf * sb, size_t pos, Str s, size_t len)
{
    ASSERT(pos <= sb->len);
    if (pos <= sb->len) {
        if (s && len > 0) {
            DEBUG_ONLY(size_t i; for (i=0; i<len; i++) ASSERT(s[i]));
            if (STRBUF_Alloc(sb, sb->len + len)) {
                memmove(sb->s+pos+len,sb->s+pos,sizeof(Char)*(sb->len-pos+1));
                memmove(sb->s+pos, s, sizeof(Char)*len);
                sb->len += len;
            } else {
                return False;
            }
        }
        return True;
    }
    return False;
}

/**
 * Inserts string at specified position.
 */
Bool STRBUF_Insert(StrBuf * sb, Str s, size_t pos)
{
    return STRBUF_InsertN(sb, pos, s, StrLen(s));
}

/**
 * Inserts character at specified position. Inserrting NULL character has
 * no effect.
 */
Bool STRBUF_InsertChar(StrBuf * sb, Char c, size_t pos)
{
    ASSERT(c);  /* should not insert NULL character */
    return STRBUF_InsertN(sb, pos, &c, 1);
}

/**
 * Store formatted message in the buffer. Appends the formatted string to
 * the current contents of the buffer.
 */
Bool STRBUF_AppendFormatVa(StrBuf * sb, Str format, va_list va)
{
    if (!sb->s || (sb->alloc <= (sb->len+1))) {
        if (!STRBUF_Alloc(sb, sb->len + 100)) {
            return False;
        }
    }

    for (;;)  {
        size_t avail = sb->alloc - sb->len;
        size_t newsize = sb->alloc;
        int nchars;

        /* try to print in the allocated space. */
#ifdef va_copy
        va_list va2;
        va_copy(va2, va);
        nchars = Vsnprintf(sb->s + sb->len, avail, format, va2);
        va_end(va2);
#else   /* va_copy */
        /* NOTE: reusing va_list is risky and may not work... */
        nchars = Vsnprintf(sb->s + sb->len, avail, format, va);
#endif  /* va_copy */

        /* if that worked, we are done. */
        if (nchars > -1 && nchars < (int)avail) {
            sb->len += nchars;
            break;
        }

        /* else try again with more space. */
        if (nchars > -1) {
            newsize = sb->len + nchars;      /* precisely what is needed */
        } else {
            newsize *= 2;                    /* twice the old size */
        }

        /* if memory allocation fails, break the loop */
        if (!STRBUF_Alloc(sb, newsize)) {
            sb->s[sb->len] = 0;
            return False;
        }
    }

    /* 
     * vsnprintf in NT kernel environment doesn't always NULL-terminate
     * the formatted string. That seems to happen when using %wZ specified
     * to format a UNICODE_STRING that contains non-printable characters.
     */
#ifndef _NT_KERNEL
    ASSERT(sb->len == StrLen(sb->s));
    sb->s[sb->len] = 0;
#endif
    return True;
}

/**
 * Store formatted message in the buffer.
 */
Bool STRBUF_FormatVa(StrBuf * sb, Str format, va_list va)
{
    STRBUF_Clear(sb);
    return STRBUF_AppendFormatVa(sb, format, va);
}

/**
 * Store formatted message in the buffer.
 */
Bool STRBUF_Format(StrBuf * sb, Str format, ...)
{
    Bool ok;
    va_list va;
    va_start(va, format);
    ok = STRBUF_FormatVa(sb, format, va);
    va_end(va);
    return ok;
}

/**
 * Append formatted message in the buffer.
 */
Bool STRBUF_AppendFormat(StrBuf * sb, Str format, ...)
{
    Bool ok;
    va_list va;
    va_start(va, format);
    ok = STRBUF_AppendFormatVa(sb, format, va);
    va_end(va);
    return ok;
}

/*==========================================================================*
 *              U T F - 8   C O N V E R S I O N
 *==========================================================================*/
#ifdef _UNICODE

Bool STRBUF_CopyUTF8(StrBuf * sb, const char * utf8)
{
    ASSERT(utf8);
    if (utf8) {
        STRBUF_Clear(sb);
        return STRBUF_AppendUTF8(sb, utf8);
    }
    return False;
}

Bool STRBUF_AppendUTF8(StrBuf * sb, const char * utf8)
{
    if (utf8) {
        const size_t len = UTF8_Length(utf8);
        if (len != UTF8_ERROR && STRBUF_Alloc(sb, sb->len + len)) {
            int n = 0;
            const char * src = utf8;
            size_t bytesleft = strlen(utf8);
            wchar_t * dest = sb->s + sb->len;
            while (*src && (n = UTF8_DecodeChar(src, &bytesleft, dest)) > 0) {
                src += n;
                dest++;
            }
            ASSERT(!bytesleft);
            if (n >= 0) {
                *dest = 0;
                sb->len += len;
                return True;
            }
        }
    }
    return False;
}

#endif /* _UNICODE */

/*
 * HISTORY:
 *
 * $Log: s_strbuf.c,v $
 * Revision 1.63  2011/02/16 18:25:50  slava
 * o fixed a bug in STRBUF_AppendUTF8 (affected the case if resize is needed)
 *
 * Revision 1.62  2010/10/25 09:49:00  slava
 * o use va_copy (if available) when we need to reuse va_list
 *
 * Revision 1.61  2010/07/04 09:44:30  slava
 * o need to check for UTF8_Length failure in STRBUF_AppendUTF8 to handle
 *   the case of broken UTF-8 strings
 *
 * Revision 1.60  2010/07/03 12:13:27  slava
 * o more UTF-8 stuff
 *
 * Revision 1.59  2010/03/10 21:16:25  slava
 * o added STRBUF_EqualsNoCase and STRBUF_EqualsToNoCase functions
 *
 * Revision 1.58  2010/02/15 17:54:42  slava
 * o changed STRBUF_Dup to better handle empty strings - it would crash on a
 *   freshly initialized string buffer
 *
 * Revision 1.57  2009/12/26 12:21:32  slava
 * o working on 64-bit issues
 *
 * Revision 1.56  2009/04/13 18:25:07  slava
 * o reduced number of warnings in 64-bit build
 *
 * Revision 1.55  2006/07/17 20:58:23  slava
 * o workaround against strange behavior of vsnprintf in NT kernel environment
 *
 * Revision 1.54  2005/10/19 21:39:58  slava
 * o fixed a bug in STRBUF_FindLast
 * o added STRBUF_FindLastNoCase function
 *
 * Revision 1.53  2005/08/26 21:11:39  slava
 * o added STRBUF_StartsWithNoCase and STRBUF_EndsWithNoCase functions
 *
 * Revision 1.52  2005/08/24 01:19:20  slava
 * o added STRBUF_FindNoCase and STRBUF_FindFromNoCase functions
 *
 * Revision 1.51  2005/02/20 20:31:30  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.50  2005/01/23 16:02:43  slava
 * o added STRBUF_Dup function
 *
 * Revision 1.49  2004/12/31 01:53:55  slava
 * o added STRBUF_AppendBuf function
 *
 * Revision 1.48  2004/11/08 06:56:03  slava
 * o new functions: STRBUF_InitBuf2, STRBUF_Find, STRBUF_FindFrom,
 *   STRBUF_FindLast, STRBUF_ReplaceStr and STRBUF_InsertN
 *
 * Revision 1.47  2004/10/20 00:38:15  slava
 * o use const pointer if we are not planning to modify the string
 *
 * Revision 1.46  2004/04/08 01:44:17  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.45  2004/03/25 19:06:00  slava
 * o fixed a few pedantic compilation warnings
 *
 * Revision 1.44  2003/11/30 02:49:58  slava
 * o port to Linux kernel mode environment
 *
 * Revision 1.43  2003/07/28 05:04:49  slava
 * o added STRBUF_Create and STRBUF_Delete functions
 *
 * Revision 1.42  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.41  2003/03/18 16:52:44  slava
 * o added Unicode build configurations for Windoze
 *
 * Revision 1.40  2003/02/13 23:15:54  slava
 * o fixed a bug STRBUF_EqualsTo - empty string buffer wasn't always
 *   equal to an empty string
 *
 * Revision 1.39  2002/11/23 05:33:03  slava
 * o fixed a bug in STRBUF_AppendN - it was reading one byte more than necessary
 *   from the input buffer. If the input buffer ends exactly at the end of the
 *   page and the next page is not mapped, this could cause a crash.
 *
 * Revision 1.38  2002/08/27 11:52:03  slava
 * o some reformatting
 *
 * Revision 1.37  2002/07/08 06:56:41  slava
 * o fixed a bug in STRBUF_AppendN
 *
 * Revision 1.36  2002/07/01 04:19:54  slava
 * o no time zone information on WinCE (yet)
 *
 * Revision 1.35  2002/07/01 02:39:20  slava
 * o compile STRBUF_AppendTime and STRBUF_FormatTime on WinCE
 *
 * Revision 1.34  2002/05/31 06:56:08  slava
 * o added STRBUF_ToUpperCase and STRBUF_ToLowerCase
 *
 * Revision 1.33  2002/05/22 04:04:55  slava
 * o renamed s_buf files into s_strbuf, renamed STRBUF_Read into
 *   FILE_ReadLine and moved it to s_file module.
 *
 * Revision 1.32  2002/02/25 16:03:49  slava
 * o optimized STRBUF_Read to avoid unnecessary memory allocations
 *
 * Revision 1.31  2002/01/22 05:04:58  slava
 * o call tzset() before using tzname for the first time
 *
 * Revision 1.30  2001/12/31 01:03:23  slava
 * o added another ASSERT
 *
 * Revision 1.29  2001/12/23 18:41:16  slava
 * o moved STRING functions to s_str.c
 * o added STRING_ToMultiByte and STRING_ToUnicode
 *
 * Revision 1.28  2001/12/20 10:44:31  slava
 * o port to Windows CE
 *
 * Revision 1.27  2001/12/20 02:47:33  slava
 * o added STRING_StartsWith and STRING_EndsWith
 *
 * Revision 1.26  2001/12/07 03:26:40  slava
 * o added STRBUF_Replace() function
 *
 * Revision 1.25  2001/12/01 06:52:44  slava
 * o do not ASSERT if STRBUF_Destroy() is called twice for the same object
 *
 * Revision 1.24  2001/11/10 11:42:49  slava
 * o added STRBUF_Trim()
 *
 * Revision 1.23  2001/11/09 11:52:58  slava
 * o quite a few bug fixes. This only affects UNICODE build (i.e. WinCE)
 *
 * Revision 1.22  2001/10/22 00:25:36  slava
 * o fixed a bug in STRBUF_Read
 *
 * Revision 1.21  2001/10/09 05:40:15  slava
 * o STRBUF_AppendN() new returns True when asked to append 0 characters
 *
 * Revision 1.20  2001/10/08 05:17:51  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.19  2001/08/06 07:10:34  slava
 * o fixed a Unicode specific bug (affects Windows CE build)
 *
 * Revision 1.18  2001/06/27 03:01:51  slava
 * o slightly optimized STRING_HashCode() and STRING_HashCodeNoCase()
 *
 * Revision 1.17  2001/06/27 01:56:34  slava
 * o added STRING_HashCode() and STRING_HashCodeNoCase() functions
 *
 * Revision 1.16  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.15  2001/05/27 11:45:51  slava
 * o port to NT kernel mode environment
 *
 * Revision 1.14  2001/05/18 22:36:57  slava
 * o THIS_FILE variable is no longer being used
 *
 * Revision 1.13  2001/05/18 22:29:54  slava
 * o slib has been (partially) ported to Windows CE
 *
 * Revision 1.12  2001/01/03 09:18:23  slava
 * o new file I/O support for transparent access to plain or compressed
 *   files, sockets, etc.
 *
 * Revision 1.11  2000/12/25 04:39:27  slava
 * o added STRBUF_Equals() and STRBUF_EqualsTo() functions and
 *   STRBUF_INIT() macro
 *
 * Revision 1.10  2000/11/17 17:05:18  slava
 * o fixed bug in STRBUF_AppendN (assumption was made that buffer
 *   allocated STRBUF_Alloc() was always null-terminated, but it
 *   wasn't the case if a fresh buffer was realloc'ed rather than
 *   malloc'ed). Fixed STRBUF_AppendN not to make this assumption
 *   and STRBUF_Alloc() to malloc fresh buffers
 *
 * Revision 1.9  2000/11/17 05:24:04  slava
 * o STRBUF_CopyFromN() has been renamed into STRBUF_CopyN()
 * o STRBUF_CopyFrom() and STRBUF_Copy() were doing the same thing -
 *   removed STRBUF_CopyFrom()
 *
 * Revision 1.8  2000/11/12 11:45:22  slava
 * o fixed bug in STRBUF_AppendFormatVa() which could try to reallocate
 *   a bad (allocated on stack) memory block
 *
 * Revision 1.7  2000/11/01 13:25:05  slava
 * o replaced TRUE/FALSE with True/False
 *
 * Revision 1.6  2000/09/18 07:04:09  slava
 * o new functions: STRBUF_AppendFormat(), STRBUF_AppendFormatVa() and
 *   STRBUF_FormatVa()
 *
 * Revision 1.5  2000/09/16 03:43:55  slava
 * o fixed compilation warning produced by gcc -03
 *
 * Revision 1.4  2000/09/03 00:34:50  slava
 * o added STRBUF_AppendN() function
 *
 * Revision 1.3  2000/09/02 13:18:34  slava
 * o added STRBUF_IndexOf() and STRBUF_LastIndexOf()
 *
 * Revision 1.2  2000/08/19 05:07:58  slava
 * o fixed "signed/unsigned mismatch" warnings issued by Microsoft
 *   VC 6.0 compiler
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
