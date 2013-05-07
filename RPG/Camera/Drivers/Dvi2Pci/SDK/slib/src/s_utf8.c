/*
 * $Id: s_utf8.c,v 1.5 2011/02/17 08:38:23 slava Exp $
 *
 * Copyright (C) 2009-2011 by Slava Monich
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
 * From rfc2044: encoding of the Unicode values on UTF-8:
 *
 * UCS-4 range (hex.)    UTF-8 octet sequence (binary)
 * 0000 0000-0000 007F   0xxxxxxx
 * 0000 0080-0000 07FF   110xxxxx 10xxxxxx
 * 0000 0800-0000 FFFF   1110xxxx 10xxxxxx 10xxxxxx 
 *
 * 0001 0000-001F FFFF   11110xxx 10xxxxxx 10xxxxxx 10xxxxxx
 * 0020 0000-03FF FFFF   111110xx 10xxxxxx 10xxxxxx 10xxxxxx 10xxxxxx
 * 0400 0000-7FFF FFFF   1111110x 10xxxxxx ... 10xxxxxx
 *
 * NOTE: wchar_t can be either 2 or 4 byte long on different platforms.
 * For example, Linux is (usually) 4 bytes and Win32 - 2 bytes.
 *==========================================================================*/

/**
 * Returns the size (in bytes) of UTF-8 encoded NULL terminated Unicode
 * string, not counting the NULL terminator.
 */
size_t UTF8_Size(const wchar_t * ws) 
{
    size_t size = 0;
    if (ws) {
        while (*ws) {
            const I32u utf4 = *ws++;
            if (utf4 < 0x80) {
                size++;
            } else if (utf4 < 0x0800) {
                size += 2;
            } else if (utf4 < 0x00010000) {
                size += 3;
            } else if (utf4 < 0x00200000) {
                size += 4;
            } else if (utf4 < 0x04000000) {
                size += 5;
            } else if (utf4 < 0x80000000) {
                size += 6;
            } else {
                return 0;
            }
        }
    }
    return size;
}

/**
 * Converts a Unicode character into UTF-8. This algorithm works under
 * the assumption that all surrogate pairs have already been converted
 * into scalar code point values within the argument. Returns number of
 * bytes successfully writtent to the output buffer.
 */
size_t UTF8_EncodeChar(char * utf8, size_t bufsize, wchar_t wc)
{
    size_t bytes = 0;
    const I32u ucs4 = wc;
    if (ucs4 < 0x80) {
        if (bufsize >= 1) {
            utf8[bytes++] = (char)ucs4;
        }
    } else if (ucs4 < 0x0800) {
        if (bufsize >= 2) {
            utf8[bytes++] = (char)(( ucs4 >> 6)          | 0xC0);
            utf8[bytes++] = (char)(( ucs4        & 0x3F) | 0x80);
        }
    } else if (ucs4 < 0x00010000) {
        if (bufsize >= 3) {
            utf8[bytes++] = (char)(( ucs4 >> 12)         | 0xE0);
            utf8[bytes++] = (char)(((ucs4 >> 6)  & 0x3F) | 0x80);
            utf8[bytes++] = (char)(( ucs4        & 0x3F) | 0x80);
        }
    } else if (ucs4 < 0x00200000) {
        if (bufsize >= 4) {
            utf8[bytes++] = (char)(( ucs4 >> 18)         | 0xF0);
            utf8[bytes++] = (char)(((ucs4 >> 12) & 0x3F) | 0x80);
            utf8[bytes++] = (char)(((ucs4 >> 6)  & 0x3F) | 0x80);
            utf8[bytes++] = (char)(( ucs4        & 0x3F) | 0x80);
        }
    } else if (ucs4 < 0x04000000) {
        if (bufsize >= 5) {
            utf8[bytes++] = (char)(( ucs4 >> 24)         | 0xF8);
            utf8[bytes++] = (char)(((ucs4 >> 18) & 0x3F) | 0x80);
            utf8[bytes++] = (char)(((ucs4 >> 12) & 0x3F) | 0x80);
            utf8[bytes++] = (char)(((ucs4 >> 6)  & 0x3F) | 0x80);
            utf8[bytes++] = (char)(( ucs4        & 0x3F) | 0x80);
        }
    } else { /* (ucs4 < 0x80000000) */
        if (bufsize >= 6) {
            utf8[bytes++] = (char)(( ucs4 >> 30)         | 0xFC);
            utf8[bytes++] = (char)(((ucs4 >> 24) & 0x3F) | 0x80);
            utf8[bytes++] = (char)(((ucs4 >> 18) & 0x3F) | 0x80);
            utf8[bytes++] = (char)(((ucs4 >> 12) & 0x3F) | 0x80);
            utf8[bytes++] = (char)(((ucs4 >> 6)  & 0x3F) | 0x80);
            utf8[bytes++] = (char)(( ucs4        & 0x3F) | 0x80);
        }
    }
    return bytes;
}

/**
 * Converts a null terminated Unicode string into UTF-8 and appends
 * encoded data to the buffer. Returns True if the entire string has
 * been successfully converted and written into the buffer, False
 * otherwise. Note that the data is appended to the buffer, i.e. the
 * existing contents is preserved. The NULL terminator is NOT written
 * to the output buffer.
 */
Bool UTF8_EncodeBuf(Buffer* buf, const wchar_t * ws)
{
    Bool ok = True;
    size_t total = 0;
    while (*ws && ok) {
        char utf8[UTF8_MAX_SIZE];
        size_t bytes = UTF8_EncodeChar(utf8, sizeof(utf8), *ws++);
        if (bytes > 0) {
            total += bytes;
            ok = BUFFER_Put(buf, utf8, bytes, False);
        } else {
            ok = False;
        }
    }
    if (!ok) {
        /* Undo the damage */
        BUFFER_Unput(buf, total);
    }
    return ok;
}

/**
 * Converts a null terminated Unicode string into UTF-8. Returns
 * number of bytes successfully converted.
 */
size_t UTF8_Encode2(char* utf8, size_t bufsize, const wchar_t * ws)
{
    size_t total = 0;
    while (*ws && bufsize) {
        size_t bytes = UTF8_EncodeChar(utf8, bufsize, *ws++);
        if (bytes > 0) {
            utf8 += bytes;
            total += bytes;
            bufsize -= bytes;
        }
    }
    return total;
}

/**
 * Converts a null terminated Unicode string into UTF-8. Returns NULL
 * if the argument is NULL, Unicode string contains an invalid value or
 * memory allocation fails. The caller must deallocate returned string
 * with MEM_Free.
 */
char * UTF8_Encode(const wchar_t * ws) 
{
    char * utf8 = NULL;
    if (ws) {

        /* allocate a byte array of the necessary size */
        size_t size = UTF8_Size(ws);
        utf8 = MEM_NewArray(char, size+1);
        if (utf8) {

            /* do the conversion from character code points to utf-8 */
            DEBUG_ONLY(size_t bytes = )UTF8_Encode2(utf8, size, ws);
            ASSERT(bytes == size);
            utf8[size] = 0;
        }
    }
    return utf8;
}

/**
 * Returns number of bytes used by a UTF8 character (1...6), -1 if 'utf8' 
 * parameter does not point to a valid UTF8 sequence or zero if size of the 
 * character appears to be larger than the amount of data available. 
 *
 * If 'maxsize' parameter is negative, the string is assumed to be NULL
 * terminated. Note: if *utf8 points to a NULL character, this function
 * returns 1.
 */
int UTF8_CharSize(const char * utf8, size_t maxlen) 
{
    ASSERT(maxlen >= 0);
    ASSERT(utf8);
    if (utf8) {
        if (maxlen == 0) {
            return 0;
        } else {
            register unsigned char c = utf8[0];
            if (c & 0x80) {
                if (maxlen == 1) {  
                    return 0; /* not enough data */
                }
                if ((utf8[1] & 0xc0) != 0x80) {
                    return (-1); /* invalid UTF8 sequence */
                }
                if ((c & 0xe0) == 0xe0) {
                    if (maxlen == 2) {
                        return 0; /* not enough data */
                    }
                    if ((utf8[2] & 0xc0) != 0x80) {
                        return (-1); /* invalid UTF8 sequence */
                    }
                    if ((c & 0xf0) == 0xf0) {
                        if (maxlen == 3) {
                            return 0; /* not enough data */
                        }
                        if ((utf8[3] & 0xc0) != 0x80) {
                            return (-1); /* invalid UTF8 sequence */
                        }
                        if ((c & 0xf8) == 0xf8) {
                            if (maxlen == 4) {
                                return 0; /* not enough data */
                            }
                            if ((utf8[4] & 0xc0) != 0x80) {
                                return (-1); /* invalid UTF8 sequence */
                            }
                            if ((c & 0xfc) == 0xfc) {
                                if (maxlen == 5) {
                                    return 0; /* not enough data */
                                }
                                if ((utf8[5] & 0xc0) != 0x80) {
                                    return (-1); /* invalid UTF8 sequence */
                                }
                                if ((c & 0xfe) == 0xfc) {
                                    return 6;    /* 6-byte code */
                                } else {
                                    return (-1); /* invalid UTF8 sequence */
                                }
                            } else if ((c & 0xfc) == 0xf8) {
                                return 5;    /* 5-byte code */
                            } else {
                                return (-1); /* invalid UTF8 sequence */
                            }
                        } else if ((c & 0xf8) == 0xf0) {
                            return 4;    /* 4-byte code */
                        } else {
                            return (-1); /* invalid UTF8 sequence */
                        }
                    } else if ((c & 0xf0) == 0xe0) {
                        return 3;    /* 3-byte code */
                    } else {
                        return (-1); /* invalid UTF8 sequence */
                    }
                } else if ((c & 0xe0) == 0xc0) {
                    return 2;    /* 2-byte code */
                } else {
                    return (-1); /* invalid UTF8 sequence */
                }
            } else {
                return 1;  /* 1-byte code */
            }
        }
    }
    return (-1);
}

/**
 * Decodes one UTF8 character. 
 * If succeeds, returns numeber of bytes used, puts the wide character 
 * into *wc, and  updates *len subtracting the number of bytes used from 
 * the original value. Returns zero if there's not enough data in the input 
 * buffer to read the next UTF8 character. If there's an error in UTF8
 * sequence, returns -1.
 */
int UTF8_DecodeChar(const char * utf8, size_t * bufsiz, wchar_t * wc)
{
    if (wc) *wc = 0;

    ASSERT(utf8);
    ASSERT(bufsiz);

    if (utf8 && bufsiz && *bufsiz > 0) {
        int size  = UTF8_CharSize(utf8, *bufsiz);
        if (size <= (sizeof(wchar_t)*8+7)/6) {
            I32u ch = 0;
            switch (size) {
            case 6:
                ch  = (utf8[0] & 0x01) << 30;
                ch |= (utf8[1] & 0x3f) << 24;
                ch |= (utf8[2] & 0x3f) << 18;
                ch |= (utf8[3] & 0x3f) << 12;
                ch |= (utf8[4] & 0x3f) << 6;
                ch |= (utf8[5] & 0x3f);
                break;

            case 5:
                ch  = (utf8[0] & 0x03) << 24;
                ch |= (utf8[1] & 0x3f) << 18;
                ch |= (utf8[2] & 0x3f) << 12;
                ch |= (utf8[3] & 0x3f) << 6;
                ch |= (utf8[4] & 0x3f);
                break;

            case 4:
                ch  = (utf8[0] & 0x07) << 18;
                ch |= (utf8[1] & 0x3f) << 12;
                ch |= (utf8[2] & 0x3f) << 6;
                ch |= (utf8[3] & 0x3f);
                break;

            case 3:
                ch  = (utf8[0] & 0x0f) << 12;
                ch |= (utf8[1] & 0x3f) << 6;
                ch |= (utf8[2] & 0x3f);
                break;

            case 2:
                ch  = (utf8[0] & 0x1f) << 6;
                ch |= (utf8[1] & 0x3f);
                break;

            case 1:
                ch = utf8[0];
                break;

            case 0:
                return 0;

            default:
                return (-1);
            }

            *bufsiz -= size;
            if (wc) *wc = (wchar_t)ch;
            return size;
        }
    }
    return (-1);
}

/**
 * Returns number of characters in a NULL terminated UTF8 string
 * or UTF8_ERROR if it's not a valid UTF8 string. Returns zero if
 * the argument is NULL.
 */
size_t UTF8_Length(const char * utf8)
{
    size_t len = 0;
    if (utf8) {
        size_t maxlen = strlen(utf8);
        while (*utf8) {
            int charsize = UTF8_CharSize(utf8, maxlen);
            if (charsize <= 0) {
                return UTF8_ERROR;
            }
            len++;
            utf8 += charsize;
            maxlen -= charsize;
        }
    }
    return len;
}

/**
 * Converts a UTF-8 encoded string into null terminated Unicode string.
 * If number of characters of -1, the input string is assumed to be NULL
 * terminated.
 */
wchar_t * UTF8_Decode(const char * utf8)
{
    size_t len = UTF8_Length(utf8);
    if (len != UTF8_ERROR) {
        wchar_t * ws = MEM_NewArray(wchar_t,len+1);
        if (ws) {
            int n = 0;
            const char * src = utf8;
            size_t bytesleft = strlen(utf8);
            wchar_t * dest = ws;
            while (*src && (n = UTF8_DecodeChar(src, &bytesleft, dest)) > 0) {
                src += n;
                dest++;
            }
            ASSERT(!bytesleft);
            if (n >= 0) {
                *dest = 0;
                return ws;
            }
            MEM_Free(ws);
        }
    }
    return NULL;
}

/*
 * HISTORY:
 *
 * $Log: s_utf8.c,v $
 * Revision 1.5  2011/02/17 08:38:23  slava
 * o fixed a bug in UTF8_Encode2
 *
 * Revision 1.4  2010/12/19 17:59:16  slava
 * o added UTF8_EncodeChar and UTF8_EncodeBuf functions
 *
 * Revision 1.3  2010/07/04 13:49:57  slava
 * o fixed gcc compilation warning in release build
 *
 * Revision 1.2  2010/07/03 12:13:27  slava
 * o more UTF-8 stuff
 *
 * Revision 1.1  2010/07/03 09:42:42  slava
 * o added UTF-8 utilities
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
