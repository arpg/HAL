/*
 * $Id: s_base64.c,v 1.15 2011/03/03 15:28:46 slava Exp $
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

#include "s_base64.h"
#include "s_mem.h"

/* BASE64 decoding context */
typedef struct _Base64Decode {
    File * in;              /* input stream */
    int nread;              /* number of input characters read */
    Buffer * out;           /* output buffer */
    const I8u* decodeMap;   /* decode map */
    int flags;              /* flags, see below */

#define FLAG_DETECT 0x01    /* decode map has not been detected yet */
#define FLAG_ERROR  0x02    /* bad BASE64 sequence was encountered */
#define FLAG_PAD    0x04    /* reading the padding */

} Base64Decode;

/**
 * Macros for BASE64 encoding
 */
#define BASE64_ENCODE_1(_buf,_offset)   (((_buf)[(_offset)]     >> 2) & 0x3F)

#define BASE64_ENCODE_2_1(_buf,_offset) (((_buf)[(_offset)]     << 4) & 0x30)
#define BASE64_ENCODE_2_2(_buf,_offset) (((_buf)[(_offset) + 1] >> 4) & 0x0F)
#define BASE64_ENCODE_2(_buf,_offset)   (BASE64_ENCODE_2_1(_buf,_offset) | \
                                         BASE64_ENCODE_2_2(_buf,_offset))

#define BASE64_ENCODE_3_1(_buf,_offset) (((_buf)[(_offset) + 1] << 2) & 0x3C)
#define BASE64_ENCODE_3_2(_buf,_offset) (((_buf)[(_offset) + 2] >> 6) & 0x03)
#define BASE64_ENCODE_3(_buf,_offset)   (BASE64_ENCODE_3_1(_buf,_offset) | \
                                         BASE64_ENCODE_3_2(_buf,_offset))

#define BASE64_ENCODE_4(_buf,_offset)   ((_buf)[(_offset) + 2]        & 0x3F)

/**
 * Macros for BASE64 decoding
 */
#define BASE64_DECODE_1_1(_c1)      ((char)(((_c1) << 2) & 0xFC)) 
#define BASE64_DECODE_1_2(_c2)      ((char)(((_c2) >> 4) & 0x03))
#define BASE64_DECODE_1(_c1,_c2)    ((char)(BASE64_DECODE_1_1(_c1) | \
                                            BASE64_DECODE_1_2(_c2)))

#define BASE64_DECODE_2_1(_c2)      ((char)(((_c2) << 4) & 0xF0)) 
#define BASE64_DECODE_2_2(_c3)      ((char)(((_c3) >> 2) & 0x0F))
#define BASE64_DECODE_2(_c2,_c3)    ((char)(BASE64_DECODE_2_1(_c2) | \
                                            BASE64_DECODE_2_2(_c3)))

#define BASE64_DECODE_3_1(_c3)      ((char)(((_c3) << 6) & 0xC0)) 
#define BASE64_DECODE_3_2(_c4)      ((char)(((_c4)     ) & 0x3F))
#define BASE64_DECODE_3(_c3,_c4)    ((char)(BASE64_DECODE_3_1(_c3) | \
                                            BASE64_DECODE_3_2(_c4)))

#define BASE64_GetChunk(_c,_m) (((_c) & 0x80) ? (-1) : ((int)(char)((_m)[_c])))

/**
 * The character to value map for the standard BASE64 encoding based
 * on RFC1521.
 */
STATIC const char base64_encodeMap[64] = {
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',     /* 0-7 */
    'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',     /* 8-15 */
    'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',     /* 16-23 */
    'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',     /* 24-31 */
    'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',     /* 32-39 */
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v',     /* 40-47 */
    'w', 'x', 'y', 'z', '0', '1', '2', '3',     /* 48-55 */
    '4', '5', '6', '7', '8', '9', '+', '/',     /* 56-63 */
};

STATIC const I8u base64_decodeMap[128] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0x3E, 0xFF, 0xFF, 0xFF, 0x3F,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B,
    0x3C, 0x3D, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF,
    0xFF, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
    0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
    0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
    0x17, 0x18, 0x19, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20,
    0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30,
    0x31, 0x32, 0x33, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/**
 * The character to value map for the "URL and filename safe" BASE64 
 * encoding defined (or rather proposed) in RFC3548.
 */
STATIC const char base64_safeEncodeMap[64] = {
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',     /* 0-7 */
    'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',     /* 8-15 */
    'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',     /* 16-23 */
    'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',     /* 24-31 */
    'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',     /* 32-39 */
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v',     /* 40-47 */
    'w', 'x', 'y', 'z', '0', '1', '2', '3',     /* 48-55 */
    '4', '5', '6', '7', '8', '9', '-', '_',     /* 56-63 */
};

STATIC const I8u base64_safeDecodeMap[128] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3E, 0xFF, 0xFF,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B,
    0x3C, 0x3D, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF,
    0xFF, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
    0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
    0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
    0x17, 0x18, 0x19, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F,
    0xFF, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20,
    0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30,
    0x31, 0x32, 0x33, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/**
 * Translates a binary buffer into a BASE64 encoded string using the specified
 * character to value mapping. The returned string is guaranteed to be a 7-bit
 * ASCII string. If buffer is empty, returns an empty string. Only returns NULL
 * if memory allocation fails. The caller will have to deallocate the returned
 * string with MEM_Free
 */
Char * BASE64_Encode(const void * data, size_t n, int flags)
{
    const char* encodeMap = ((flags & BASE64_URLSAFE) ? 
        base64_safeEncodeMap : 
        base64_encodeMap);

    size_t alloc = ((n+2)/3)*4 + 1;
    Char * dest = MEM_NewArray(Char, alloc);
    if (dest) {
        const char * buf = (const char *)data;
        Char * p = dest;
        size_t off;

        /* encode the bulk of the data */
        for (off=0; (off+3)<=n; off+=3) {
            *p++ = encodeMap[BASE64_ENCODE_1(buf,off)];
            *p++ = encodeMap[BASE64_ENCODE_2(buf,off)];
            *p++ = encodeMap[BASE64_ENCODE_3(buf,off)];
            *p++ = encodeMap[BASE64_ENCODE_4(buf,off)];
        }

        /* manage the last few bytes */
        switch (n%3) {
        case 0:
            break;
        case 1:
            *p++ = encodeMap[BASE64_ENCODE_1(buf,off)];
            *p++ = encodeMap[BASE64_ENCODE_2_1(buf,off)];
            if (flags & BASE64_PAD) {
                *p++ = '=';
                *p++ = '=';
            }
            break;
        case 2:
            *p++ = encodeMap[BASE64_ENCODE_1(buf,off)];
            *p++ = encodeMap[BASE64_ENCODE_2(buf,off)];
            *p++ = encodeMap[BASE64_ENCODE_3_1(buf,off)];
            if (flags & BASE64_PAD) *p++ = '=';
            break;
        }

        /* null terminate the destination string */
        *p = 0;
        ASSERT(StrLen(dest) < (size_t)alloc);
    }
    return dest;
}

/**
 * The same as BASE64_Encode but stores encoded data into a string buffer.
 * Does not destroy the original contents of the string buffer. The
 * Base64 string is appended to it. Returns pointer to string buffer
 * data, NULL on memory allocation failure
 */
Str BASE64_EncodeStr(const void * data, size_t size, StrBuf * sb, int flags)
{
    const char* encodeMap = ((flags & BASE64_URLSAFE) ? 
        base64_safeEncodeMap : 
        base64_encodeMap);

    size_t alloc = ((size+2)/3)*4;
    if (STRBUF_Alloc(sb, STRBUF_Length(sb)+alloc)) {
        const char * buf = (const char *)data;
        size_t off;

        /* encode the bulk of the data */
        for (off=0; (off+3)<=size; off+=3) {
            STRBUF_AppendChar(sb,encodeMap[BASE64_ENCODE_1(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE64_ENCODE_2(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE64_ENCODE_3(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE64_ENCODE_4(buf,off)]);
        }

        /* manage last one or two bytes */
        switch (size%3) {
        case 0:
            break;
        case 1:
            STRBUF_AppendChar(sb,encodeMap[BASE64_ENCODE_1(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE64_ENCODE_2_1(buf,off)]);
            if (flags & BASE64_PAD) {
                STRBUF_AppendChar(sb,'=');
                STRBUF_AppendChar(sb,'=');
            }
            break;
        case 2:
            STRBUF_AppendChar(sb,encodeMap[BASE64_ENCODE_1(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE64_ENCODE_2(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE64_ENCODE_3_1(buf,off)]);
            if (flags & BASE64_PAD) STRBUF_AppendChar(sb,'=');
            break;
        }
        
        return STRBUF_Text(sb);
    }
    
    /* could not allocate memory */
    return NULL;    
}

/**
 * Reads a chunk from the input stream. Returns the number of bytes
 * recovered from the input stream, zero on end of file or -1 in case
 * of BASE64 format error.
 */
STATIC int BASE64_ReadChunk(Base64Decode * decode, I8u chunk[4])
{
    /* the index where the padding begins */
    int end = ((decode->flags & FLAG_PAD) ? 0 : -1);
    int n = 0;

    while (n < 4) {
        int nextChar = FILE_Getc(decode->in);
        if (nextChar < 0) {
            break;
        } else {
            I8u k;
            Char c = (Char)nextChar;

            /* count this character */
            decode->nread++;
            if ((c & (~0x7f)) != 0) {
                decode->flags |= FLAG_ERROR;
                return -1;
            }

            /* ignore whitespaces */
            if (IsSpace(c)) {
                continue;
            }

            if (c == '=') {
                if (end < 0) end = n;
                decode->flags |= FLAG_PAD;
                ASSERT(decode->decodeMap[(int)c] == 0);
                k = 0;
            } else {
                if (decode->flags & FLAG_PAD) {
                    /* junk after the padding */
                    decode->flags |= FLAG_ERROR;
                    return -1;
                }

                /* run it through the decoding map */
                k = decode->decodeMap[(int)c];
                if (k == 0xff) {
                    if (decode->flags & FLAG_DETECT) {
                        decode->flags &= (~FLAG_DETECT);
                        decode->decodeMap = base64_safeDecodeMap;
                        k = decode->decodeMap[(int)c];
                        if (k == 0xff) {
                            decode->flags |= FLAG_ERROR;
                            return -1;
                        }
                    } else {
                        decode->flags |= FLAG_ERROR;
                        return -1;
                    }
                }
            }
            chunk[n++] = k;
        }
    }

    if (n < 4) {
        int k = n;
        while (k < 4) chunk[k++] = 0;
    }

    if (end >= 0) {
        return end;
    } else {
        return n;
    }
}

/**
 * Decodes a BASE64 encoded data block. Returns True if string has been 
 * successfully decoded, or False if it cannot be decoded (or memory
 * allocation fails). Does not destroy the original contents of the 
 * buffer
 */
STATIC Bool BASE64_InternalDecode(Base64Decode * decode)
{
    /* remember the original length so that we can undo the damage */
    Buffer* out = decode->out;
    size_t origLen = (out ? BUFFER_Size(out) : 0);

    /* read the bulk of the data */
    int n;
    Bool ok = True;
    I8u chunk[4];
    while ((n = BASE64_ReadChunk(decode, chunk)) == 4) {
        if (out) {
            if (!BUFFER_PutByte(out,BASE64_DECODE_1(chunk[0],chunk[1])) ||
                !BUFFER_PutByte(out,BASE64_DECODE_2(chunk[1],chunk[2])) ||
                !BUFFER_PutByte(out,BASE64_DECODE_3(chunk[2],chunk[3]))) {
                /* out of memory */
                ok = False;
                break;
            }
        }
    }

    /* deal with the trailer */
    if (out && ok) {
        switch (n) {
        case 0:
            break;
        case 1:
            /* premature end of BASE64 stream */
            decode->flags |= FLAG_ERROR;
            break;
        case 2:
            if (!BUFFER_PutByte(out,BASE64_DECODE_1(chunk[0],chunk[1]))) {
                /* out of memory */
                ok = False;
            }
            break;
        case 3:
            if (!BUFFER_PutByte(out,BASE64_DECODE_1(chunk[0],chunk[1])) ||
                !BUFFER_PutByte(out,BASE64_DECODE_2(chunk[1],chunk[2]))) {
                /* out of memory */
                ok = False;
            }
            break;
        case 4:
            if (!BUFFER_PutByte(out,BASE64_DECODE_1(chunk[0],chunk[1])) ||
                !BUFFER_PutByte(out,BASE64_DECODE_2(chunk[1],chunk[2])) ||
                !BUFFER_PutByte(out,BASE64_DECODE_3(chunk[2],chunk[3]))) {
                /* out of memory */
                ok = False;
            }
            break;
        }
    }

    /* return True if the input was a valid BASE64 sequence */
    if ((decode->flags & FLAG_ERROR) || !ok) {
        if (out) {
            BUFFER_Unput(out,BUFFER_Size(out)-origLen);
            ASSERT(BUFFER_Size(out) == (int)origLen);
        }
        return False;
    } else {
        return True;
    }
}

/**
 * Decodes a BASE64 encoded data block. Returns True if string has been 
 * successfully decoded, or False if it cannot be decoded (or memory
 * allocation fails). The len parameter is negative if the amount of
 * data is unknown in advance.
 */
STATIC Bool BASE64_DecodeFile2(File * in, size_t len, Buffer * out,
    const I8u * map)
{
    Base64Decode decode;

    /* pre-allocate memory in the output buffer */
    if (out && len > 0) {
        size_t nbytes = 3*len/4;
        if (len%4) nbytes++;
        if (!BUFFER_EnsureCapacity(out, BUFFER_Size(out) + nbytes, False)) {
            return False;
        }
    }

    /* initialize the context */
    decode.in = in;
    decode.nread = 0;
    decode.out = out;
    if (map) {
        decode.decodeMap = map;
        decode.flags = 0;
    } else {
        decode.decodeMap = base64_decodeMap;
        decode.flags = FLAG_DETECT;
    }

    /* run the decoder */
    return BASE64_InternalDecode(&decode);
}

/**
 * Decodes a BASE64 encoded data block. Returns True if string has been 
 * successfully decoded, or False if it the input is not a valid BASE64
 * sequence cannot be decoded or if memory allocation fails.
 */
STATIC Bool BASE64_DecodeStr(Str base64, Buffer * out, const I8u* decodeMap)
{
    if (base64) {
        size_t len = StrLen(base64);
        if (len > 0) {
            Bool ok = False;
            File* in = FILE_MemIn(base64, len);
            if (in) {
                ok = BASE64_DecodeFile2(in, len, out, decodeMap);
                FILE_Close(in);
            }
            return ok;
        }
    }

    /* empty string is OK */
    return True;
}

/**
 * Decodes a BASE64 encoded data block allowing both the standard BASE64 
 * encoding based on RFC1521 and the "URL and filename safe" BASE64 encoding
 * defined in RFC3548
 */
Bool BASE64_Decode(Str base64, Buffer * out)
{
    return BASE64_DecodeStr(base64, out, NULL);
}

/**
 * Decodes a BASE64 encoded data block assuming the standard BASE64 encoding
 * based on RFC1521.
 */
Bool BASE64_StdDecode(Str base64, Buffer * out)
{
    return BASE64_DecodeStr(base64, out, base64_decodeMap);
}

/**
 * Decodes a BASE64 encoded data block assuming the "URL and filename safe"
 * BASE64 encoding defined in RFC3548
 */
Bool BASE64_SafeDecode(Str base64, Buffer * out)
{
    return BASE64_DecodeStr(base64, out, base64_safeDecodeMap);
}

/**
 * Decodes a BASE64 encoded data block allowing both the standard BASE64 
 * encoding based on RFC1521 and the "URL and filename safe" BASE64 encoding
 * defined in RFC3548
 */
Bool BASE64_DecodeFile(File * in, Buffer * out)
{
    return BASE64_DecodeFile2(in, -1, out, NULL);
}

/**
 * Decodes a BASE64 encoded data block assuming the standard BASE64 encoding
 * based on RFC1521.
 */
Bool BASE64_StdDecodeFile(File * in, Buffer * out)
{
    return BASE64_DecodeFile2(in, -1, out, base64_decodeMap);
}

/**
 * Decodes a BASE64 encoded data block assuming the "URL and filename safe"
 * BASE64 encoding defined in RFC3548
 */
Bool BASE64_SafeDecodeFile(File * in, Buffer * out)
{
    return BASE64_DecodeFile2(in, -1, out, base64_safeDecodeMap);
}

/*
 * HISTORY:
 *
 * $Log: s_base64.c,v $
 * Revision 1.15  2011/03/03 15:28:46  slava
 * o a few minor fixes suggested by Apple's code analyzer
 *
 * Revision 1.14  2009/05/23 10:14:55  slava
 * o fixed 32-bit warnings introduced in the process of fixing 64 bit warnings
 *
 * Revision 1.13  2009/05/23 10:05:06  slava
 * o continuing to fight compilation warnings...
 *
 * Revision 1.12  2009/05/23 09:18:52  slava
 * o fixed a few more 64-bit warnings
 *
 * Revision 1.11  2009/05/23 09:14:39  slava
 * o a few tweaks for x86_64 build
 *
 * Revision 1.10  2009/04/09 21:59:20  slava
 * o fixed compilation warnings (signed vs unsigned)
 *
 * Revision 1.9  2009/04/09 21:54:56  slava
 * o use size_t instead of int where it's appropriate
 *
 * Revision 1.8  2006/10/13 16:20:57  slava
 * o simpler way to calculate number of bytes required for BASE64 encoded data
 *
 * Revision 1.7  2004/10/29 19:28:21  slava
 * o fixed gcc compilation warnings
 *
 * Revision 1.6  2004/10/20 00:18:50  slava
 * o fixed Unicode compilation issues
 *
 * Revision 1.5  2004/08/18 02:49:37  slava
 * o fixed compilation warning
 *
 * Revision 1.4  2004/08/18 02:43:20  slava
 * o removed BASE64_SafeEncode and BASE64_SafeEncodeStr functions. Instead,
 *   added flags to BASE64_Encode and BASE64_EncodeStr functions. The flags
 *   allow you to select the type of BASE64 encoding and the padding option
 *
 * Revision 1.3  2004/07/31 18:23:48  slava
 * o fixed pedantic compilation warning
 *
 * Revision 1.2  2004/07/19 22:56:18  slava
 * o minor reformatting
 *
 * Revision 1.1  2004/07/19 22:55:11  slava
 * o moved BASE64 encoding functions from s_util to s_base64 module
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
