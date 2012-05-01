/*
 * $Id: s_base32.c,v 1.9 2011/03/03 15:28:46 slava Exp $
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

#include "s_base32.h"
#include "s_mem.h"

typedef struct _Base32Decode {
    File * in;              /* input stream */
    int nread;              /* number of input characters read */
    Buffer * out;           /* output buffer */
    const I8u* decodeMap;   /* decode map */
    int flags;              /* flags, see below */

#define FLAG_ERROR  0x01    /* bad BASE32 sequence was encountered */
#define FLAG_PAD    0x02    /* reading the padding */

} Base32Decode;

#define ENCODE_CHUNK_SIZE 5 /* 5 bytes of binary data */
#define DECODE_CHUNK_SIZE 8 /* get encoded into 8 printable characters */

/**
 * Macros for BASE32 encoding
 */
#define BASE32_ENCODE_1(_buf,_off)   (((_buf)[_off]     >> 3) & 0x1F)

#define BASE32_ENCODE_2_1(_buf,_off) (((_buf)[_off]     << 2) & 0x1C)
#define BASE32_ENCODE_2_2(_buf,_off) (((_buf)[(_off)+1] >> 6) & 0x03)
#define BASE32_ENCODE_2(_buf,_off)   (BASE32_ENCODE_2_1(_buf,_off) | \
                                      BASE32_ENCODE_2_2(_buf,_off))

#define BASE32_ENCODE_3(_buf,_off)   (((_buf)[(_off)+1] >> 1) & 0x1F)

#define BASE32_ENCODE_4_1(_buf,_off) (((_buf)[(_off)+1] << 4) & 0x10)
#define BASE32_ENCODE_4_2(_buf,_off) (((_buf)[(_off)+2] >> 4) & 0x0F)
#define BASE32_ENCODE_4(_buf,_off)   (BASE32_ENCODE_4_1(_buf,_off) | \
                                      BASE32_ENCODE_4_2(_buf,_off))

#define BASE32_ENCODE_5_1(_buf,_off) (((_buf)[(_off)+2] << 1) & 0x1E)
#define BASE32_ENCODE_5_2(_buf,_off) (((_buf)[(_off)+3] >> 7) & 0x01)
#define BASE32_ENCODE_5(_buf,_off)   (BASE32_ENCODE_5_1(_buf,_off) | \
                                      BASE32_ENCODE_5_2(_buf,_off))

#define BASE32_ENCODE_6(_buf,_off)   (((_buf)[(_off)+3] >> 2) & 0x1F)

#define BASE32_ENCODE_7_1(_buf,_off) (((_buf)[(_off)+3] << 3) & 0x18)
#define BASE32_ENCODE_7_2(_buf,_off) (((_buf)[(_off)+4] >> 5) & 0x07)
#define BASE32_ENCODE_7(_buf,_off)   (BASE32_ENCODE_7_1(_buf,_off) | \
                                      BASE32_ENCODE_7_2(_buf,_off))

#define BASE32_ENCODE_8(_buf,_off)   ((_buf)[(_off) +4]       & 0x1F)

/**
 * Macros for BASE32 decoding
 */
#define BASE32_DECODE_1_1(_c1)       ((char)(((_c1) << 3) & 0xF8)) 
#define BASE32_DECODE_1_2(_c2)       ((char)(((_c2) >> 2) & 0x07))
#define BASE32_DECODE_1(_c1,_c2)     ((char)(BASE32_DECODE_1_1(_c1) | \
                                            BASE32_DECODE_1_2(_c2)))

#define BASE32_DECODE_2_1(_c2)       ((char)(((_c2) << 6) & 0xC0)) 
#define BASE32_DECODE_2_2(_c3)       ((char)(((_c3) << 1) & 0x3E))
#define BASE32_DECODE_2_3(_c4)       ((char)(((_c4) >> 4) & 0x01))
#define BASE32_DECODE_2(_c2,_c3,_c4) ((char)(BASE32_DECODE_2_1(_c2) | \
                                             BASE32_DECODE_2_2(_c3) | \
                                             BASE32_DECODE_2_3(_c4)))

#define BASE32_DECODE_3_1(_c4)       ((char)(((_c4) << 4) & 0xF0)) 
#define BASE32_DECODE_3_2(_c5)       ((char)(((_c5) >> 1) & 0x0F))
#define BASE32_DECODE_3(_c4,_c5)     ((char)(BASE32_DECODE_3_1(_c4) | \
                                             BASE32_DECODE_3_2(_c5)))

#define BASE32_DECODE_4_1(_c5)       ((char)(((_c5) << 7) & 0x80)) 
#define BASE32_DECODE_4_2(_c6)       ((char)(((_c6) << 2) & 0x7C))
#define BASE32_DECODE_4_3(_c7)       ((char)(((_c7) >> 3) & 0x03))
#define BASE32_DECODE_4(_c5,_c6,_c7) ((char)(BASE32_DECODE_4_1(_c5) | \
                                             BASE32_DECODE_4_2(_c6) | \
                                             BASE32_DECODE_4_3(_c7)))

#define BASE32_DECODE_5_1(_c7)       ((char)(((_c7) << 5) & 0xE0)) 
#define BASE32_DECODE_5_2(_c8)       ((char)(_c8))
#define BASE32_DECODE_5(_c7,_c8)     ((char)(BASE32_DECODE_5_1(_c7) | \
                                             BASE32_DECODE_5_2(_c8)))

/**
 * The decoding maps are only 128 bytes long. We need to watch for the
 * input characters that are greater than 127
 */
#define BASE32_GetChunk(_c,_m) (((_c) & 0x80) ? (-1) : ((int)(char)((_m)[_c])))

/**
 * The character to value map for the standard BASE32 encoding based
 * on RFC3548.
 */
STATIC const char base32_upperCaseEncodeMap[64] = {
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', /* 0-7 */
    'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', /* 8-15 */
    'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', /* 16-23 */
    'Y', 'Z', '2', '3', '4', '5', '6', '7'  /* 24-31 */
};

STATIC const char base32_lowerCaseEncodeMap[64] = {
    'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', /* 0-7 */
    'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', /* 8-15 */
    'q', 'r', 's', 't', 'u', 'v', 'w', 'x', /* 16-23 */
    'y', 'z', '2', '3', '4', '5', '6', '7'  /* 24-31 */
};

STATIC const I8u base32_caseSensitiveDecodeMap[128] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF,
    0xFF, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 
    0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
    0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 
    0x17, 0x18, 0x19, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

STATIC const I8u base32_caseInsensitiveDecodeMap[128] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF,
    0xFF, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 
    0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
    0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 
    0x17, 0x18, 0x19, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 
    0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
    0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 
    0x17, 0x18, 0x19, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

STATIC const Str base32_pad[5] = {
    TEXT(""),
    TEXT("======"),
    TEXT("===="),
    TEXT("==="),
    TEXT("=")
};

/**
 * Translates binary data into a BASE32 encoded string. The returned string 
 * is guaranteed to be a 7-bit ASCII string. If buffer is empty, returns an 
 * empty string. Only returns NULL if memory allocation fails. The caller will
 * have to deallocate the returned string with MEM_Free
 */
Char* BASE32_Encode(const void * data, size_t n, int flags)
{
    const char* encodeMap = ((flags & BASE32_LOWERCASE) ? 
        base32_lowerCaseEncodeMap : 
        base32_upperCaseEncodeMap);

    size_t alloc = (n/ENCODE_CHUNK_SIZE+((n%ENCODE_CHUNK_SIZE)?1:0))*
        DECODE_CHUNK_SIZE+1;

    /*
     *   +--- octet 0 ---+--- octet 1 ---+--- octet 2 ---+
     *   |7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|
     *   +---------+-----+---+---------+-+-------+-------+
     *   |4 3 2 1 0|4 3 2 1 0|4 3 2 1 0|4 3 2 1 0|4 3 2 1
     *   +-0.index-+-1.index-+-2.index-+-3.index-+-4.index
     *   
     *   +--- octet 3 ---+--- octet 4 ---+
     *   |7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|
     *   +-+---------+---+-----+---------+
     *    0|4 3 2 1 0|4 3 2 1 0|4 3 2 1 0|
     *   --+-5.index-+-6.index-+-7.index-+
     */

    Char * dest = MEM_NewArray(Char, alloc);
    if (dest) {
        const char * buf = (const char *)data;
        Char* p = dest;
        size_t off;

        /* encode the bulk of the data */
        for (off=0; (off+ENCODE_CHUNK_SIZE)<=n; off+=ENCODE_CHUNK_SIZE) {
            *p++ = encodeMap[BASE32_ENCODE_1(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_2(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_3(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_4(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_5(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_6(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_7(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_8(buf,off)];
        }

        /* manage the last few bytes */
        switch (n % ENCODE_CHUNK_SIZE) {
        case 0:
            break;
        case 1:
            *p++ = encodeMap[BASE32_ENCODE_1(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_2_1(buf,off)];
            break;
        case 2:
            *p++ = encodeMap[BASE32_ENCODE_1(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_2(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_3(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_4_1(buf,off)];
            break;
        case 3:
            *p++ = encodeMap[BASE32_ENCODE_1(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_2(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_3(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_4(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_5_1(buf,off)];
            break;
        case 4:
            *p++ = encodeMap[BASE32_ENCODE_1(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_2(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_3(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_4(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_5(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_6(buf,off)];
            *p++ = encodeMap[BASE32_ENCODE_7_1(buf,off)];
            break;
        }

        /* append padding characters */
        if (flags & BASE32_PAD) {
            Str pad = base32_pad[n % ENCODE_CHUNK_SIZE];
            while (*pad) *p++ = *pad++;
        }

        /* null terminate the destination string */
        *p = 0;
        ASSERT(StrLen(dest) < (size_t)alloc);
    }
    return dest;
}

/**
 * The same as BASE32_Encode but stores encoded data into a string buffer.
 * Does not destroy the original contents of the string buffer. The
 * Base32 string is appended to it. Returns pointer to string buffer
 * data, NULL on memory allocation failure
 */
Str BASE32_EncodeStr(const void * data, size_t n, StrBuf * sb, int flags)
{
    const char* encodeMap = ((flags & BASE32_LOWERCASE) ? 
        base32_lowerCaseEncodeMap : 
        base32_upperCaseEncodeMap);

    size_t alloc = (n/ENCODE_CHUNK_SIZE+((n%ENCODE_CHUNK_SIZE)?1:0))*
        DECODE_CHUNK_SIZE+1;

    /*
     *   +--- octet 0 ---+--- octet 1 ---+--- octet 2 ---+
     *   |7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|
     *   +---------+-----+---+---------+-+-------+-------+
     *   |4 3 2 1 0|4 3 2 1 0|4 3 2 1 0|4 3 2 1 0|4 3 2 1
     *   +-0.index-+-1.index-+-2.index-+-3.index-+-4.index
     *   
     *   +--- octet 3 ---+--- octet 4 ---+
     *   |7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|
     *   +-+---------+---+-----+---------+
     *    0|4 3 2 1 0|4 3 2 1 0|4 3 2 1 0|
     *   --+-5.index-+-6.index-+-7.index-+
     */

    if (STRBUF_Alloc(sb, STRBUF_Length(sb)+alloc)) {
        const char * buf = (const char *)data;
        size_t off;

        /* encode the bulk of the data */
        for (off=0; (off+ENCODE_CHUNK_SIZE)<=n; off+=ENCODE_CHUNK_SIZE) {
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_1(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_2(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_3(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_4(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_5(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_6(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_7(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_8(buf,off)]);
        }

        /* manage the last few bytes */
        switch (n % ENCODE_CHUNK_SIZE) {
        case 0:
            break;
        case 1:
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_1(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_2_1(buf,off)]);
            break;
        case 2:
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_1(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_2(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_3(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_4_1(buf,off)]);
            break;
        case 3:
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_1(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_2(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_3(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_4(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_5_1(buf,off)]);
            break;
        case 4:
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_1(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_2(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_3(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_4(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_5(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_6(buf,off)]);
            STRBUF_AppendChar(sb,encodeMap[BASE32_ENCODE_7_1(buf,off)]);
            break;
        }

        /* append padding characters */
        if (flags & BASE32_PAD) {
            STRBUF_Append(sb, base32_pad[n % ENCODE_CHUNK_SIZE]);
        }

        return STRBUF_Text(sb);
    }
    
    /* could not allocate memory */
    return NULL;    
}

/**
 * Reads a chunk from the input stream. Returns the number of bytes
 * recovered from the input stream, zero on end of file or -1 in case
 * of BASE32 format error.
 */
STATIC int BASE32_ReadChunk(Base32Decode * decode, I8u chunk[4])
{
    /* the index where the padding begins */
    int end = ((decode->flags & FLAG_PAD) ? 0 : -1);
    int n = 0;

    while (n < DECODE_CHUNK_SIZE) {
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
                    decode->flags |= FLAG_ERROR;
                    return -1;
                }
            }
            chunk[n++] = k;
        }
    }

    if (n < DECODE_CHUNK_SIZE) {
        int k = n;
        while (k < DECODE_CHUNK_SIZE) chunk[k++] = 0;
    }

    if (end >= 0) {
        return end;
    } else {
        return n;
    }
}

/**
 * Decodes a BASE32 encoded data block. Returns True if string has been 
 * successfully decoded, or False if it cannot be decoded (or memory
 * allocation fails). Does not destroy the original contents of the 
 * buffer
 */
STATIC Bool BASE32_InternalDecode(Base32Decode * decode)
{
    /* remember the original length so that we can undo the damage */
    Buffer * out = decode->out;
    size_t origLen = (out ? BUFFER_Size(out) : 0);

    /* read the bulk of the data */
    int n;
    Bool ok = True;
    I8u b[DECODE_CHUNK_SIZE];
    while ((n = BASE32_ReadChunk(decode, b)) == DECODE_CHUNK_SIZE) {
        if (out) {
            if (!BUFFER_PutByte(out,BASE32_DECODE_1(b[0],b[1])) ||
                !BUFFER_PutByte(out,BASE32_DECODE_2(b[1],b[2],b[3])) ||
                !BUFFER_PutByte(out,BASE32_DECODE_3(b[3],b[4])) ||
                !BUFFER_PutByte(out,BASE32_DECODE_4(b[4],b[5],b[6])) ||
                !BUFFER_PutByte(out,BASE32_DECODE_5(b[6],b[7]))) {
                /* out of memory */
                ok = False;
                break;
            }
        }
    }

    /*
     *   +--- octet 0 ---+--- octet 1 ---+--- octet 2 ---+
     *   |7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|
     *   +---------+-----+---+---------+-+-------+-------+
     *   |4 3 2 1 0|4 3 2 1 0|4 3 2 1 0|4 3 2 1 0|4 3 2 1
     *   +-0.index-+-1.index-+-2.index-+-3.index-+-4.index
     *   
     *   +--- octet 3 ---+--- octet 4 ---+
     *   |7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|
     *   +-+---------+---+-----+---------+
     *    0|4 3 2 1 0|4 3 2 1 0|4 3 2 1 0|
     *   --+-5.index-+-6.index-+-7.index-+
     */

    /* deal with the trailer */
    if (out && ok) {
        switch (n) {
        case 0:
            break;
        case 1:
        case 3:
        case 6:
            /* incorrect trailer */
            decode->flags |= FLAG_ERROR;
            break;
        case 2:
            if (!BUFFER_PutByte(out,BASE32_DECODE_1(b[0],b[1]))) {
                /* out of memory */
                ok = False;
            }
            break;
        case 4:
            if (!BUFFER_PutByte(out,BASE32_DECODE_1(b[0],b[1])) ||
                !BUFFER_PutByte(out,BASE32_DECODE_2(b[1],b[2],b[3]))) {
                /* out of memory */
                ok = False;
            }
            break;
        case 5:
            if (!BUFFER_PutByte(out,BASE32_DECODE_1(b[0],b[1])) ||
                !BUFFER_PutByte(out,BASE32_DECODE_2(b[1],b[2],b[3])) ||
                !BUFFER_PutByte(out,BASE32_DECODE_3(b[3],b[4]))) {
                /* out of memory */
                ok = False;
            }
            break;
        case 7:
            if (!BUFFER_PutByte(out,BASE32_DECODE_1(b[0],b[1])) ||
                !BUFFER_PutByte(out,BASE32_DECODE_2(b[1],b[2],b[3])) ||
                !BUFFER_PutByte(out,BASE32_DECODE_3(b[3],b[4])) ||
                !BUFFER_PutByte(out,BASE32_DECODE_4(b[4],b[5],b[6]))) {
                /* out of memory */
                ok = False;
            }
            break;
        }
    }

    /* return True if the input was a valid BASE32 sequence */
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
 * Decodes a BASE32 encoded data block. Returns True if string has been 
 * successfully decoded, or False if it cannot be decoded (or memory
 * allocation fails). The len parameter is negative if the amount of
 * data is unknown in advance.
 */
STATIC Bool BASE32_DecodeFile2(File * in, size_t len, Buffer * out,
    const I8u * map)
{
    Base32Decode decode;

    /* pre-allocate maximum amount of memory for the output buffer */
    if (out && len > 0) {
        size_t nbytes = ENCODE_CHUNK_SIZE*(len/DECODE_CHUNK_SIZE);
        switch (len % DECODE_CHUNK_SIZE) {
        case 0: 
        case 1: 
            break;
        case 2: 
        case 3: 
            nbytes++; 
            /* NOBREAK */
        case 4:
            nbytes++; 
            /* NOBREAK */
        case 5: 
        case 6: 
            nbytes++; 
            /* NOBREAK */
        case 7: 
            nbytes++;
            break;
        }
        if (!BUFFER_EnsureCapacity(out, BUFFER_Size(out) + nbytes, False)) {
            return False;
        }
    }

    /* initialize the context */
    decode.in = in;
    decode.nread = 0;
    decode.out = out;
    decode.decodeMap = map;
    decode.flags = 0;

    /* run the decoder */
    return BASE32_InternalDecode(&decode);
}

/**
 * Decodes a BASE32 encoded data block. Returns True if string has been 
 * successfully decoded, or False if it the input is not a valid BASE32
 * sequence cannot be decoded or if memory allocation fails.
 */
STATIC Bool BASE32_DecodeStr(Str base32, Buffer * out, const I8u* decodeMap)
{
    if (base32) {
        size_t len = StrLen(base32);
        if (len > 0) {
            Bool ok = False;
            File* in = FILE_MemIn(base32, len);
            if (in) {
                ok = BASE32_DecodeFile2(in, len, out, decodeMap);
                FILE_Close(in);
            }
            return ok;
        }
    }

    /* empty string is OK */
    return True;
}

/**
 * Decodes a BASE32 encoded data block (case insensitive)
 */
Bool BASE32_Decode(Str base32, Buffer * out)
{
    return BASE32_DecodeStr(base32, out, base32_caseInsensitiveDecodeMap);
}

/**
 * Decodes a BASE32 encoded data block assuming the standard BASE32 encoding
 * based on RFC3548 (case sensitive).
 */
Bool BASE32_StrictDecode(Str base32, Buffer * out)
{
    return BASE32_DecodeStr(base32, out, base32_caseSensitiveDecodeMap);
}

/**
 * Decodes a BASE32 encoded data block (case insensitive) reading characters 
 * from the file.
 */
Bool BASE32_DecodeFile(File * in, Buffer * out)
{
    return BASE32_DecodeFile2(in, -1, out, base32_caseInsensitiveDecodeMap);
}

/**
 * Decodes a BASE32 encoded data block assuming the standard BASE32 encoding
 * based on RFC3548 (case sensitive) reading characters from the file.
 */
Bool BASE32_StrictDecodeFile(File * in, Buffer * out)
{
    return BASE32_DecodeFile2(in, -1, out, base32_caseSensitiveDecodeMap);
}

/*
 * HISTORY:
 *
 * $Log: s_base32.c,v $
 * Revision 1.9  2011/03/03 15:28:46  slava
 * o a few minor fixes suggested by Apple's code analyzer
 *
 * Revision 1.8  2009/05/23 10:14:55  slava
 * o fixed 32-bit warnings introduced in the process of fixing 64 bit warnings
 *
 * Revision 1.7  2009/05/23 09:18:52  slava
 * o fixed a few more 64-bit warnings
 *
 * Revision 1.6  2009/04/09 21:59:20  slava
 * o fixed compilation warnings (signed vs unsigned)
 *
 * Revision 1.5  2009/04/09 21:54:56  slava
 * o use size_t instead of int where it's appropriate
 *
 * Revision 1.4  2004/10/29 19:28:21  slava
 * o fixed gcc compilation warnings
 *
 * Revision 1.3  2004/10/20 00:18:50  slava
 * o fixed Unicode compilation issues
 *
 * Revision 1.2  2004/08/19 01:29:18  slava
 * o fixed the function name (BASE32_EncodeStr2 -> BASE32_EncodeStr)
 *
 * Revision 1.1  2004/08/18 02:52:01  slava
 * o added support for BASE32 encoding
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
