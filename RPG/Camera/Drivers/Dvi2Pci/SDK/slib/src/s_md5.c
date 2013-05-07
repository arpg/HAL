/*
 * $Id: s_md5.c,v 1.5 2009/11/07 09:12:53 slava Exp $
 *
 * Copyright (C) 2000-2009 by Slava Monich
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

#include "s_mdp.h"
#include "s_mem.h"
#include "s_buf.h"              /* defines ntohl macro */

/* The MD-5 block size, in bytes */
#define MD5_DATASIZE    64

/* MD-5 context */
typedef struct _Md5 {
    Digest md;                      /* used by the message digest framework */
    I64u count;                     /* 64-bit bit count */
    I32u digest[MD5_DIGEST_SIZE/4]; /* Scratch buffer */
    I32u data[MD5_DATASIZE/4];      /* MD5 data buffer */
} Md5;

/* Message digest callbacks */
static void MD5_Init   P_((Digest * d));
static void MD5_Update P_((Digest * d, const void * data, size_t size));
static void MD5_Finish P_((Digest * d, void * out));
static void MD5_Free   P_((Digest * d));

/* MD-5 message digest type definition */
static const DigestType Md5Digest = {
    TEXT("MD-5")        /* name */,
    MD5_DIGEST_SIZE     /* size */,
    MD5_Init            /* init */,
    MD5_Update          /* update */,
    MD5_Finish          /* finish*/,
    MD5_Free            /* free */
};

/* RFC 1321 assumes little endian byte order */
#if BYTE_ORDER == BIG_ENDIAN
#  define MD5_LittleEndian(_x) DATA_Conv32(_x,LITTLE_ENDIAN,BYTE_ORDER)
#elif BYTE_ORDER == LITTLE_ENDIAN
#  define MD5_LittleEndian(_x) (_x)
#else
#  error "Please fix BYTE_ORDER"
#endif /* BYTE_ORDER */

/* MD-5 magic */
#define F(x, y, z) (((x) & (y)) | ((~x) & (z)))
#define G(x, y, z) (((x) & (z)) | ((y) & (~z)))
#define H(x, y, z) ((x) ^ (y) ^ (z))
#define I(x, y, z) ((y) ^ ((x) | (~z))) 

#define ROTATE_LEFT(x, n) (((x) << (n)) | ((x) >> (32-(n))))

#define FF(a, b, c, d, x, s, ac) \
   (a) += F ((b), (c), (d)) + (x) + (I32u)(ac); \
   (a) = ROTATE_LEFT((a), (s)); \
   (a) += (b);

#define GG(a, b, c, d, x, s, ac) \
   (a) += G ((b), (c), (d)) + (x) + (I32u)(ac); \
   (a) = ROTATE_LEFT((a), (s)); \
   (a) += (b);

#define HH(a, b, c, d, x, s, ac) \
   (a) += H ((b), (c), (d)) + (x) + (I32u)(ac); \
   (a) = ROTATE_LEFT((a), (s)); \
   (a) += (b);

#define II(a, b, c, d, x, s, ac) \
   (a) += I ((b), (c), (d)) + (x) + (I32u)(ac); \
   (a) = ROTATE_LEFT((a), (s)); \
   (a) += (b);

#define S11 7
#define S12 12
#define S13 17
#define S14 22
#define S21 5
#define S22 9
#define S23 14
#define S24 20
#define S31 4
#define S32 11
#define S33 16
#define S34 23
#define S41 6
#define S42 10
#define S43 15
#define S44 21

/**
 * Basic MD-5 step 
 */
static void MD5_Transform(I32u * buf, I32u * in)
{
    I32u a = buf[0];
    I32u b = buf[1];
    I32u c = buf[2];
    I32u d = buf[3];

#if BYTE_ORDER == BIG_ENDIAN
    int i;
    for (i=0; i<MD5_DATASIZE/4; i++) {
        register I32u x = in[i];
        in[i] = ((x >> 24) | 
                ((x & 0x00ff0000) >> 8) | 
                ((x & 0x0000ff00) << 8) | 
                ((x & 0x000000ff) << 24));
    }
#endif /* BYTE_ORDER == BIG_ENDIAN */

    /* Round 1 */
    FF( a, b, c, d, in[ 0], S11, 0xd76aa478); /* 1 */
    FF( d, a, b, c, in[ 1], S12, 0xe8c7b756); /* 2 */
    FF( c, d, a, b, in[ 2], S13, 0x242070db); /* 3 */
    FF( b, c, d, a, in[ 3], S14, 0xc1bdceee); /* 4 */
    FF( a, b, c, d, in[ 4], S11, 0xf57c0faf); /* 5 */
    FF( d, a, b, c, in[ 5], S12, 0x4787c62a); /* 6 */
    FF( c, d, a, b, in[ 6], S13, 0xa8304613); /* 7 */
    FF( b, c, d, a, in[ 7], S14, 0xfd469501); /* 8 */
    FF( a, b, c, d, in[ 8], S11, 0x698098d8); /* 9 */
    FF( d, a, b, c, in[ 9], S12, 0x8b44f7af); /* 10 */
    FF( c, d, a, b, in[10], S13, 0xffff5bb1); /* 11 */
    FF( b, c, d, a, in[11], S14, 0x895cd7be); /* 12 */
    FF( a, b, c, d, in[12], S11, 0x6b901122); /* 13 */
    FF( d, a, b, c, in[13], S12, 0xfd987193); /* 14 */
    FF( c, d, a, b, in[14], S13, 0xa679438e); /* 15 */
    FF( b, c, d, a, in[15], S14, 0x49b40821); /* 16 */

    /* Round 2 */
    GG( a, b, c, d, in[ 1], S21, 0xf61e2562); /* 17 */
    GG( d, a, b, c, in[ 6], S22, 0xc040b340); /* 18 */
    GG( c, d, a, b, in[11], S23, 0x265e5a51); /* 19 */
    GG( b, c, d, a, in[ 0], S24, 0xe9b6c7aa); /* 20 */
    GG( a, b, c, d, in[ 5], S21, 0xd62f105d); /* 21 */
    GG( d, a, b, c, in[10], S22,  0x2441453); /* 22 */
    GG( c, d, a, b, in[15], S23, 0xd8a1e681); /* 23 */
    GG( b, c, d, a, in[ 4], S24, 0xe7d3fbc8); /* 24 */
    GG( a, b, c, d, in[ 9], S21, 0x21e1cde6); /* 25 */
    GG( d, a, b, c, in[14], S22, 0xc33707d6); /* 26 */
    GG( c, d, a, b, in[ 3], S23, 0xf4d50d87); /* 27 */
    GG( b, c, d, a, in[ 8], S24, 0x455a14ed); /* 28 */
    GG( a, b, c, d, in[13], S21, 0xa9e3e905); /* 29 */
    GG( d, a, b, c, in[ 2], S22, 0xfcefa3f8); /* 30 */
    GG( c, d, a, b, in[ 7], S23, 0x676f02d9); /* 31 */
    GG( b, c, d, a, in[12], S24, 0x8d2a4c8a); /* 32 */

    /* Round 3 */
    HH( a, b, c, d, in[ 5], S31, 0xfffa3942); /* 33 */
    HH( d, a, b, c, in[ 8], S32, 0x8771f681); /* 34 */
    HH( c, d, a, b, in[11], S33, 0x6d9d6122); /* 35 */
    HH( b, c, d, a, in[14], S34, 0xfde5380c); /* 36 */
    HH( a, b, c, d, in[ 1], S31, 0xa4beea44); /* 37 */
    HH( d, a, b, c, in[ 4], S32, 0x4bdecfa9); /* 38 */
    HH( c, d, a, b, in[ 7], S33, 0xf6bb4b60); /* 39 */
    HH( b, c, d, a, in[10], S34, 0xbebfbc70); /* 40 */
    HH( a, b, c, d, in[13], S31, 0x289b7ec6); /* 41 */
    HH( d, a, b, c, in[ 0], S32, 0xeaa127fa); /* 42 */
    HH( c, d, a, b, in[ 3], S33, 0xd4ef3085); /* 43 */
    HH( b, c, d, a, in[ 6], S34,  0x4881d05); /* 44 */
    HH( a, b, c, d, in[ 9], S31, 0xd9d4d039); /* 45 */
    HH( d, a, b, c, in[12], S32, 0xe6db99e5); /* 46 */
    HH( c, d, a, b, in[15], S33, 0x1fa27cf8); /* 47 */
    HH( b, c, d, a, in[ 2], S34, 0xc4ac5665); /* 48 */

    /* Round 4 */
    II( a, b, c, d, in[ 0], S41, 0xf4292244); /* 49 */
    II( d, a, b, c, in[ 7], S42, 0x432aff97); /* 50 */
    II( c, d, a, b, in[14], S43, 0xab9423a7); /* 51 */
    II( b, c, d, a, in[ 5], S44, 0xfc93a039); /* 52 */
    II( a, b, c, d, in[12], S41, 0x655b59c3); /* 53 */
    II( d, a, b, c, in[ 3], S42, 0x8f0ccc92); /* 54 */
    II( c, d, a, b, in[10], S43, 0xffeff47d); /* 55 */
    II( b, c, d, a, in[ 1], S44, 0x85845dd1); /* 56 */
    II( a, b, c, d, in[ 8], S41, 0x6fa87e4f); /* 57 */
    II( d, a, b, c, in[15], S42, 0xfe2ce6e0); /* 58 */
    II( c, d, a, b, in[ 6], S43, 0xa3014314); /* 59 */
    II( b, c, d, a, in[13], S44, 0x4e0811a1); /* 60 */
    II( a, b, c, d, in[ 4], S41, 0xf7537e82); /* 61 */
    II( d, a, b, c, in[11], S42, 0xbd3af235); /* 62 */
    II( c, d, a, b, in[ 2], S43, 0x2ad7d2bb); /* 63 */
    II( b, c, d, a, in[ 9], S44, 0xeb86d391); /* 64 */

    buf[0] += a;
    buf[1] += b;
    buf[2] += c;
    buf[3] += d;
}

/**
 * Initializes the MD5 values
 */
static void MD5_Init(Digest* d)
{
    Md5 * md = CAST(d,Md5,md);

    d->type = &Md5Digest;
    d->flags = MD_INITIAL;

    /* Set the h-vars to their initial values */
    md->digest[0] = 0x67452301;
    md->digest[1] = 0xefcdab89;
    md->digest[2] = 0x98badcfe;
    md->digest[3] = 0x10325476;

    /* Initialise the bit count */
    md->count = 0;
}

/**
 * Updates MD5 for a block of data. 
 */
static void MD5_Update(Digest * d, const void * in, size_t count)
{
    Md5 * md = CAST(d, Md5, md);
    const I8u * buffer = (I8u*)in;

    /* Get count of bytes already in data */
    size_t dataCount = (((I32u)md->count) >> 3) & 0x3F;

    /* Update bitcount */
    md->count += (((I64u)count) << 3);

    /* Handle any leading odd-sized chunks */
    if (dataCount) {
        I8u * p = ((I8u*)md->data) + dataCount;
        dataCount = MD5_DATASIZE - dataCount;
        if (count < dataCount) {
            memcpy(p, buffer, count);
            return;
        }
        memcpy(p, buffer, dataCount);
        MD5_Transform(md->digest, md->data);
        buffer += dataCount;
        count -= dataCount;
    }

    /* Process data in MD5_DATASIZE chunks */
    while (count >= MD5_DATASIZE) {
        memcpy(md->data, buffer, MD5_DATASIZE);
        MD5_Transform(md->digest, md->data);
        buffer += MD5_DATASIZE;
        count -= MD5_DATASIZE;
    }

    /* Handle any remaining bytes of data. */
    memcpy(md->data, buffer, count);
}

/**
 * Completes the MD-5 calculation
 */
static void MD5_Finish(Digest * d, void * out)
{
    Md5 * md = CAST(d, Md5, md);

#if BYTE_ORDER == BIG_ENDIAN
    I8u * output = (I8u*)out;
    int i, j;
#endif /* BYTE_ORDER == BIG_ENDIAN */

    /* Compute number of bytes mod 64 */
    int count = (((I32u)md->count) >> 3) & 0x3F;

    /* Set the first char of padding to 0x80.  This is safe since there is
     * always at least one byte free */
    I8u *dataPtr = ((I8u*)md->data) + count;
    *dataPtr++ = 0x80;

    /* Bytes of padding needed to make 64 bytes */
    count = MD5_DATASIZE - 1 - count;

    /* Pad out to 56 mod 64 */
    if (count < 8) {

        /* Pad the first block to 64 bytes */
        memset(dataPtr, 0, count);
        MD5_Transform(md->digest, md->data);

        /* Now fill the next block with 56 bytes */
        memset(md->data, 0, MD5_DATASIZE - 8);
    } else {
        /* Pad block to 56 bytes */
        memset(dataPtr, 0, count - 8);
    }

    /* Append length in bits and transform */
    md->data[14] = MD5_LittleEndian((I32u)md->count);
    md->data[15] = MD5_LittleEndian((I32u)(md->count >> 32));
    MD5_Transform(md->digest, md->data);

    /* Output to an array of bytes */
#if BYTE_ORDER == BIG_ENDIAN
    for(i=0, j=0; j<MD5_DIGEST_SIZE; i++, j+=4) {
        register I32u x = md->digest[i];
        output[j  ] = (I8u)x;
        output[j+1] = (I8u)(x >> 8);
        output[j+2] = (I8u)(x >> 16);
        output[j+3] = (I8u)(x >> 24);
    }
#elif BYTE_ORDER == LITTLE_ENDIAN
    memcpy(out, md->digest, sizeof(md->digest));
#else
#  error "Please fix BYTE_ORDER"
#endif /* BYTE_ORDER */

    /* Wipe out sensitive stuff */
    memset(md->digest, 0, sizeof(md->digest));
    memset(md->data, 0, sizeof(md->data));
}

/**
 * Deallocates the MD-5 object
 */
static void MD5_Free(Digest * d)
{
    MEM_Free(CAST(d,Md5,md));
}

/**
 * Allocates the MD-5 digest object
 */
Digest * MD5_Create()
{
    Md5 * md = MEM_New(Md5);
    if (md) MD5_Init(&md->md);
    return &md->md;
}

/** 
 * Does init/update/finish in one shot. There must be at least 
 * MD5_DIGEST_SIZE bytes available in the output buffer
 */
void MD5_Digest(const void * in, size_t n, void * out)
{
    Md5 md;
    MD5_Init(&md.md);
    MD5_Update(&md.md, in, n);
    MD5_Finish(&md.md, out);
}

/*
 * HISTORY:
 *
 * $Log: s_md5.c,v $
 * Revision 1.5  2009/11/07 09:12:53  slava
 * o replaced a few more int's with size_t
 *
 * Revision 1.4  2009/04/09 21:17:17  slava
 * o user size_t instead of int where it's appropriate
 *
 * Revision 1.3  2004/10/20 00:18:50  slava
 * o fixed Unicode compilation issues
 *
 * Revision 1.2  2004/07/31 07:43:07  slava
 * o fixed byte order issues
 *
 * Revision 1.1  2004/07/31 07:27:03  slava
 * o implemented MD-5 digest (RFC 1321)
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
