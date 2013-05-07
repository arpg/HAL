/*
 * $Id: s_sha1.c,v 1.6 2009/11/07 09:12:54 slava Exp $
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

/* The SHS block size, in bytes */
#define SHS_DATASIZE    64

/* SHA-1 context */
typedef struct _Sha1 {
    Digest md;                      /* used by the message digest framework */
    I64u count;                     /* 64-bit bit count */
    I32u digest[SHA1_DIGEST_SIZE/4];/* Message digest */
    I32u data[SHS_DATASIZE/4];      /* SHS data buffer */
} Sha1;

/* Message digest callbacks */
static void SHA1_Init   P_((Digest * d));
static void SHA1_Update P_((Digest * d, const void * data, size_t size));
static void SHA1_Finish P_((Digest * d, void * out));
static void SHA1_Free   P_((Digest * d));

/* SHA-1 message digest type definition */
static const DigestType Sha1Digest = {
    TEXT("SHA-1")       /* name */,
    SHA1_DIGEST_SIZE    /* size */,
    SHA1_Init           /* init */,
    SHA1_Update         /* update */,
    SHA1_Finish         /* finish*/,
    SHA1_Free           /* free */
};

/* The SHS f()-functions. */
/*#define F1(x,y,z) ((x & y)|(~x & z))          // Rounds  0-19 */
#define F1(x,y,z)   (z ^ (x & (y ^ z)))         /* Rounds  0-19 */
#define F2(x,y,z)   (x ^ y ^ z)                 /* Rounds 20-39 */
/*#define F3(x,y,z) ((x & y)|(x & z)|(y & z))   // Rounds 40-59 */
#define F3(x,y,z)   ((x & y)|(z & (x | y)))     /* Rounds 40-59 */
#define F4(x,y,z)   (x ^ y ^ z)                 /* Rounds 60-79 */

/* The SHS Mysterious Constants */
#define K1  0x5A827999L                         /* Rounds  0-19 */
#define K2  0x6ED9EBA1L                         /* Rounds 20-39 */
#define K3  0x8F1BBCDCL                         /* Rounds 40-59 */
#define K4  0xCA62C1D6L                         /* Rounds 60-79 */

/* SHS initial values */
#define SHS_INIT_0  0x67452301L
#define SHS_INIT_1  0xEFCDAB89L
#define SHS_INIT_2  0x98BADCFEL
#define SHS_INIT_3  0x10325476L
#define SHS_INIT_4  0xC3D2E1F0L

/* 32-bit rotate left - kludged with shifts */
#define ROTL(n,X)  (((X) << (n)) | ((X) >> (32-(n))))

/*
 * The initial EXPANDing function.  The hash function is defined over an
 * 80-UINT2 EXPANDed input array W, where the first 16 are copies of the input
 * data, and the remaining 64 are defined by
 *
 *      W[ i ] = W[ i - 16 ] ^ W[ i - 14 ] ^ W[ i - 8 ] ^ W[ i - 3 ]
 *
 * This implementation generates these values on the fly in a circular
 * buffer 
 */

#define EXPAND(W,i) \
    (W[i & 15] = ROTL(1,(W[ i       & 15] ^ \
                         W[(i - 14) & 15] ^ \
                         W[(i -  8) & 15] ^ \
                         W[(i -  3) & 15])))

/*
 * The prototype SHS sub-round.  The fundamental sub-round is:
 *
 *     a' = e + ROTL( 5, a ) + f( b, c, d ) + k + data;
 *     b' = a;
 *     c' = ROTL( 30, b );
 *     d' = c;
 *     e' = d;
 *
 * but this is implemented by unrolling the loop 5 times and renaming the
 * variables ( e, a, b, c, d ) = ( a', b', c', d', e' ) each iteration.
 * This code is then replicated 20 times for each of the 4 functions, using
 * the next 20 values from the W[] array each time 
 */

#define SUBROUND(a,b,c,d,e,f,k,data) \
    (e += ROTL(5,a) + f(b,c,d) + k + data, b = ROTL(30,b))

/**
 * Performs the SHS transformation. Note that this code seems to break some
 * optimizing compilers due to the complexity of the expressions and the size
 * of the basic block. It may be necessary to split it into sections, e.g. 
 * based on the four SUBROUNDs
 *
 * Note that this corrupts the sh->data area 
 */
static void SHA1_Transform(I32u * digest, I32u * data)
{
    I32u A, B, C, D, E;

    /* Expanded data */
    I32u eData[SHS_DATASIZE/4];

#if BYTE_ORDER == LITTLE_ENDIAN
    int i;
    for (i=0; i<SHS_DATASIZE/4; i++) {
        register I32u x = data[i];
        x = ((x >> 24) | 
            ((x & 0x00ff0000) >> 8) | 
            ((x & 0x0000ff00) << 8) | 
            ((x & 0x000000ff) << 24));
        eData[i] = data[i] = x;
    }
#elif BYTE_ORDER == BIG_ENDIAN
    memcpy(eData, data, sizeof(eData));
#else
#  error "Please fix BYTE_ORDER"
#endif /* BYTE_ORDER */

    /* Set up first buffer and local data buffer */
    A = digest[0];
    B = digest[1];
    C = digest[2];
    D = digest[3];
    E = digest[4];

    /* Heavy mangling, in 4 sub-rounds of 20 interations each. */
    SUBROUND( A, B, C, D, E, F1, K1, eData[  0 ] );
    SUBROUND( E, A, B, C, D, F1, K1, eData[  1 ] );
    SUBROUND( D, E, A, B, C, F1, K1, eData[  2 ] );
    SUBROUND( C, D, E, A, B, F1, K1, eData[  3 ] );
    SUBROUND( B, C, D, E, A, F1, K1, eData[  4 ] );
    SUBROUND( A, B, C, D, E, F1, K1, eData[  5 ] );
    SUBROUND( E, A, B, C, D, F1, K1, eData[  6 ] );
    SUBROUND( D, E, A, B, C, F1, K1, eData[  7 ] );
    SUBROUND( C, D, E, A, B, F1, K1, eData[  8 ] );
    SUBROUND( B, C, D, E, A, F1, K1, eData[  9 ] );
    SUBROUND( A, B, C, D, E, F1, K1, eData[ 10 ] );
    SUBROUND( E, A, B, C, D, F1, K1, eData[ 11 ] );
    SUBROUND( D, E, A, B, C, F1, K1, eData[ 12 ] );
    SUBROUND( C, D, E, A, B, F1, K1, eData[ 13 ] );
    SUBROUND( B, C, D, E, A, F1, K1, eData[ 14 ] );
    SUBROUND( A, B, C, D, E, F1, K1, eData[ 15 ] );
    SUBROUND( E, A, B, C, D, F1, K1, EXPAND( eData, 16 ) );
    SUBROUND( D, E, A, B, C, F1, K1, EXPAND( eData, 17 ) );
    SUBROUND( C, D, E, A, B, F1, K1, EXPAND( eData, 18 ) );
    SUBROUND( B, C, D, E, A, F1, K1, EXPAND( eData, 19 ) );

    SUBROUND( A, B, C, D, E, F2, K2, EXPAND( eData, 20 ) );
    SUBROUND( E, A, B, C, D, F2, K2, EXPAND( eData, 21 ) );
    SUBROUND( D, E, A, B, C, F2, K2, EXPAND( eData, 22 ) );
    SUBROUND( C, D, E, A, B, F2, K2, EXPAND( eData, 23 ) );
    SUBROUND( B, C, D, E, A, F2, K2, EXPAND( eData, 24 ) );
    SUBROUND( A, B, C, D, E, F2, K2, EXPAND( eData, 25 ) );
    SUBROUND( E, A, B, C, D, F2, K2, EXPAND( eData, 26 ) );
    SUBROUND( D, E, A, B, C, F2, K2, EXPAND( eData, 27 ) );
    SUBROUND( C, D, E, A, B, F2, K2, EXPAND( eData, 28 ) );
    SUBROUND( B, C, D, E, A, F2, K2, EXPAND( eData, 29 ) );
    SUBROUND( A, B, C, D, E, F2, K2, EXPAND( eData, 30 ) );
    SUBROUND( E, A, B, C, D, F2, K2, EXPAND( eData, 31 ) );
    SUBROUND( D, E, A, B, C, F2, K2, EXPAND( eData, 32 ) );
    SUBROUND( C, D, E, A, B, F2, K2, EXPAND( eData, 33 ) );
    SUBROUND( B, C, D, E, A, F2, K2, EXPAND( eData, 34 ) );
    SUBROUND( A, B, C, D, E, F2, K2, EXPAND( eData, 35 ) );
    SUBROUND( E, A, B, C, D, F2, K2, EXPAND( eData, 36 ) );
    SUBROUND( D, E, A, B, C, F2, K2, EXPAND( eData, 37 ) );
    SUBROUND( C, D, E, A, B, F2, K2, EXPAND( eData, 38 ) );
    SUBROUND( B, C, D, E, A, F2, K2, EXPAND( eData, 39 ) );

    SUBROUND( A, B, C, D, E, F3, K3, EXPAND( eData, 40 ) );
    SUBROUND( E, A, B, C, D, F3, K3, EXPAND( eData, 41 ) );
    SUBROUND( D, E, A, B, C, F3, K3, EXPAND( eData, 42 ) );
    SUBROUND( C, D, E, A, B, F3, K3, EXPAND( eData, 43 ) );
    SUBROUND( B, C, D, E, A, F3, K3, EXPAND( eData, 44 ) );
    SUBROUND( A, B, C, D, E, F3, K3, EXPAND( eData, 45 ) );
    SUBROUND( E, A, B, C, D, F3, K3, EXPAND( eData, 46 ) );
    SUBROUND( D, E, A, B, C, F3, K3, EXPAND( eData, 47 ) );
    SUBROUND( C, D, E, A, B, F3, K3, EXPAND( eData, 48 ) );
    SUBROUND( B, C, D, E, A, F3, K3, EXPAND( eData, 49 ) );
    SUBROUND( A, B, C, D, E, F3, K3, EXPAND( eData, 50 ) );
    SUBROUND( E, A, B, C, D, F3, K3, EXPAND( eData, 51 ) );
    SUBROUND( D, E, A, B, C, F3, K3, EXPAND( eData, 52 ) );
    SUBROUND( C, D, E, A, B, F3, K3, EXPAND( eData, 53 ) );
    SUBROUND( B, C, D, E, A, F3, K3, EXPAND( eData, 54 ) );
    SUBROUND( A, B, C, D, E, F3, K3, EXPAND( eData, 55 ) );
    SUBROUND( E, A, B, C, D, F3, K3, EXPAND( eData, 56 ) );
    SUBROUND( D, E, A, B, C, F3, K3, EXPAND( eData, 57 ) );
    SUBROUND( C, D, E, A, B, F3, K3, EXPAND( eData, 58 ) );
    SUBROUND( B, C, D, E, A, F3, K3, EXPAND( eData, 59 ) );

    SUBROUND( A, B, C, D, E, F4, K4, EXPAND( eData, 60 ) );
    SUBROUND( E, A, B, C, D, F4, K4, EXPAND( eData, 61 ) );
    SUBROUND( D, E, A, B, C, F4, K4, EXPAND( eData, 62 ) );
    SUBROUND( C, D, E, A, B, F4, K4, EXPAND( eData, 63 ) );
    SUBROUND( B, C, D, E, A, F4, K4, EXPAND( eData, 64 ) );
    SUBROUND( A, B, C, D, E, F4, K4, EXPAND( eData, 65 ) );
    SUBROUND( E, A, B, C, D, F4, K4, EXPAND( eData, 66 ) );
    SUBROUND( D, E, A, B, C, F4, K4, EXPAND( eData, 67 ) );
    SUBROUND( C, D, E, A, B, F4, K4, EXPAND( eData, 68 ) );
    SUBROUND( B, C, D, E, A, F4, K4, EXPAND( eData, 69 ) );
    SUBROUND( A, B, C, D, E, F4, K4, EXPAND( eData, 70 ) );
    SUBROUND( E, A, B, C, D, F4, K4, EXPAND( eData, 71 ) );
    SUBROUND( D, E, A, B, C, F4, K4, EXPAND( eData, 72 ) );
    SUBROUND( C, D, E, A, B, F4, K4, EXPAND( eData, 73 ) );
    SUBROUND( B, C, D, E, A, F4, K4, EXPAND( eData, 74 ) );
    SUBROUND( A, B, C, D, E, F4, K4, EXPAND( eData, 75 ) );
    SUBROUND( E, A, B, C, D, F4, K4, EXPAND( eData, 76 ) );
    SUBROUND( D, E, A, B, C, F4, K4, EXPAND( eData, 77 ) );
    SUBROUND( C, D, E, A, B, F4, K4, EXPAND( eData, 78 ) );
    SUBROUND( B, C, D, E, A, F4, K4, EXPAND( eData, 79 ) );

    /* Build message digest */
    digest[0] += A;
    digest[1] += B;
    digest[2] += C;
    digest[3] += D;
    digest[4] += E;
}

/**
 * Initializes the SHS values
 */
static void SHA1_Init(Digest* d)
{
    Sha1 * sh = CAST(d, Sha1, md);

    d->type = &Sha1Digest;
    d->flags = MD_INITIAL;

    /* Set the h-vars to their initial values */
    sh->digest[0] = SHS_INIT_0;
    sh->digest[1] = SHS_INIT_1;
    sh->digest[2] = SHS_INIT_2;
    sh->digest[3] = SHS_INIT_3;
    sh->digest[4] = SHS_INIT_4;

    /* Initialise the bit count */
    sh->count = 0;
}

/**
 * Updates SHS for a block of data. 
 */
static void SHA1_Update(Digest * d, const void * in, size_t count)
{
    Sha1 * sh = CAST(d, Sha1, md);
    const I8u * buffer = (I8u*)in;

    /* Get count of bytes already in data */
    size_t dataCount = (((I32u)sh->count) >> 3) & 0x3F;

    /* Update bitcount */
    sh->count += (((I64u)count) << 3);

    /* Handle any leading odd-sized chunks */
    if (dataCount) {
        I8u * p = ((I8u*)sh->data) + dataCount;
        dataCount = SHS_DATASIZE - dataCount;
        if (count < dataCount) {
            memcpy(p, buffer, count);
            return;
        }
        memcpy(p, buffer, dataCount);
        SHA1_Transform(sh->digest, sh->data);
        buffer += dataCount;
        count -= dataCount;
    }

    /* Process data in SHS_DATASIZE chunks */
    while (count >= SHS_DATASIZE) {
        memcpy(sh->data, buffer, SHS_DATASIZE);
        SHA1_Transform(sh->digest, sh->data);
        buffer += SHS_DATASIZE;
        count -= SHS_DATASIZE;
    }

    /* Handle any remaining bytes of data. */
    memcpy(sh->data, buffer, count);
}

/**
 * Completes the SH calculation
 */
static void SHA1_Finish(Digest * d, void * out)
{
    Sha1 * sh = CAST(d, Sha1, md);

#if BYTE_ORDER == LITTLE_ENDIAN
    I8u * output = (I8u*)out;
    int i, j;
#endif /* BYTE_ORDER == LITTLE_ENDIAN */

    /* Compute number of bytes mod 64 */
    size_t count = (((I32u)sh->count) >> 3) & 0x3F;

    /* Set the first char of padding to 0x80.  This is safe since there is
     * always at least one byte free */
    I8u *dataPtr = ((I8u*)sh->data) + count;
    *dataPtr++ = 0x80;

    /* Bytes of padding needed to make 64 bytes */
    count = SHS_DATASIZE - 1 - count;

    /* Pad out to 56 mod 64 */
    if (count < 8) {

        /* Pad the first block to 64 bytes */
        memset(dataPtr, 0, count);
        SHA1_Transform(sh->digest, sh->data);

        /* Now fill the next block with 56 bytes */
        memset(sh->data, 0, SHS_DATASIZE - 8);
    } else {
        /* Pad block to 56 bytes */
        memset(dataPtr, 0, count - 8);
    }

    /* Append length in bits and transform */
    sh->data[14] = ntohl((I32u)(sh->count >> 32));
    sh->data[15] = ntohl((I32u)sh->count);
    SHA1_Transform(sh->digest, sh->data);

    /* Output to an array of bytes */
#if BYTE_ORDER == LITTLE_ENDIAN
    for(i = 0, j = 0; j < SHA1_DIGEST_SIZE; i++, j += 4) {
        register I32u x = sh->digest[i];
        output[j  ] = (I8u)(x >> 24);
        output[j+1] = (I8u)(x >> 16);
        output[j+2] = (I8u)(x >> 8);
        output[j+3] = (I8u)x;
    }
#elif BYTE_ORDER == BIG_ENDIAN
    memcpy(out, sh->digest, sizeof(sh->digest));
#else
#  error "Please fix BYTE_ORDER"
#endif /* BYTE_ORDER */

    /* Wipe out sensitive stuff */
    memset(sh->digest, 0, sizeof(sh->digest));
    memset(sh->data, 0, sizeof(sh->data));
}

/**
 * Deallocates the SHA-1 object
 */
static void SHA1_Free(Digest * d)
{
    MEM_Free(CAST(d,Sha1,md));
}

/**
 * Allocates the SHA-1 digest object
 */
Digest * SHA1_Create()
{
    Sha1 * sh = MEM_New(Sha1);
    if (sh) SHA1_Init(&sh->md);
    return &sh->md;
}

/** 
 * Does init/update/finish in one shot. There must be at least 
 * SHA1_DIGEST_SIZE bytes available in the output buffer
 */
void SHA1_Digest(const void * in, size_t n, void * out)
{
    Sha1 sh;
    SHA1_Init(&sh.md);
    SHA1_Update(&sh.md, in, n);
    SHA1_Finish(&sh.md, out);
}

/*
 * HISTORY:
 *
 * $Log: s_sha1.c,v $
 * Revision 1.6  2009/11/07 09:12:54  slava
 * o replaced a few more int's with size_t
 *
 * Revision 1.5  2009/04/09 21:17:17  slava
 * o user size_t instead of int where it's appropriate
 *
 * Revision 1.4  2004/10/20 00:18:50  slava
 * o fixed Unicode compilation issues
 *
 * Revision 1.3  2004/07/31 07:25:50  slava
 * o use const pointers where appropriate
 *
 * Revision 1.2  2004/07/29 17:57:47  slava
 * o fixed SHA-1 calculation for big endian platform (e.g. Solaris)
 *
 * Revision 1.1  2004/07/29 17:09:37  slava
 * o message digest framework
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
