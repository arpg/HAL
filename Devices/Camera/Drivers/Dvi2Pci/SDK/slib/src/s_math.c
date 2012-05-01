/*
 * $Id: s_math.c,v 1.3 2004/11/28 18:59:21 slava Exp $
 *
 * Copyright (C) 2001-2004 by Slava Monich
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

#include "s_math.h"

/*==========================================================================*
 *              I N T E G E R    S Q U A R E    R O O T 
 *==========================================================================*/

/* 32-bit version */
I32u isqrt32(I32u val)
{
    register I32u bit = 1 << 15;
    register I32u x = 0;
    do {
        register I32u x2;
        x ^= bit;  
        x2 = x * x;
        if (x2 > val) {
            x ^= bit;
        } else if (x2 == val) {
            break;
        }
    } while (bit >>= 1);
    return x;
}
       
/* 64-bit version */
I64u isqrt64(I64u val)
{
    I64u bit = (__UINT64_C(1)) << 31;
    I64u x = 0;
    do {
        I64u x2;
        x ^= bit;  
        x2 = x * x;
        if (x2 > val) {
            x ^= bit;
        } else if (x2 == val) {
            break;
        }
    } while (bit >>= 1);
    return x;
}
       
/*==========================================================================*
 *              U T I L I T I E S
 *==========================================================================*/

/**
 * Returns number of bits in the specified 32-bit integer.
 */
int MATH_BitLen(I32s i)
{
    /* binary search unrolled for efficiency */
    return  (i<(1<<15) ?
                (i<(1<<7) ?
                    (i<(1<<3) ?
                        (i<(1<<1) ? 
                            (i<(1<<0) ? ((i<0) ? 32 : 0) : 1) : 
                            (i<(1<<2) ? 2 : 3)) :
                        (i<(1<<5)  ? 
                            (i<(1<<4) ? 4  : 5)  : 
                            (i<(1<<6) ? 6  : 7))) :
                    (i<(1<<11) ?
                        (i<(1<<9)  ? 
                            (i<(1<<8)  ? 8  : 9)  : 
                            (i<(1<<10) ? 10 : 11)) :
                        (i<(1<<13) ? 
                            (i<(1<<12) ? 12 : 13) : 
                            (i<(1<<14) ? 14 : 15)))) :
                (i<(1<<23) ?
                    (i<(1<<19) ?
                        (i<(1<<17) ? 
                            (i<(1<<16) ? 16 : 17) : 
                            (i<(1<<18) ? 18 : 19)) :
                        (i<(1<<21) ? 
                            (i<(1<<20) ? 20 : 21) : 
                            (i<(1<<22) ? 22 : 23))) :
                    (i<(1<<27) ?
                        (i<(1<<25) ? 
                            (i<(1<<24) ? 24 : 25) : 
                            (i<(1<<26) ? 26 : 27)) :
                        (i<(1<<29) ? 
                            (i<(1<<28) ? 28 : 29) : 
                            (i<(1<<30) ? 30 : 31)))));
}

/**
 * Counts number of bits set in an integer
 * 
 * Algorithm: subtracting 1 from any number causes all bits
 * up to and including the least significant non-zero bit to
 * be complemented.
 * 
 * This algorithm is very quick when there are only a few bits
 * turned on.
 */
int MATH_BitCnt(int i)
{
    int n = 0;
    while (i) {
        i &= i-1;  
        n++;
    }
    return n;
}

/*
 * HISTORY:
 *
 * $Log: s_math.c,v $
 * Revision 1.3  2004/11/28 18:59:21  slava
 * o reformatted MATH_BitLen
 *
 * Revision 1.2  2003/07/24 15:03:02  slava
 * o added MATH_BitLen and MATH_BitCnt functions
 *
 * Revision 1.1  2001/11/26 07:59:55  slava
 * o added s_math module
 *
 * Local Variables:
 * c-basic-offset:4
 * indent-tabs-mode: nil
 * End:
 */
