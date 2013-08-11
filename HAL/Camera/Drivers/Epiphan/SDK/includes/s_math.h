/*
 * $Id: s_math.h,v 1.12 2004/08/27 19:13:13 slava Exp $
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

#ifndef _SLAVA_MATH_H_
#define _SLAVA_MATH_H_

#include "s_strbuf.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* integer square root */
extern I32u isqrt32 P_((I32u val));
extern I64u isqrt64 P_((I64u val));

/* extern unsigned int isqrt(unsigned int x); */
#define isqrt(_x) isqrt32(_x)

/* extern unsigned long lsqrt(unsigned long x); */
#ifdef __LONG_64__
#define lsqrt(_x) isqrt64(_x)
#else /* !__LONG_64__ */
#define lsqrt(_x) isqrt32(_x)
#endif /* !__LONG_64__ */

/* check if the number is a power of 2 */
#define ispow2(_n) (((_n) & -(_n)) == (_n))

/* utilities */
extern int MATH_BitLen P_((I32s i));
extern int MATH_BitCnt P_((int i));

/* multiple precision math */
typedef I32u BIElem;    /* storage element, must be unsigned */

typedef struct _BigInt {
    BIElem * value;     /* the value of this BigInt */
    int len;            /* number of ints in use in the above array */
    int alloc;          /* the allocated length of the above array, in ints */
    int signum;         /* -1 for negative, 0 for zero, or 1 for positive*/
    int bitlen;         /* bit length (cached) */
    int lowbit;         /* lowest bit set (cached) */
} BigInt;

extern BigInt * BIGINT_Create P_((void));
extern BigInt * BIGINT_Create32 P_((I32s n));
extern BigInt * BIGINT_Create64 P_((I64s n));
extern BigInt * BIGINT_Clone P_((const BigInt * b));
extern void BIGINT_Delete P_((BigInt * b));
extern void BIGINT_Init P_((BigInt * b));
extern void BIGINT_Destroy P_((BigInt * b));
extern void BIGINT_Zero P_((BigInt * b));
extern void BIGINT_Neg P_((BigInt * b));
extern int  BIGINT_BitLen P_((BigInt * b));
extern int  BIGINT_LowBit P_((BigInt * b));
extern Bool BIGINT_IsBitSet P_((const BigInt * b, int pos));
extern int  BIGINT_Compare P_((const BigInt * b1, const BigInt * b2));
extern int  BIGINT_AbsCompare P_((const BigInt * b1, const BigInt * b2));
extern Str  BIGINT_Format P_((const BigInt * b, StrBuf * sb, int radix));
extern Bool BIGINT_Parse P_((BigInt * b, Str s, int radix));
extern Bool BIGINT_Equal P_((const BigInt * b1, const BigInt * b2));
extern Bool BIGINT_SetValue32 P_((BigInt * b, I32s n));
extern Bool BIGINT_SetValue64 P_((BigInt * b, I64s n));
extern Bool BIGINT_IsNeg P_((const BigInt * b));
extern Bool BIGINT_IsPos P_((const BigInt * b));
extern Bool BIGINT_IsOne P_((const BigInt * b));
extern Bool BIGINT_IsZero P_((const BigInt * b));
extern Bool BIGINT_IsOdd P_((const BigInt * b));
extern Bool BIGINT_IsEven P_((const BigInt * b));
extern Bool BIGINT_ShiftLeft P_((BigInt * b, int n));
extern Bool BIGINT_ShiftRight P_((BigInt * b, int n));
extern Bool BIGINT_Copy P_((BigInt * dest, const BigInt * src));
extern Bool BIGINT_Add P_((BigInt * b1, const BigInt * b2));
extern Bool BIGINT_Sub P_((BigInt * b1, const BigInt * b2));
extern Bool BIGINT_Mul P_((BigInt * b1, const BigInt * b2));
extern Bool BIGINT_MulInt P_((BigInt * b, int val));
extern Bool BIGINT_Square P_((BigInt * b));
extern Bool BIGINT_Div P_((BigInt * b1, const BigInt * b2, BigInt * rem));
extern Bool BIGINT_DivInt P_((BigInt * b, int n, int * rem));
extern Bool BIGINT_Exp P_((BigInt * b, BigInt * p));
extern Bool BIGINT_ExpMod P_((BigInt * b, BigInt * p, const BigInt * m));
extern Bool BIGINT_ModInverse P_((BigInt * b, const BigInt * m));
extern Bool BIGINT_Gcd P_((BigInt * gcd, const BigInt* b1, const BigInt* b2));

#define BIGINT_Clear(_b,_i)   BIGINT_Zero(_b,_i)
#define BIGINT_SetInt(_b,_i)  BIGINT_SetValue32(_b,_i)
#define BIGINT_CreateInt(_i)  BIGINT_Create32(_i)

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_MATH_H_ */

/*
 * HISTORY:
 *
 * $Log: s_math.h,v $
 * Revision 1.12  2004/08/27 19:13:13  slava
 * o fixed bugs in BIGINT_Exp and BIGINT_ExpMod (sign could be wrong)
 * o added BIGINT_AbsCompare function
 *
 * Revision 1.11  2004/08/27 04:16:44  slava
 * o fixed gcc compilation warnings
 *
 * Revision 1.10  2004/08/27 04:09:15  slava
 * o added BIGINT_ExpMod function which calculates (b^p mod m) far more
 *   efficiently than if you first do (b^p) and then (mod m), even though
 *   it is using the most primitive algorithm. As always, there's a lot of
 *   room for improvement. Anyway, the bottom line is that now this module
 *   contains all the necessary ingredients for implementing many popular
 *   cryptographic algorithms, such as DSA signature verification. QED.
 *
 * Revision 1.9  2004/08/27 02:16:33  slava
 * o added BIGINT_Zero, BIGINT_IsBitSet, BIGINT_Exp, BIGINT_Square and
 *   BIGINT_ModInverse functions
 *
 * Revision 1.8  2004/08/26 16:29:00  slava
 * o added BIGINT_Parse, BIGINT_IsOdd and BIGINT_IsEven functions
 * o fixed a bug in BIGINT_Equal, introduced in rev 1.5
 *
 * Revision 1.7  2003/07/31 15:15:46  slava
 * o multiple precision arithmetic
 *
 * Revision 1.6  2003/07/25 03:00:34  slava
 * o added extern keyword to the prototypes
 *
 * Revision 1.5  2003/07/24 15:03:02  slava
 * o added MATH_BitLen and MATH_BitCnt functions
 *
 * Revision 1.4  2003/07/23 03:31:59  slava
 * o added ispow2 macro
 *
 * Revision 1.3  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.2  2002/03/04 01:35:55  slava
 * o fixed comment
 *
 * Revision 1.1  2001/11/26 07:59:55  slava
 * o added s_math module
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
