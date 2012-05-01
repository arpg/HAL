/*
 * $Id: s_mpm.c,v 1.17 2010/09/25 09:31:32 slava Exp $
 *
 * Copyright (C) 2002-2010 by Slava Monich
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

#include "s_mem.h"
#include "s_math.h"

/*==========================================================================*
 *              M U L T I P L E    P R E C I S I O N    M A T H
 *==========================================================================*/

typedef I64u BIElem2;   /* twice as large as BIElem */
typedef I64s BIElem2s;  /* same as BIElem2 but signed */

#define ELEM_BITS 32
#define ELEM_MASK 0xffffffff

/* All possible chars for representing a number as a String */
static Char DIGITS [] = {
    '0' , '1' , '2' , '3' , '4' , '5' ,
    '6' , '7' , '8' , '9' , 'a' , 'b' ,
    'c' , 'd' , 'e' , 'f' , 'g' , 'h' ,
    'i' , 'j' , 'k' , 'l' , 'm' , 'n' ,
    'o' , 'p' , 'q' , 'r' , 's' , 't' ,
    'u' , 'v' , 'w' , 'x' , 'y' , 'z'
};

/* number of trailing zero bits in one byte */
static I8s TRAILING_ZEROS [] = {
  -25, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    7, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0
};

#define MIN_RADIX 2
#define MAX_RADIX COUNT(DIGITS)
#define DEFAULT_RADIX 10

/**
 * Invalidates redundant fields
 */
STATIC void BIGINT_Invalidate(BigInt * b)
{
    b->bitlen = -1;
    b->lowbit = -1;
}

/**
 * Initializes BigInt to the value of zero
 */
void BIGINT_Init(BigInt * b)
{
    memset(b, 0, sizeof(*b));
    BIGINT_Invalidate(b);
}


/**
 * Deallocates all the memory that has been allocated for this BigInt
 */
void BIGINT_Destroy(BigInt * b)
{
    BIGINT_Invalidate(b);
    MEM_Free(b->value);
    b->signum = 0;
    b->value = NULL;
    b->alloc = 0;
    b->len = 0;
}

/**
 * Creates a BigInt that has a value of zero.
 */
BigInt * BIGINT_Create()
{
    BigInt * b = MEM_New(BigInt);
    if (b) BIGINT_Init(b);
    return b;
}

/**
 * Deletes a BigInt that was created by one of the BIGINT_Create
 * functions. Ignores NULL argument.
 */
void BIGINT_Delete(BigInt * b)
{
    if (b) {
        BIGINT_Destroy(b);
        MEM_Free(b);
    }
}

/**
 * If this BigInt cannot hold min words, increase the size
 * of the value array to len words.
 */
STATIC Bool BIGINT_Alloc(BigInt * b, int min)
{
    int len = b->alloc;
    if (min > len) {
        BIElem * newBuf;
        int newLen = (len + 7) & (-8);
        if (newLen < min) newLen = (min + 7) & (-8);
#ifdef NO_REALLOC
        newBuf = MEM_NewArray(BIElem, newLen);
        if (!newBuf) return False;
        if (b->len) memcpy(newBuf, b->value, b->len * sizeof(BIElem));
        MEM_Free(b->value);
#else
        newBuf = (BIElem*)MEM_Realloc(b->value, newLen * sizeof(BIElem));
        if (!newBuf) return False;
#endif /* NO_REALLOC */
        b->value = newBuf;
        b->alloc = newLen;
    }
    return True;
}

/**
 * Creates a BigInt that the specified signed 32 bit value
 */
BigInt * BIGINT_Create32(I32s n)
{
    BigInt * b = BIGINT_Create();
    if (b) {
        if (n == 0) {
            return b;
        } else if (BIGINT_Alloc(b, 1)) {
            if (n > 0) {
                b->signum = 1;
            } else {
                b->signum = -1;
                n = -n;
            }
            b->value[b->len++] = n;
            return b;
        } else {
            BIGINT_Delete(b);
        }
    }
    return NULL;
}

/**
 * Creates a BigInt that the specified signed 64 bit value
 */
BigInt * BIGINT_Create64(I64s n)
{
    BigInt * b = BIGINT_Create();
    if (b) {
        if (n == 0) {
            return b;
        } else if (BIGINT_Alloc(b, 2)) {
            if (n > 0) {
                b->signum = 1;
            } else {
                b->signum = -1;
                n = -n;
            }
            b->value[b->len++] = (BIElem)n;
            if ((n >> ELEM_BITS) != 0) {
                b->value[b->len++] = (BIElem)(n >> ELEM_BITS);
            }
            return b;
        }
        BIGINT_Delete(b);
    }
    return NULL;
}

/**
 * Assignes signed 32-bit value to this BigInt
 */
Bool BIGINT_SetValue32(BigInt * b, I32s n)
{
    if (n == 0) {
        BIGINT_Zero(b);
        return True;
    } else if (BIGINT_Alloc(b, 1)) {
        if (n > 0) {
            b->signum = 1;
        } else {
            b->signum = -1;
            n = -n;
        }
        b->value[0] = n;
        b->len = 1;
        BIGINT_Invalidate(b);
        return True;
    } else {
        return False;
    }
}

/**
 * Assignes signed 64-bit value to this BigInt
 */
Bool BIGINT_SetValue64(BigInt * b, I64s n)
{
    if (n == 0) {
        BIGINT_Zero(b);
        return True;
    } else if (BIGINT_Alloc(b, 2)) {
        if (n > 0) {
            b->signum = 1;
        } else {
            b->signum = -1;
            n = -n;
        }
        b->value[0] = (BIElem)n;
        b->len = 1;
        if ((n >> ELEM_BITS) != 0) {
            b->value[b->len++] = (BIElem)(n >> ELEM_BITS);
        }
        BIGINT_Invalidate(b);
        return True;
    } else {
        return False;
    }
}

/**
 * Copies one BigInt into another
 */
Bool BIGINT_Copy(BigInt * dest, const BigInt * src)
{
    if (BIGINT_Alloc(dest, src->len)) {
        memcpy(dest->value, src->value, src->len * sizeof(BIElem));
        dest->len = src->len;
        dest->signum = src->signum;
        dest->bitlen = src->bitlen;
        dest->lowbit = src->lowbit;
        return True;
    }
    return False;
}

/**
 * Clones the specified BigInt
 */
BigInt * BIGINT_Clone(const BigInt * b)
{
    BigInt * dest = BIGINT_Create();
    if (dest) {
        if (BIGINT_Copy(dest, b)) {
            return dest;
        }
        BIGINT_Delete(dest);
    }
    return NULL;
}

/**
 * Assigns the value of zero to the specified BigInt
 */
void BIGINT_Zero(BigInt * b)
{
    if (b->signum != 0) {
        b->signum = 0;
        b->len = 0;
        BIGINT_Invalidate(b);
    }
}

/**
 * Returns the number of bits in the minimal two's-complement
 * representation of this BigInt, excluding a sign bit. For 
 * positive BigInts, this is equivalent to the number of bits 
 * in the ordinary binary representation. 
 */
int BIGINT_BitLen(BigInt * b)
{
    if (b->bitlen < 0) {
        if (b->signum == 0) {
            b->bitlen = 0;
        } else {
            /* Calculate the bit length of the magnitude */
            int bitsInHighWord = MATH_BitLen(b->value[b->len-1]);
            int n = (b->len-1)*ELEM_BITS + bitsInHighWord;
            if (b->signum < 0) {
                int i;
                Bool pow2 = BoolValue(bitsInHighWord == 1);
                for(i=b->len-2; i>=0 && pow2; i--) {
                    pow2 = BoolValue(MATH_BitLen(b->value[i]) == 0);
                }
                b->bitlen = (pow2 ? (n-1) : n);
            } else {
                b->bitlen = n;
            }
        }
    }
    return b->bitlen;
}

/**
 * Returns the index of the lowest set bit in this BigInt, or -1 if it's zero
 */
int BIGINT_LowBit(BigInt * b)
{
    if (b->lowbit < 0 && b->signum != 0) {
        int i, x, n = 0;
        for (i=0; i<b->len && b->value[i] == 0; i++) n++;
        ASSERT(i<b->len);
        x = b->value[i];
        n *= ELEM_BITS;
        if (x & 0xff) {
            n += TRAILING_ZEROS[x & 0xff];
        } else if (x & 0xff00) {
            n += TRAILING_ZEROS[(x >> 8) & 0xff];
        } else if (x & 0xff0000) {
            n += TRAILING_ZEROS[(x >> 16) & 0xff];
        } else {
            ASSERT(x & 0xff000000);
            n += TRAILING_ZEROS[(x >> 24) & 0xff];
        }
        b->lowbit = n;
    }
    return b->lowbit;
}

/**
 * Tests the specified bit in the BigInt
 */
Bool BIGINT_IsBitSet(const BigInt * b, register int pos)
{
    if (b->signum != 0) {
        register int elemIndex = pos/ELEM_BITS;
        if (elemIndex < b->len) {
            BIElem mask = (((BIElem)1) << (pos%ELEM_BITS));
            if (b->value[elemIndex] & mask) {
                return True;
            }
        }
    }
    return False;
}

/**
 * Changes the sign of the BigInt
 */
void BIGINT_Neg(BigInt * b)
{
    b->signum *= -1;
}

/**
 * Tests whether the specified BigInt is negative
 */
Bool BIGINT_IsNeg(const BigInt * b)
{
    return BoolValue(b->signum < 0);
}

/**
 * Tests whether the specified BigInt is positive
 */
Bool BIGINT_IsPos(const BigInt * b)
{
    return BoolValue(b->signum > 0);
}

/**
 * Tests whether the specified BigInt is +1 or -1
 */
Bool BIGINT_IsOne(const BigInt * b)
{
    return BoolValue((b->len == 1) && b->value[0] == 1);
}

/**
 * Tests whether the specified BigInt is zero
 */
Bool BIGINT_IsZero(const BigInt * b)
{
    return BoolValue(b->signum == 0);
}

/**
 * Tests whether the specified BigInt is odd
 */
Bool BIGINT_IsOdd(const BigInt * b)
{
    return BoolValue((b->len > 0) && (b->value[0] & 1) == 1);
}

/**
 * Tests whether the specified BigInt is even
 */
Bool BIGINT_IsEven(const BigInt * b)
{
    return BoolValue((b->len == 0) || (b->value[0] & 1) == 0);
}

/**
 * Ensures that the BigInt is in normal form, specifically
 * making sure that there are no leading zeros, and that if the
 * magnitude is zero, then signum is zero.
 */
STATIC void BIGINT_Normalize(BigInt * b)
{
    while (b->len > 0 && !b->value[b->len - 1]) b->len--;
    if (b->len == 0) {
        b->signum = 0;
    }
}

/**
 * Compares two magnitude values of the same length. Returns -1, 0 or 1 
 * as v1 is less than, equal to, or greater than v2. 
 */
STATIC int BIGINT_CompareValues(const BIElem * v1, const BIElem * v2, int n)
{
    int i;
    for (i=n-1; i>=0; i--) {
        if (v1[i] > v2[i]) {
            return 1;
        } else if (v1[i] < v2[i]) {
            return (-1);
        }
    }
    return 0;
}

/**
 * Compares two BigInts. Returns -1, 0 or 1 if b1 is numerically 
 * less than, equal to, or greater than b2. 
 */
int BIGINT_Compare(const BigInt * b1, const BigInt * b2)
{
    if (b1->signum == b2->signum) {
        int n1 = b1->len;
        int n2 = b2->len;
        if (n1 > n2) {
            return b1->signum;
        } else if (n1 < n2) {
            return (-b1->signum);
        } else if (n1 > 0) {
            int cmp = BIGINT_CompareValues(b1->value, b2->value, n1);
            return (cmp * b1->signum);
        } else {
            return 0;
        }
    } else {
        return (b1->signum > b2->signum) ? 1 : (-1);
    }
}

/**
 * Compares the absolute values of two BigInts. Returns -1, 0 or 1 if 
 * absolute value of b1 is numerically less than, equal to, or greater
 * than the absolute value of b2. 
 */
int BIGINT_AbsCompare(const BigInt * b1, const BigInt * b2)
{
    int n1 = b1->len;
    int n2 = b2->len;
    if (n1 > n2) {
        return 1;
    } else if (n1 < n2) {
        return -1;
    } else if (n1 > 0) {
        return BIGINT_CompareValues(b1->value, b2->value, n1);
    } else {
        return 0;
    }
}

/**
 * Tests whether two BigInts are equal to each other. This function should
 * more efficient than BIGINT_Compare.
 */
Bool BIGINT_Equal(const BigInt * b1, const BigInt * b2)
{
    return BoolValue(b1->signum == b2->signum && b1->len == b2->len &&
        MEM_Equal(b1->value, b2->value, b1->len * sizeof(BIElem)));
}

/**
 * Left shifts this BigInt n bits, where n is less than 32.
 */
STATIC void BIGINT_QuickLeftShift(BigInt * b, int n)
{
    if (n > 0) {
        int i;
        int n2 = ELEM_BITS - n;
        for (i=b->len-1; i>0; i--) {
            BIElem e1 = b->value[i];
            BIElem e2 = b->value[i-1];
            b->value[i] = (e1 << n) | (e2 >> n2);
        }
        b->value[0] <<= n;
    }
}

/**
 * Right shifts this BigInt n bits, where n is less than 32.
 */
STATIC void BIGINT_QuickRightShift(BigInt * b, int n)
{
    if (n > 0) {
        int i;
        int n2 = ELEM_BITS - n;
        for (i=0; i<(b->len-1); i++) {
            BIElem e1 = b->value[i];
            BIElem e2 = b->value[i+1];
            b->value[i] = (e1 >> n) | (e2 << n2);
        }
        b->value[i] >>= n;
    }
}

/**
 * Shifts this BigInt left n bits. n must be positive.
 */
STATIC Bool BIGINT_ShiftLeft2(BigInt * b, int n)
{
    ASSERT(n > 0);
    if (b->signum != 0) {
        int bitsInHighWord = MATH_BitLen(b->value[b->len-1]);
        if (n <= (ELEM_BITS - bitsInHighWord)) {
            BIGINT_QuickLeftShift(b, n);
        } else {
            int nInts = n / ELEM_BITS;
            int nBits = n % ELEM_BITS;
            int k = nInts + ((nBits > (ELEM_BITS-bitsInHighWord)) ? 1 : 0);
            if ((b->len + k) > b->alloc) {
                /* grow the array */
                int newAlloc = ((b->len + k) + 8) & (-8);
                BIElem * newBuf = MEM_NewArray(BIElem, newAlloc);
                if (newBuf) {
                    newBuf[b->len + k - 1] = 0;
                    memcpy(newBuf + k, b->value, b->len*sizeof(BIElem));
                    memset(newBuf, 0, k*sizeof(BIElem));
                    MEM_Free(b->value);
                    b->alloc = newAlloc;
                    b->value = newBuf;
                } else {
                    return False;
                }
            } else {
                /* still have some room */
                b->value[b->len + k -1] = 0;
                memmove(b->value + k, b->value, b->len*sizeof(BIElem));
                memset(b->value, 0, k*sizeof(BIElem));
            }
            b->len += k;
            if (nBits > (ELEM_BITS-bitsInHighWord)) {
                BIGINT_QuickRightShift(b, ELEM_BITS-nBits);
            } else {
                BIGINT_QuickLeftShift(b, nBits);
            }
        }
        BIGINT_Invalidate(b);
    }
    return True;
}

/**
 * Shifts this BigInt right n bits. n must be positive.
 */
STATIC void BIGINT_ShiftRight2(BigInt * b, int n)
{
    if (b->signum != 0) {
        if (BIGINT_BitLen(b) <= n) {
            BIGINT_Zero(b);
        } else {
            int bitsInHighWord = MATH_BitLen(b->value[b->len-1]);
            if (n <= bitsInHighWord) {
                BIGINT_QuickRightShift(b, n);
            } else {
                int nInts = n / ELEM_BITS;
                int nBits = n % ELEM_BITS;
                if (nBits >= bitsInHighWord) {
                    int i;
                    int k = ELEM_BITS - nBits;
                    for (i=b->len-1; i>0; i--) {
                        BIElem e1 = b->value[i];
                        BIElem e2 = b->value[i-1];
                        b->value[i] = (e1 << k) | (e2 >> nBits);
                    }
                    b->value[0] <<= k;
                    nInts++;
                    b->len -= nInts;
                    memmove(b->value, b->value+nInts, b->len*sizeof(BIElem));
                } else {
                    b->len -= nInts;
                    memmove(b->value, b->value+nInts, b->len*sizeof(BIElem));
                    BIGINT_QuickRightShift(b, nBits);
                }
            }     
            BIGINT_Invalidate(b);
        }
    }
}

/**
 * If n is positive, shifts this BigInt left n bits. 
 * If n is negative, shifts this BigInt right -n bits. 
 * If n is zero, does nothing
 */
Bool BIGINT_ShiftLeft(BigInt * b, int n)
{
    if (n > 0) {
        return BIGINT_ShiftLeft2(b, n);
    }
    if (n < 0) {
        BIGINT_ShiftRight2(b, -n);
    }
    return True;
}

/**
 * If n is positive, shifts this BigInt right n bits. 
 * If n is negative, shifts this BigInt left -n bits. 
 * If n is zero, does nothing
 */
Bool BIGINT_ShiftRight(BigInt * b, int n)
{
    if (n < 0) {
        return BIGINT_ShiftLeft2(b, -n);
    }
    if (n > 0) {
        BIGINT_ShiftRight2(b, n);
    }
    return True;
}

/**
 * Adds the specified value to b, placing the result to b, 
 * keeping the signum
 */
STATIC Bool BIGINT_Add2(BigInt * b, const BIElem * value, int len)
{
    int x = b->len;
    int y = len;
    int resultLen = MAX(x,y);
    int resultAlloc = b->alloc;
    const BIElem * src1 = b->value;
    const BIElem * src2 = value;
    Bool resultAllocated = False;
    BIElem * result;
    BIElem * dest;
    BIElem2 sum = 0;

    if (resultLen > resultAlloc) {
        resultAlloc = (resultLen + 7) & (-8);
        result = MEM_NewArray(BIElem, resultAlloc);
        if (result) {
            resultAllocated = True;
        } else {
            return False;
        }
    } else {
        result = b->value;
    }

    /* add common parts of both numbers */
    dest = result;
    while (x>0 && y>0) {
        x--; y--;
        sum = ((BIElem2)(*src1++)) + (*src2++) + (sum >> ELEM_BITS);
        *dest++ = (BIElem)sum;
    }

    /* Add remainder of the longer number */
    while (x>0) {
        x--;
        sum = ((BIElem2)(*src1++)) + (sum >> ELEM_BITS);
        *dest++ = (BIElem)sum;
    }
    while(y>0) {
        y--;
        sum = ((BIElem2)(*src2++)) + (sum >> ELEM_BITS);
        *dest++ = (BIElem)sum;
    }

    if ((sum >> ELEM_BITS) > 0) { 
        /* Result must grow in length */
        if (resultLen == resultAlloc) {
            BIElem * temp;
            resultAlloc = (resultLen + 8) & (-8);
            temp = MEM_NewArray(BIElem, resultAlloc);
            if (!temp) {
                if (resultAllocated) {
                    MEM_Free(result);
                }
                return False;
            }
            memcpy(temp, result, resultLen * sizeof(BIElem));
            temp[resultLen] = 1;
            if (resultAllocated) MEM_Free(result);
            resultAllocated = True;
            result = temp;
        } else {
            *dest++ = 1;
        }
        resultLen++;
    }

    if (resultAllocated) {
        /* switch to the new buffer */
        MEM_Free(b->value);
        b->value = result;
        b->alloc = resultAlloc;
    }

    b->len = resultLen;
    BIGINT_Invalidate(b);
    return True;
}

/**
 * Subtracts the smaller value (v2, n2 elements) from the larger 
 * (v1, n1 elements) placing the result to BigInt, keeping the signum.
 * The caller MUST make sure that v1 is larger than v2.
 */
STATIC Bool BIGINT_Sub2(BigInt * b,
                        const BIElem * v1, int len1,
                        const BIElem * v2, int len2)
{
    Bool resultAllocated = False;
    int resultAlloc = b->alloc;
    int resultLen = len1;
    BIElem2s diff = (BIElem2s)0;
    BIElem * result;
    BIElem * dest;
    const BIElem * src1 = v1;
    const BIElem * src2 = v2;

    ASSERT(len1 >= len2);
    if (resultAlloc < resultLen) {
        resultAlloc = (resultLen + 7) & (-8);
        result = MEM_NewArray(BIElem, resultAlloc);
        if (result) {
            resultAllocated = True;
        } else {
            return False;
        }
    } else {
        result = b->value;
    }

    /* subtract common parts of both numbers */
    dest = result;
    len1 -= len2;
    while ((len2--)>0) {
        diff = (((BIElem2s)(*src1++)) & ELEM_MASK) - 
            (((BIElem2s)(*src2++)) & ELEM_MASK) - 
            ((BIElem)(-(diff>>ELEM_BITS)));
        *dest++ = (BIElem)diff;
    }

    /* subtract remainder of longer number (v1) */
    while ((len1--)>0 && (diff >>= ELEM_BITS) != 0) {
        diff = (((BIElem2s)(*src1++))&ELEM_MASK) - ((BIElem)(-diff));
        *dest++ = (BIElem)diff;
    }

    if (resultAllocated) {
        /* switch to the new buffer */
        MEM_Free(b->value);
        b->value = result;
        b->alloc = resultAlloc;
    }

    b->len = resultLen;
    BIGINT_Invalidate(b);
    BIGINT_Normalize(b);
    return True;
}


/**
 * Adds b2 to b1, placing the result to b1
 */
Bool BIGINT_Add(BigInt * b1, const BigInt * b2)
{
    if (BIGINT_IsZero(b2)) {
        return True;
    } else if (BIGINT_IsZero(b1)) {
        return BIGINT_Copy(b1, b2);
    } else {
        if (b1->signum == b2->signum) {
            return BIGINT_Add2(b1, b2->value, b2->len);
        } else {
            int cmp;
            if (b1->len > b2->len) {
                cmp = 1;
            } else if (b1->len < b2->len) {
                cmp = -1;
            } else {
                cmp = BIGINT_CompareValues(b1->value, b2->value, b1->len);
                if (cmp == 0) {
                    /* opposite values, the result is zero */
                    b1->len = 0;
                    b1->signum = 0;
                    BIGINT_Invalidate(b1);
                    return True;
                }
            }
            if (cmp < 0) {
                /* the sign will change */
                if (BIGINT_Sub2(b1, b2->value,b2->len, b1->value,b1->len)) {
                    b1->signum = (-b1->signum);
                } else {
                    return False;
                }
            } else {
                return BIGINT_Sub2(b1, b1->value,b1->len, b2->value,b2->len);
            }
            return True;
        }
    }
}

/**
 * Subtracts b2 from b1, placing the result to b1
 */
Bool BIGINT_Sub(BigInt * b1, const BigInt * b2)
{
    if (BIGINT_IsZero(b2)) {
        return True;
    } else if (BIGINT_IsZero(b1)) {
        if (BIGINT_Copy(b1, b2)) {
            b1->signum = (-b2->signum);
            return True;
        } else {
            return False;
        }
    } else {
        if (b1->signum != b2->signum) {
            return BIGINT_Add2(b1, b2->value, b2->len);
        } else {
            int cmp;
            if (b1->len > b2->len) {
                cmp = 1;
            } else if (b1->len < b2->len) {
                cmp = -1;
            } else {
                cmp = BIGINT_CompareValues(b1->value, b2->value, b1->len);
                if (cmp == 0) {
                    /* equal values, the result is zero */
                    b1->len = 0;
                    b1->signum = 0;
                    BIGINT_Invalidate(b1);
                    return True;
                }
            }
            if (cmp < 0) {
                /* the sign will change */
                if (BIGINT_Sub2(b1, b2->value,b2->len, b1->value,b1->len)) {
                    b1->signum = (-b1->signum);
                } else {
                    return False;
                }
            } else {
                return BIGINT_Sub2(b1, b1->value,b1->len, b2->value,b2->len);
            }
            return True;
        }
    }
}

/**
 * Multiplies BigInt by the integer
 */
Bool BIGINT_MulInt(BigInt * b, int val)
{
    if (val == 0) {
        BIGINT_Zero(b);
    } else if (val == -1) {
        b->signum = (-b->signum);
        BIGINT_Invalidate(b);
    } else if (val != 1) {
        int i;
        BIElem2 val2, carry;
        if (val < 0) {
            b->signum = (-b->signum);
            val = -val;
        }
        val2 = val;
        carry = 0;
        for (i=0; i<b->len; i++) {
            BIElem2 product = ((BIElem2)val2) * b->value[i] + carry;
            b->value[i] = (BIElem)product;
            carry = product >> ELEM_BITS;
        }
        if (carry != 0) {
            if (BIGINT_Alloc(b, b->len+1)) {
                b->value[b->len++] = (BIElem)carry;
            } else {
                return False;
            }
        }
        BIGINT_Invalidate(b);
    }
    return True;
}

/**
 * Multiplies two BigInts.
 */
Bool BIGINT_Mul(BigInt * b1, const BigInt * b2)
{
    int i1, i2;
    int len1 = b1->len;
    int len2 = b2->len;
    int newLen = len1 + len2;
    BIElem2 product;
    BIElem2 carry = (BIElem2)0;
    BIElem * src = b1->value;    
    ASSERT(b1 != b2);
    ASSERT(b1->value != b2->value || !b1->value);

    /* trivial cases */
    if (BIGINT_IsZero(b1)) {
        return True;
    } else if (BIGINT_IsZero(b2)) {
        BIGINT_Zero(b1);
        return True;
    } else if (BIGINT_IsOne(b2)) {
        if (b2->signum < 0) {
            b1->signum *= b2->signum;
            BIGINT_Invalidate(b1);
        }
        return True;
    } else if (BIGINT_IsOne(b1)) {
        int signum = b1->signum * b2->signum;
        if (BIGINT_Copy(b1, b2)) {
            b1->signum = signum;
            return True;
        } else {
            return False;
        }
    }

    /* allocate storage for the product */
    if (b1->alloc < newLen) {
        /* 
         * allocate enough storage for the product, use the old buffer
         * as a temporary storage. Note that we avoid copying the data
         * in this case.
         */
        BIElem * newBuf = MEM_NewArray(BIElem, newLen);
        if (newBuf) {
            src = b1->value;
            b1->value = newBuf;
            b1->alloc = newLen;
        } else {
            return False;
        }
    } else if (b1->len) {
        /* 
         * the existing storage is already large enough, we have to allocate
         * temporary buffer and copy the data. We could allocate more memory
         * and use the new buffer as the destination, which would not require
         * copying the data but means allocating more space than we really
         * need. Typical tradeoff - CPU vs memory... We save the memory and
         * copy the data because that also makes the code a bit simpler.
         */
        src = MEM_NewArray(BIElem, b1->len);
        if (src) {
            memcpy(src, b1->value, b1->len * sizeof(BIElem));
        } else {
            return False;
        }
    }

    /* do the first iteration to initialize first len2 elements */
    for (i2=0; i2<len2; i2++) {
        product = ((BIElem2)b2->value[i2]) * src[0] + carry;
        b1->value[i2] = (BIElem)product;
        carry = product >> ELEM_BITS;
    }
    b1->value[len2] = (BIElem)carry;

    /* Perform the multiplication word by word */
    for (i1=1; i1<len1; i1++) {
        carry = (BIElem2)0;
        for (i2=0; i2<len2; i2++) {
            product = ((BIElem2)b2->value[i2])*src[i1]+b1->value[i1+i2]+carry;
            b1->value[i1+i2] = (BIElem)product;
            carry = product >> ELEM_BITS;
        }
        b1->value[i1+len2] = (BIElem)carry;
    }

    /* finish up */
    MEM_Free(src);
    b1->len = newLen;
    b1->signum *= b2->signum;
    BIGINT_Normalize(b1);
    BIGINT_Invalidate(b1);
    return True;
}

/**
 * Squares BigInt
 */
Bool BIGINT_Square(BigInt * b)
{
    /* Not the most efficient implementation */
    Bool ok = False;
    BigInt b1;
    BIGINT_Init(&b1);
    if (BIGINT_Copy(&b1, b)) {
        ok = BIGINT_Mul(b, &b1);
    }
    BIGINT_Destroy(&b1);
    return ok;
}

/**
 * This method divides a long quantity by an int to estimate
 * qhat for two multi precision numbers. It is used when 
 * the signed value of n is less than zero.
 */
STATIC void BIGINT_DivWord2(BIElem result[2], BIElem2 n, BIElem d)
{
    if (d == 1) {
        result[0] = (BIElem)n;
        result[1] = 0;
    } else {                
        /* approximate the quotient and remainder */
        BIElem2s d2 = ((BIElem2s)d) & ELEM_MASK;
        BIElem2s q = (n >> 1)/(d2 >> 1);
        BIElem2s r = (BIElem2s)n - q*d2;

        /* correct the approximation */
        while (r < 0) {
            r += d2;
            q--;
        }
        while (r >= d2) {
            r -= d2;
            q++;
        }
        result[0] = (BIElem)q;
        result[1] = (BIElem)r;
    }
}

/**
 * Divides BigInt by divisor and places the remainder to rem.
 * Both dividend and divisor are assumed to be non-zero. The
 * sign of the BigInt is left unchanged. The caller must check
 * the trivial and special cases.
 */
STATIC void BIGINT_DivWord(BigInt * b, BIElem divisor, BIElem * rem)
{
    int i, shift;
    BIElem r;
    BIElem qword[2];
    BIElem2 r2;
    BIElem2 d2 = divisor;

    /* special case of one word dividend */
    if (b->len == 1) {
        int rlen;
        BIElem2 val = b->value[0];
        BIElem qval = (BIElem) (val / d2);
        r = (BIElem)(val - (qval * d2));
        rlen = (r ? 1 : 0);
        *rem = r;
        b->value[0] = qval;
        b->len = (qval ? 1 : 0);
        BIGINT_Invalidate(b);
        return;
    }

    /* normalize the divisor */
    shift = ELEM_BITS - MATH_BitLen(divisor);
    i = b->len-1; 
    r = b->value[i];
    r2 = r;
    if (r2 < d2) {
        b->value[i] = 0;
    } else {
        b->value[i] = (BIElem)(r2/d2);
        r = (BIElem)(r2 - (b->value[i] * d2));
        r2 = r;
    }

    while (--i >= 0) {
        /* estimate the dividend */
        BIElem2 est = (r2 << ELEM_BITS) | ((BIElem2)b->value[i]);
        if (est >= 0) {
            qword[0] = (BIElem)(est/d2);
            qword[1] = (BIElem)(est - (qword[0] * d2));
        } else {
            BIGINT_DivWord2(qword, est, divisor);
        }
        b->value[i] = (BIElem)qword[0];
        r = (BIElem)qword[1];
        r2 = r;
    }
        
    /* unnormalize */
    if (shift > 0) {
        *rem = (r %= divisor);
    } else {
        *rem = r;
    }
    BIGINT_Invalidate(b);
}

/**
 * Divides BigInt by divisor and places the remainder to rem.
 * If divisor is zero, returns False.
 */
Bool BIGINT_DivInt(BigInt * b, int divisor, int * rem)
{
    int r, dsign = 1;

    /* special cases */
    if (!divisor) {
        ASSMSG("BigInt divide by zero");
        return False;
    }
    if (BIGINT_IsZero(b)) {
        if (rem) *rem = 0;
        return True;
    }

    /* extract sign from the divisor */
    if (divisor < 0) {
        divisor = (-divisor);
        dsign = -1;
    }

    if (!rem) rem = &r;
    BIGINT_DivWord(b, divisor, (BIElem*)rem);
    ASSERT((*rem) >= 0);
    (*rem) *= b->signum;
    b->signum *= dsign;
    BIGINT_Normalize(b);
    return True;
}

/**
 * This function is used for division. It multiplies an n word input a by one
 * word input x, and subtracts the n word product from q. This is needed
 * when subtracting qhat*divisor from dividend.
 */
STATIC BIElem BIGINT_MulSub(BIElem * q, const BIElem * a, int len, BIElem x)
{
    BIElem2 x2 = x;
    BIElem2 carry = 0;
    int i;
    for (i=0; i<len; i++) {
        BIElem2 product = ((BIElem2)a[i]) * x2 + carry;
        BIElem2 diff = (BIElem2)(*q) - product;
        (*q++) = (BIElem)diff;
        carry = (product >> ELEM_BITS) + (((diff & ELEM_MASK) >
                     (((~((BIElem)product)) & ELEM_MASK))) ? 1:0);
    }
    return (BIElem)carry;
}

/**
 * A primitive used for division. This function adds in one multiple of the
 * divisor a back to the dividend result at a specified offset. It is used
 * when qhat was estimated too large, and must be adjusted.
 */
STATIC BIElem BIGINT_DivAdd(BIElem * r, int rlen, const BIElem * a, int alen)
{
    BIElem2 carry = 0;
    int i;
    for (i=0; i<alen; i++) {
        BIElem2 sum = ((BIElem2)a[i]) + r[rlen-i-1] + carry;
        r[rlen-i-1] = (BIElem)sum;
        carry = sum >> ELEM_BITS;
    }
    return (BIElem)carry;
}

/**
 * Divides b1 by b2 and places the remainder to rem.
 * If b2 is zero, returns False.
 */
Bool BIGINT_Div(BigInt * b1, const BigInt * b2, BigInt * rem)
{
    BigInt dividend;
    BigInt tmpDivisor;
    const BigInt * divisor = b2;
    int cmp, nlen, limit, j;
    int dsign = b1->signum;
    BIElem shift, borrow, dh, dl;
    BIElem2 dh2;

    if (BIGINT_IsZero(b2)) {
        ASSMSG("BigInt divide by zero");
        return False;
    }

    /* simple cases */
    if (BIGINT_IsZero(b1) || BIGINT_IsOne(b2)) {
        BIGINT_Zero(rem);
        b1->signum *= b2->signum;
        return True;
    }
    b1->signum = b2->signum;
    cmp = BIGINT_Compare(b1, b2) * b1->signum;
    if (cmp < 0) {
        if (BIGINT_Copy(rem, b1)) {
            BIGINT_Zero(b1);
            rem->signum = dsign;
            return True;
        } else {
            b1->signum = dsign;
            return False;
        }
    }
    if (cmp == 0) {
        ASSERT(b1->len > 0);
        BIGINT_Zero(rem);
        b1->value[0] = 1;
        b1->len = 1;
        b1->signum = dsign * b2->signum;
        BIGINT_Invalidate(b1);
        return True;
    }

    /* one word divisor */
    if (b2->len == 1) {
        if (BIGINT_Alloc(rem, 1)) {
            BIGINT_DivWord(b1, b2->value[0], rem->value);
            if (rem->value[0]) {
                rem->len = 1;
                rem->signum = dsign;
            } else {
                rem->len = 0;
                rem->signum = 0;
            }
            b1->signum = dsign * b2->signum;
            BIGINT_Normalize(b1);
            BIGINT_Invalidate(b1);
            BIGINT_Invalidate(rem);
            return True;
        } else {
            b1->signum = dsign;
            return False;
        }
    }

    /* remainder starts as dividend with space for a leading zero */
    if (!BIGINT_Alloc(rem, b1->len+1)) {
        b1->signum = dsign;
        return False;
    }
    BIGINT_Copy(rem, b1);

    /* allocate memory fot the quotient, store the dividend value */
    nlen = rem->len;
    limit = nlen - b2->len + 1;
    dividend = *b1;
    b1->value = MEM_NewArray(BIElem, limit);
    if (!b1->value) {
        b1->value = dividend.value;
        b1->signum = dsign;
        return False;
    }
    memset(b1->value, 0, limit*sizeof(BIElem));
    b1->alloc = limit;
    b1->len = limit;
        
    /* D1. normalize the divisor */
    BIGINT_Init(&tmpDivisor);
    shift = ELEM_BITS - MATH_BitLen(divisor->value[divisor->len-1]);
    if (shift > 0) {
        if (!BIGINT_Copy(&tmpDivisor, divisor)) {
            *b1 = dividend;
            b1->signum = dsign;
            return False;
        }
        BIGINT_QuickLeftShift(&tmpDivisor, shift);
        BIGINT_ShiftLeft(rem, shift); /* memory is already allocated */
        divisor = &tmpDivisor;
    }
       
    /* insert leading 0 in rem if its length did not change */
    if (rem->len == nlen) {
        rem->value[rem->len++] = 0;
    }

    dh = divisor->value[divisor->len-1];
    dl = divisor->value[divisor->len-2];
    dh2 = dh;
        
    /* D2. initialize */
    for(j=0; j<limit; j++) {

        /* D3. calculate qhat */
        /* estimate qhat */
        BIElem qhat = 0;
        BIElem qrem = 0;
        Bool skipCorrection = False;
        BIElem nh = rem->value[rem->len-j-1];
        BIElem nm = rem->value[rem->len-j-2];
        BIElem nh2 = nh + (1 << (ELEM_BITS-1));

        if (nh == dh) {
            qhat = ~((BIElem)0);
            qrem = nh + nm;
            skipCorrection = BoolValue((qrem + ((1 << (ELEM_BITS-1)))) < nh2);
        } else {
            BIElem2s nChunk = (((BIElem2)nh) << ELEM_BITS) | ((BIElem2)nm);
            if (nChunk >= 0) {
                qhat = (BIElem)(nChunk / dh2);
                qrem = (BIElem)(nChunk - (qhat * dh2));
            } else {
                BIElem qword[2];
                BIGINT_DivWord2(qword, nChunk, dh);
                qhat = qword[0];
                qrem = qword[1];
            }
        }

        if (qhat == 0) {
            continue;
        }
            
        if (!skipCorrection) { // Correct qhat
            BIElem2 nl = rem->value[rem->len-j-3];
            BIElem2 rs = (((BIElem2)qrem) << ELEM_BITS) | nl;
            BIElem2 est = ((BIElem2)dl) * ((BIElem2)qhat);

            if (est > rs) {
                qhat--;
                qrem = (BIElem)(qrem + dh2);
                if ((qrem & ELEM_MASK) >=  dh2) {
                    est = (((BIElem2)dl) & ELEM_MASK) * 
                          (((BIElem2)qhat) & ELEM_MASK);
                    rs = (((BIElem2)qrem) << ELEM_BITS) | nl;
                    if (est > rs) {
                        qhat--;
                    }
                }
            }
        }

        /* D4. multiply and subtract */
        rem->value[rem->len-j-1] = 0;
        borrow = BIGINT_MulSub(rem->value + (rem->len - j-1 - divisor->len),
            divisor->value, divisor->len, qhat);

        /* D5. test remainder */
        if ((borrow + (1 << (ELEM_BITS-1))) > nh2) {
            /* D6. add back */
            BIGINT_DivAdd(rem->value+j+1, rem->len-j-1, 
                divisor->value, divisor->len);
            qhat--;
        }

        /* store the quotient digit */
        b1->value[limit-j-1] = qhat;
    }

    /* D8. unnormalize */
    if (shift > 0) {
        if (!BIGINT_ShiftRight(rem, shift)) {
            MEM_Free(b1->value);
            BIGINT_Destroy(&tmpDivisor);
            *b1 = dividend;
            b1->signum = dsign;
            return False;
        }
    }

    rem->signum = dsign;
    b1->signum = dsign * b2->signum;
    MEM_Free(dividend.value);
    BIGINT_Normalize(b1);
    BIGINT_Normalize(rem);
    BIGINT_Invalidate(b1);
    BIGINT_Invalidate(rem);
    BIGINT_Destroy(&tmpDivisor);
    return True;
}

/**
 * Calculates b^p
 * The exponent must be non-negative
 */
Bool BIGINT_Exp(BigInt * b, BigInt * p)
{
    if (BIGINT_IsZero(p)) {
        if (BIGINT_IsZero(b)) {
            return True;
        } else {
            return BIGINT_SetInt(b,1);
        }
    } else if (BIGINT_IsPos(p)) {
        if (BIGINT_IsOne(p)) {
            return True;
        } else if (BIGINT_IsOne(b)) {
            if (BIGINT_IsEven(p)) {
                b->signum = 1;
            }
            return True;
        } else {
            Bool ok = False;
            BigInt r, t, v;
            int bits = BIGINT_BitLen(p);
            BIGINT_Init(&r);
            BIGINT_Init(&t);
            BIGINT_Init(&v);
            if (BIGINT_Copy(&v,b)) {
                if (BIGINT_IsOdd(p)) {
                    ok = BIGINT_Copy(&r,b);
                } else{
                    ok = BIGINT_SetInt(&r,1);
                }
                if (ok) {
                    int i;
                    for (i=1; i<bits && ok; i++) {
                        if (BIGINT_Copy(&t,&v) && BIGINT_Mul(&v,&t)) {
                            if (BIGINT_IsBitSet(p,i)) {
                                ok = BIGINT_Mul(&r,&v);
                            }
                        } else {
                            ok = False;
                        }
                    }
                }
            }
            if (ok) {
                /* swap buffer to avoid copying */
                BigInt tmp = *b;
                *b = r;
                r = tmp;
            }
            BIGINT_Destroy(&r);
            BIGINT_Destroy(&t);
            BIGINT_Destroy(&v);
            return ok;
        }
    } else {
        ASSMSG("Negative exponent!");
        return False;
    }
}

/**
 * Calculates b^p mod m
 * Not very efficient but far better than doing b^p and mod m separately
 */
Bool BIGINT_ExpMod(BigInt * b, BigInt * p, const BigInt * m)
{
    Bool ok = False;
    BigInt t;
    int sign = BIGINT_IsEven(p) ? 1 : b->signum;
    b->signum = 1;
    BIGINT_Init(&t);
    if (BIGINT_AbsCompare(b,m) > 0 || (
        BIGINT_Div(b,m,&t) &&
        BIGINT_Copy(b,&t))) {
        BigInt r, v;
        int bits = BIGINT_BitLen(p);
        BIGINT_Init(&r);
        BIGINT_Init(&v);
        if (BIGINT_Copy(&v,b)) {
            if (BIGINT_IsOdd(p)) {
                ok = BIGINT_Copy(&r,b);
            } else{
                ok = BIGINT_SetInt(&r,1);
            }
            if (ok) {
                int i;
                for (i=1; i<bits && ok; i++) {
                    if (BIGINT_Copy(&t,&v) && 
                        BIGINT_Mul(&t,&v) &&
                        BIGINT_Div(&t,m,&v)) {
                        if (BIGINT_IsBitSet(p,i)) {
                            ok = BIGINT_Mul(&r,&v);
                            if (ok && BIGINT_AbsCompare(&r,m) > 0) {
                                if (!BIGINT_Div(&r,m,&t) || 
                                    !BIGINT_Copy(&r,&t)) {
                                    ok = False;
                                }
                            }
                        }
                    } else {
                        ok = False;
                    }
                }
            }
        }
        if (ok) {
            /* swap buffer to avoid copying */
            BigInt tmp = *b;
            *b = r;
            r = tmp;
            b->signum = sign;
        }
        BIGINT_Destroy(&r);
        BIGINT_Destroy(&v);
    }
    BIGINT_Destroy(&t);
    return ok;
}

/**
 * Solves ab == 1 (mod n)
 * The result is put back to b
 * b must be > 0 && m must be > 1
 */
Bool BIGINT_ModInverse(BigInt * b, const BigInt * m)
{
     Bool ok = False;
     if (BIGINT_IsPos(b) && 
         BIGINT_IsPos(m) && (m->len > 1 || m->value[0] > 1)) {
        if (BIGINT_IsEven(b) && BIGINT_IsEven(m)) {
#if DEBUG && !(defined(_UNIX) && defined(__KERNEL__))
            Str s;
            StrBuf sb;
            STRBUF_Init(&sb);
            s = BIGINT_Format(b,&sb,10);
            TRACE1("SLIB: %s is not invertable ",s);
            TRACE1("mod %s because both are even\n",BIGINT_Format(m,&sb,10));
            STRBUF_Destroy(&sb);
#endif /* DEBUG */
        } else if (BIGINT_IsEven(b) && BIGINT_IsOne(m)) {
            ok = BIGINT_SetInt(b, 1);
        } else {
            BigInt A,B,X,Y,M,D,T;

            BIGINT_Init(&A);
            BIGINT_Init(&B);
            BIGINT_Init(&X);
            BIGINT_Init(&Y);
            BIGINT_Init(&M);
            BIGINT_Init(&D);
            BIGINT_Init(&T);

            if (BIGINT_SetInt(&Y,1) &&
                BIGINT_Copy(&B,b) &&
                BIGINT_Div(&B,m,&A) && /* A = b mod n */
                BIGINT_Copy(&B,m)) {
                int sign = 1;
                ok = True;

                while (!BIGINT_IsZero(&B)) {
                    if (BIGINT_Copy(&D,&A) &&
                        BIGINT_Div(&D,&B,&M)) {

                        BigInt tmp = A;
                        A = B;
                        B = M;
                        M = T;
                        T = D;
                        D = tmp;

                        if (BIGINT_Mul(&T,&X) &&
                            BIGINT_Add(&T,&Y)) {

                            tmp = M;
                            M = Y;
                            Y = X;
                            X = tmp;
                            if (BIGINT_Copy(&X, &T)) {
                                sign= -sign;
                                continue;
                            }
                        }
                    }
                    ok = False;
                    break;
                }

                if (ok) {
                    if (sign < 0) {
                        if (!BIGINT_Sub(&Y,m)) {
                            ok = False;
                        } else {
                            BIGINT_Neg(&Y);
                        }
                    }

                    if (ok) {
                        if (BIGINT_IsOne(&A)) {
                            /* the result goes into A */
                            ok = BIGINT_Div(&Y,m,&A);
                        } else {
                            /* not invertable */
#if DEBUG && !(defined(_UNIX) && defined(__KERNEL__))
                            Str s;
                            StrBuf sb;
                            STRBUF_Init(&sb);
                            s = BIGINT_Format(b,&sb,10);
                            TRACE1("SLIB: %s is not invertable ",s);
                            TRACE1("mod %s\n",BIGINT_Format(m,&sb,10));
                            STRBUF_Destroy(&sb);
#endif /* DEBUG */
                            ok = False;
                        }
                    }
                }
            }

            /* copy the result from A */
            if (ok) {
                ok = BIGINT_Copy(b, &A);
            }

            BIGINT_Destroy(&A);
            BIGINT_Destroy(&B);
            BIGINT_Destroy(&X);
            BIGINT_Destroy(&Y);
            BIGINT_Destroy(&M);
            BIGINT_Destroy(&D);
            BIGINT_Destroy(&T);

         }
     }
     return ok;
}

/**
 * Calculates greatest common divisor of b1 and b2 and places it into gcd
 *
 * Note: this is not the most efficient algorithm. Another candidate is
 * algorithm B from Knuth section 4.5.2. Even better approach is taken 
 * by the authors of the java.math.MutableBigInteger class. They use
 * what they call a "hybrid" method. They use the algorithm below until
 * the numbers are approximately the same length (within the order of
 * magnitude from each other) and then switch to the Knuth algorithm.
 */
Bool BIGINT_Gcd(BigInt * gcd, const BigInt* b1, const BigInt* b2)
{
    if (BIGINT_IsOne(b1) || BIGINT_IsOne(b2)) {
        return BIGINT_SetInt(gcd, 1);
    } else {
        Bool ok = False;
        if (BIGINT_Copy(gcd, b1)) {
            BigInt tmp1;
            BIGINT_Init(&tmp1);
            if (BIGINT_Div(gcd, b2, &tmp1)) {
                ok = BIGINT_Copy(gcd, b2);
                if (ok && !BIGINT_IsZero(&tmp1)) {
                    BigInt tmp2;
                    BigInt * x = gcd;
                    BigInt * y = &tmp1;
                    BigInt * z = &tmp2;
                    BIGINT_Init(&tmp2);
                    while (!BIGINT_IsZero(y)) {
                        BigInt * a = x;
                        if (!BIGINT_Div(x, y, z)) {
                            BIGINT_Destroy(&tmp1);
                            BIGINT_Destroy(&tmp2);
                            return False;
                        }
                        x = y;
                        y = z;
                        z = a;
                    }
                    if (x == gcd) {
                        ok = True;
                    } else {
                        ok = BIGINT_Copy(gcd, x);
                    }
                    gcd->signum = 1;
                    BIGINT_Destroy(&tmp2);
                }
            }
            BIGINT_Destroy(&tmp1);
        }
        return ok;
    }
}

/**
 * Writes a string represendation of this BigInt to a string buffer.
 */
#if !(defined(_UNIX) && defined(__KERNEL__))
Str BIGINT_Format(const BigInt * b, StrBuf * sb, int radix)
{
    BigInt tmp;
    if (radix == 0) {
        radix = DEFAULT_RADIX;
    } else {
        ASSERT(radix >= MIN_RADIX);
        ASSERT(radix <= MAX_RADIX);
        if (radix < MIN_RADIX) {
            radix = MIN_RADIX;
        } else if (radix > MAX_RADIX) {
            radix = MAX_RADIX;
        }
    }
    if (!sb || !b) return NULL;
    STRBUF_Clear(sb);
    BIGINT_Init(&tmp);
    if (BIGINT_Copy(&tmp, b)) {
        Bool ok = True;
        StrBuf32 tmpBuf;
        StrBuf * buf = &tmpBuf.sb;
        STRBUF_InitBufXXX(&tmpBuf);
        if (BIGINT_IsNeg(&tmp)) BIGINT_Neg(&tmp);
        while (!BIGINT_IsZero(&tmp) && ok) {
            int rem;
            if (BIGINT_DivInt(&tmp, radix, &rem)) {
                if (STRBUF_AppendChar(buf, DIGITS[rem])) {
                    continue;
                }
            }
            ok = False;
        }
        if (ok) {
            if (BIGINT_IsNeg(b)) {
                ok = STRBUF_AppendChar(buf,'-');
            } else if (STRBUF_Length(buf) == 0) {
                STRBUF_AppendChar(buf,'0');
            }
            if (ok) {
                size_t minlen = STRBUF_Length(sb) + STRBUF_Length(buf);
                if (STRBUF_Alloc(sb, minlen)) {
                    /* nothing can fail from this point on */
                    size_t i = STRBUF_Length(buf);
                    while (i>0) {
                        STRBUF_AppendChar(sb, STRBUF_CharAt(buf, --i));
                    }
                    STRBUF_Destroy(buf);
                    BIGINT_Destroy(&tmp);
                    return STRBUF_Text(sb);
                }
            }
        }
        STRBUF_Destroy(buf);
    }
    BIGINT_Destroy(&tmp);
    STRBUF_Clear(sb);
    return NULL;
}
#endif

/**
 * Translates a character into value < radix
 * Returns -1 if character is not translatable
 */
STATIC int BIGINT_CharValue(Char c, int radix)
{
    int i;
    if (c >= '0' && c <= '9') {
        i = c - '0';
    } else if (c >= 'a' && c <= 'z') {
        i = (c - 'a') + 10;
    } else if (c >= 'A' && c <= 'Z') {
        i = (c - 'A') + 10;
    } else {
        return -1;
    }
    return (i < radix) ? i : -1;
}

/** 
 * Parses the string into a big integer
 */
Bool BIGINT_Parse(BigInt * b, Str s, int radix)
{
    BIGINT_Zero(b);
    if (radix > 0 && radix <= MAX_RADIX) {
        Char c = *s;
        while (c && IsSpace(c)) c = *(++s);
        if (c) {

            /* skip the sign character */
            int sign = 1;
            if (c == '-') {
                sign = -1;
                c = *(++s);
            } else if (c == '+') {
                c = *(++s);
            }

            while (c && !IsSpace(c)) {
                int i = BIGINT_CharValue(c, radix);
                if (i < 0 ||
                    !BIGINT_MulInt(b, radix) ||
                    !BIGINT_Add2(b, (BIElem*)&i, 1)) {
                    BIGINT_Zero(b);
                    return False;
                }
                c = *(++s);
            }
            if (IsSpace(c)) {
                while (c && IsSpace(c)) c = *(++s);
                if (c) return False;
            }
            b->signum = sign;
            BIGINT_Normalize(b);
            return True;
        }
    }
    BIGINT_Zero(b);
    return False;
}

/*
 * HISTORY:
 *
 * $Log: s_mpm.c,v $
 * Revision 1.17  2010/09/25 09:31:32  slava
 * o made it possible to compile some slib modules in Mac OS X kernel build
 *   environment
 *
 * Revision 1.16  2010/06/26 09:26:42  slava
 * o use MEM_Equal instead of memcmp to resolve linking problem with some
 *   versions of Windows DDK
 *
 * Revision 1.15  2010/06/21 16:50:30  slava
 * o a few changes to compile parts of slib in Linux kernel environment
 *
 * Revision 1.14  2009/05/23 09:11:58  slava
 * o fixed a recently introduced bug in BIGINT_Format
 *
 * Revision 1.13  2009/04/09 21:54:57  slava
 * o use size_t instead of int where it's appropriate
 *
 * Revision 1.12  2007/06/03 23:28:59  slava
 * o removed unused function BIGINT_Init2
 *
 * Revision 1.11  2004/10/20 00:52:31  slava
 * o fixed a few pendantic compilation warnings
 * o made BIGINT_CharValue static
 *
 * Revision 1.10  2004/08/27 19:13:13  slava
 * o fixed bugs in BIGINT_Exp and BIGINT_ExpMod (sign could be wrong)
 * o added BIGINT_AbsCompare function
 *
 * Revision 1.9  2004/08/27 04:16:44  slava
 * o fixed gcc compilation warnings
 *
 * Revision 1.8  2004/08/27 04:09:15  slava
 * o added BIGINT_ExpMod function which calculates (b^p mod m) far more
 *   efficiently than if you first do (b^p) and then (mod m), even though
 *   it is using the most primitive algorithm. As always, there's a lot of
 *   room for improvement. Anyway, the bottom line is that now this module
 *   contains all the necessary ingredients for implementing many popular
 *   cryptographic algorithms, such as DSA signature verification. QED.
 *
 * Revision 1.7  2004/08/27 02:16:33  slava
 * o added BIGINT_Zero, BIGINT_IsBitSet, BIGINT_Exp, BIGINT_Square and
 *   BIGINT_ModInverse functions
 *
 * Revision 1.6  2004/08/26 16:29:00  slava
 * o added BIGINT_Parse, BIGINT_IsOdd and BIGINT_IsEven functions
 * o fixed a bug in BIGINT_Equal, introduced in rev 1.5
 *
 * Revision 1.5  2004/04/08 01:44:18  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.4  2003/08/28 17:16:23  slava
 * o replaced tabs with spaces
 *
 * Revision 1.3  2003/08/01 05:34:25  slava
 * o fixed a bug introduced in rev 1.2 as a side effect of "fixing" gcc
 *   compilation warnings
 *
 * Revision 1.2  2003/07/31 16:13:22  slava
 * o fixed gcc compilation warnings
 *
 * Revision 1.1  2003/07/31 15:15:46  slava
 * o multiple precision arithmetic
 *
 * Local Variables:
 * c-basic-offset: 4
 * compile-command: "make -C .."
 * indent-tabs-mode: nil
 * End:
 */
