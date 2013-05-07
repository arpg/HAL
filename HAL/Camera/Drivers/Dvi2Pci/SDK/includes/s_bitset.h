/*
 * $Id: s_bitset.h,v 1.10 2006/09/23 23:30:51 slava Exp $
 *
 * Copyright (C) 2001-2006 by Slava Monich
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

#ifndef _SLAVA_BITSET_H_
#define _SLAVA_BITSET_H_

#include "s_def.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/*
 * a vector of bits that grows as needed. The bits of a BitSet are indexed 
 * by nonnegative integers. Individual indexed bits can be examined, set, 
 * or cleared. One BitSet may be used to modify the contents of another 
 * BitSet through logical AND, logical inclusive OR, and XOR operations.
 */
typedef unsigned long BitUnit;
typedef struct _BitSet {
    int inuse;              /* number of units in use */
    int alloc;              /* number of allocated units */
    union _BitUnits {
        BitUnit unit;       /* single unit */
        BitUnit * units;    /* the bits in this BitSet */
    } storage;
} BitSet;

/* operations */
extern BitSet * BITSET_Create P_((void));
extern BitSet * BITSET_Clone P_((const BitSet * bs));
extern void BITSET_Delete P_((BitSet * bs));
extern void BITSET_Init P_((BitSet * bs));
extern void BITSET_Destroy P_((BitSet * bs));
extern void BITSET_Trim P_((BitSet * bs));
extern int  BITSET_Size P_((const BitSet * bs));
extern int  BITSET_Length P_((const BitSet * bs));
extern int  BITSET_BitCnt P_((const BitSet * bs));
extern int  BITSET_HashCode P_((const BitSet * bs));
extern Bool BITSET_Alloc P_((BitSet * bs, int nbits));
extern Bool BITSET_Get P_((const BitSet * bs, int bitIndex));
extern Bool BITSET_Set P_((BitSet * bs, int bitIndex));
extern void BITSET_Clear P_((BitSet * bs, int bitIndex));
extern void BITSET_ClearAll P_((BitSet * bs));
extern Bool BITSET_Copy P_((BitSet * bs1, const BitSet * bs2));
extern void BITSET_And P_((BitSet * bs1, const BitSet * bs2));
extern void BITSET_AndNot P_((BitSet * bs1, const BitSet * bs2));
extern Bool BITSET_Or P_((BitSet * bs1, const BitSet * bs2));
extern Bool BITSET_Xor P_((BitSet * bs1, const BitSet * bs2));
extern Bool BITSET_Equal P_((const BitSet * bs1, const BitSet * bs2));
extern int  BITSET_NextBit P_((const BitSet * bs, int pos));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_BITSET_H_ */

/*
 * HISTORY:
 *
 * $Log: s_bitset.h,v $
 * Revision 1.10  2006/09/23 23:30:51  slava
 * o added BITSET_Copy function
 *
 * Revision 1.9  2004/03/16 06:23:23  slava
 * o added BITSET_BitCnt and BITSET_ClearAll functions
 *
 * Revision 1.8  2003/04/29 18:04:09  slava
 * o added BITSET_Alloc function
 *
 * Revision 1.7  2003/03/11 08:12:25  slava
 * o added BITSET_Clone
 *
 * Revision 1.6  2003/02/24 22:58:22  slava
 * o added BITSET_Create, BITSET_Delete and BITSET_HashCode
 *
 * Revision 1.5  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.4  2002/08/09 11:01:31  slava
 * o added BITSET_NextBit and BITSET_Trim functions
 *
 * Revision 1.3  2002/06/20 12:55:23  slava
 * o added BITSET_Equal function
 *
 * Revision 1.2  2001/12/28 02:39:40  slava
 * o optimized bitset for the case when bit count is less than sizeof(long)
 *   by using the same field as a storage (if all bits fit into a long), and
 *   as a pointer to external buffer
 *
 * Revision 1.1  2001/12/28 00:41:16  slava
 * o added BitSet functions
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
