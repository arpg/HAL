/*
 * $Id: s_bitset.c,v 1.18 2010/06/26 09:26:42 slava Exp $
 *
 * Copyright (C) 2001-2010 by Slava Monich
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

#include "s_bitset.h"
#include "s_mem.h"

/*
 * BitSets are packed into arrays of "units."  A unit is a long, which 
 * consists of 64 or 32 bits, depending on CPU, OS and the compiler. 
 * 64 bits require 6 address bits, 32 bits require 5 address bits. 
 */
#ifdef __LONG_64__
COMPILE_ASSERT(sizeof(BitUnit) == 8)
#  define BITSET_ADDRESS_BITS_PER_UNIT 6
#else  /* __LONG_64__ */
COMPILE_ASSERT(sizeof(BitUnit) == 4)
#  define BITSET_ADDRESS_BITS_PER_UNIT 5
#endif /* __LONG_64__ */

#define BITSET_BITS_PER_UNIT        (1 << BITSET_ADDRESS_BITS_PER_UNIT)
#define BITSET_BIT_INDEX_MASK       (BITSET_BITS_PER_UNIT - 1)

/** given a bit index returns unit index containing it. */
#define UNIT_INDEX(_bitIndex) \
  (((unsigned int)(_bitIndex)) >> BITSET_ADDRESS_BITS_PER_UNIT)

/* given a bit index, returns a unit that masks that bit in its unit. */
#define BIT(_bitIndex) \
  (((BitUnit)1) << ((_bitIndex) & BITSET_BIT_INDEX_MASK))

/* return pointer to the storage. */
#define UNITS(_bs) \
  (((_bs)->alloc) ? ((_bs)->storage.units) : (&((_bs)->storage.unit)))

/*==========================================================================*
 *              B I T S E T
 *==========================================================================*/

/**
 * Allocates a new BitSet
 */
BitSet * BITSET_Create()
{
    BitSet * bs = MEM_New(BitSet);
    if (bs) {
        BITSET_Init(bs);
        return bs;
    }
    return NULL;
}

/**
 * Creates a clone of the bitset
 */
BitSet * BITSET_Clone(const BitSet * bs)
{
    BitSet * clon = BITSET_Create();
    if (clon) {
        if (BITSET_Copy(clon, bs)) {
            return clon;
        }
        BITSET_Destroy(clon);
    }
    return NULL;
}

/**
 * Deletes the BitSet
 */
void BITSET_Delete(BitSet * bs)
{
    if (bs) {
        BITSET_Destroy(bs);
        MEM_Free(bs);
    }
}

/**
 * Initializes the BitSet
 */
void BITSET_Init(BitSet * bs)
{
    memset(bs, 0, sizeof(*bs));
}

/**
 * Destroys the BitSet
 */
void BITSET_Destroy(BitSet * bs)
{
    if (bs->alloc) {
        ASSERT(bs->storage.units);
        MEM_Free(bs->storage.units);
    }
    memset(bs, 0, sizeof(*bs));
}

/**
 * Free unused memory.
 */
void BITSET_Trim(BitSet * bs)
{
    if (bs->alloc > bs->inuse) {
        ASSERT(bs->storage.units);
        if (bs->inuse == 0) {
            MEM_Free(bs->storage.units);
            bs->storage.units  = NULL;
            bs->alloc = 0;
        } else if (bs->inuse == 1) {
            BitUnit unit = UNITS(bs)[0];
            MEM_Free(bs->storage.units);
            bs->storage.unit = unit;
            bs->alloc = 0;
        } else {
            BitUnit * savebuf = bs->storage.units;
            size_t size = sizeof(BitUnit) * bs->inuse;

#ifdef NO_REALLOC
            bs->storage.units = (BitUnit*)MEM_Alloc(size);
            if (bs->storage.units) {
                memcpy(bs->storage.units, savebuf, size);
                MEM_Free(bs->storage.units);
            }
#else
            bs->storage.units = (BitUnit*)MEM_Realloc(savebuf, size);
#endif /* NO_REALLOC */

            if (bs->storage.units) {
                bs->alloc = bs->inuse;
            } else {
                bs->storage.units = savebuf;
            }
        }
    }
}

/**
 * Returns the number of bits of space actually in use by the BitSet 
 * to represent bit values. The maximum element in the set is the 
 * size - 1st element. Note that this is different from the "length" 
 * of the bitset (see comments on BITSET_Length function below)
 */
int BITSET_Size(const BitSet * bs)
{
    int alloc = MAX(bs->alloc,1);
    return (alloc << BITSET_ADDRESS_BITS_PER_UNIT);
}

/**
 * Returns the "logical size" of the BitSet: the index of the highest 
 * set bit in the BitSet plus one. Returns zero if the BitSet contains 
 * no set bits.
 */
int BITSET_Length(const BitSet * bs)
{
    if (bs->inuse > 0) {
        int highestBit = (bs->inuse - 1) * sizeof(BitUnit) * 8;
        BitUnit highestUnit = UNITS(bs)[bs->inuse - 1];
        do {
            highestUnit >>= 1;
            highestBit++;
        } while (highestUnit > 0);
        return highestBit;
    }
    return 0;
}

/**
 * Calculates the number of bits set
 */
int BITSET_BitCnt(const BitSet * bs)
{
    int i, n = 0;
    for (i=0; i<bs->inuse; i++) {
        BitUnit u = UNITS(bs)[i];
        /* subtracting 1 from any number causes all bits up to and including
         * the least significant non-zero bit to be complemented. */
        while (u) {
            u &= u-1;  
            n++;
        }
    }
    return n;
}

/**
 * Returns the hash code for this BitSet. Equal bitsets are quaranteed
 * to have the same hash code.
 */
int BITSET_HashCode(const BitSet * bs)
{
    int i;
    BitUnit h = 1234;
    for (i = bs->inuse; --i >= 0;) {
        h ^= UNITS(bs)[i] * (i+1);
    }
#ifdef __LONG_64__
    return (int)((h >> 32) ^ h);
#else /* __LONG_64__ */
    return h;
#endif /* __LONG_64__ */
}

/**
 * Returns the value of the bit with the specified index. The value 
 * is True if the bit with the index bitIndex is currently set in 
 * this BitSet; otherwise, the result is False.
 */
Bool BITSET_Get(const BitSet * bs, int bitIndex)
{
    Bool result = False;
    ASSERT(bitIndex >= 0);
    if (bitIndex >= 0) {
        int unitIndex = UNIT_INDEX(bitIndex);
        if (unitIndex < bs->inuse) {
            result = BoolValue((UNITS(bs)[unitIndex] & BIT(bitIndex)) != 0);
        }
    }
    return result;
}

/**
 * Ensures that the BitSet can hold enough units.
 */
STATIC Bool BITSET_EnsureCapacity(BitSet * bs, int unitsRequired) 
{
    if (unitsRequired > 1 && bs->alloc < unitsRequired) {

        /* allocate larger of doubled size or required size */
        int request = MAX(2 * bs->alloc, unitsRequired);
        BitUnit * newUnits = MEM_NewArray(BitUnit,request);
        if (newUnits) {

            /* clear unused portion of the array */
            int nbytes = (request - bs->alloc)*sizeof(BitUnit);
            memset(newUnits+bs->alloc, 0, nbytes);

            /* copy old contents and free old buffer */
            if (bs->alloc > 0) {
                nbytes = bs->alloc*sizeof(BitUnit);
                memcpy(newUnits, bs->storage.units, nbytes);
                MEM_Free(bs->storage.units);
            } else {
                newUnits[0] = bs->storage.unit;
            }

            /* successfully reallocated the buffer */
            bs->alloc = request;
            bs->storage.units = newUnits;
            return True;
        }

        /* memory allocation failed */
        return False;
    }

    /* current size is sufficient */
    return True;
}

/**
 * Ensures that the bitset has enough memory to hold the specified number
 * of bits. Returns False if memory allocation fails.
 */
Bool BITSET_Alloc(BitSet * bs, int nbits)
{
    ASSERT(nbits >= 0);
    if (nbits > 0) {
       return BITSET_EnsureCapacity(bs, UNIT_INDEX(nbits-1)+1);
    } else if (nbits == 0) {
        return True;
    } else {
        return False;
    }
}

/**
 * Sets the bit specified by the index to True. Returns False if memory
 * allocation fails; otherwise returns True
 */
Bool BITSET_Set(BitSet * bs, int bitIndex)
{
    ASSERT(bitIndex >= 0);
    if (bitIndex >= 0) {
        int unitIndex = UNIT_INDEX(bitIndex);
        int unitsRequired = unitIndex + 1;

        /* reallocate the buffer if necessary */
        if (bs->inuse < unitsRequired) {
            if (!BITSET_EnsureCapacity(bs,unitsRequired)) {
                return False;
            }
            bs->inuse = unitsRequired;
        }

        UNITS(bs)[unitIndex] |= BIT(bitIndex);
        return True;
    } else {
        return False;
    }
}

/**
 * Sets the field 'inuse' with the logical size in units of the bit
 * set. WARNING:This function assumes that the number of units actually
 * in use is less than or equal to the current value of 'inuse'!
 */
STATIC void BITSET_UpdateUnitsInUse(BitSet * bs) 
{
    int i;
    BitUnit * units = UNITS(bs);
    for (i=bs->inuse-1; i>=0 && !units[i]; i--) NOTHING;
    bs->inuse = i+1; /* the new logical size */
}

/**
 * Sets the bit specified by the index to False.
 */
void BITSET_Clear(BitSet * bs, int bitIndex)
{
    ASSERT(bitIndex >= 0);
    if (bitIndex >= 0) {
        int unitIndex = UNIT_INDEX(bitIndex);
        if (unitIndex < bs->inuse) {
            BitUnit * units = UNITS(bs);
            units[unitIndex] &= ~BIT(bitIndex);
            if (!units[bs->inuse - 1]) {
                BITSET_UpdateUnitsInUse(bs);
            }
        }
    }
}

/**
 * Sets all the bits to False.
 */
void BITSET_ClearAll(BitSet * bs)
{
    if (bs->inuse > 0) {

        /* 
         * NOTE: the rest of the code assumes that the unused part
         * of the array is filled with zeros.
         */
        BitUnit * bu = UNITS(bs);
        memset(bu, 0, sizeof(BitUnit)*bs->inuse);
        bs->inuse = 0;
    }
}

/**
 * Copies contents from one bitset to another
 */
Bool BITSET_Copy(BitSet * bs1, const BitSet * bs2)
{
    int unitsToCopy = bs2->inuse;
    if (unitsToCopy > 1 && bs1->alloc < unitsToCopy) {
        BitUnit * newUnits = MEM_NewArray(BitUnit,unitsToCopy);
        if (newUnits) {
            bs1->inuse = 0;
            bs1->alloc = unitsToCopy;
            bs1->storage.units = newUnits;
            memset(newUnits, 0, sizeof(BitUnit)*unitsToCopy);
        } else {
            return False;
        }
    }
    
    bs1->inuse = bs2->inuse;
    memcpy(UNITS(bs1), UNITS(bs2), sizeof(BitUnit)*unitsToCopy);
    return True;
}

/**
 * Performs a logical AND of bs1 with bs2. 
 * bs1 is modified so that each bit in it has the value True if and 
 * only if it both initially had the value True and the corresponding 
 * bit in bs2 also had the value True. 
 */
void BITSET_And(BitSet * bs1, const BitSet * bs2)
{
    if (bs1 != bs2) {
        int i;
        BitUnit * bu1 = UNITS(bs1); 
        const BitUnit * bu2 = UNITS(bs2); 

        /* perform logical AND on bits in common  */
        int oldUnitsInUse = bs1->inuse;
        bs1->inuse = MIN(bs1->inuse, bs2->inuse);

        for(i=0; i<bs1->inuse; i++) {
            bu1[i] &= bu2[i];
        }

        /* clear out units no longer used */
        if (i < oldUnitsInUse) {
            memset(bu1 + i, 0, (oldUnitsInUse - i)*sizeof(BitUnit));
        }

        /* Recalculate units in use if necessary */
        if (bs1->inuse > 0 && bu1[bs1->inuse - 1] == 0) {
            BITSET_UpdateUnitsInUse(bs1);
        }
    }
}

/**
 * Clears all of the bits in bs1 whose corresponding bit is set in bs2
 */
void BITSET_AndNot(BitSet * bs1, const BitSet * bs2)
{
    BitUnit * bu1 = UNITS(bs1); 
    const BitUnit * bu2 = UNITS(bs2); 

    /* perform logical (a & !b) on bits in common */
    int i, unitsInCommon = MIN(bs1->inuse, bs2->inuse);
    for (i=0; i<unitsInCommon; i++) {
        bu1[i] &= ~bu2[i];
    }

    BITSET_UpdateUnitsInUse(bs1);
}

/**
 * Performs a logical OR of bs1 with bs2. 
 * bs1 is modified so that a bit in it has the value True 
 * if and only if it either already had the value True or 
 * the corresponding bit in bs2 has the value True.
 *
 * Returns False if memory allocation fails; otherwise 
 * returns True;
 */
Bool BITSET_Or(BitSet * bs1, const BitSet * bs2)
{
    if (bs1 != bs2) {
        if (BITSET_EnsureCapacity(bs1, bs2->inuse)) {
            BitUnit * bu1 = UNITS(bs1); 
            const BitUnit * bu2 = UNITS(bs2); 

            /*  perform logical OR on bits in common */
            int i, unitsInCommon = MIN(bs1->inuse, bs2->inuse);
            for(i=0; i<unitsInCommon; i++) {
                bu1[i] |= bu2[i];
            }

            /* copy any remaining bits */
            if (i < bs2->inuse) {
                memcpy(bu1 + i, bu2 + i, (bs2->inuse - i) * sizeof(BitUnit));
            }

            if (bs1->inuse < bs2->inuse) {
                bs1->inuse = bs2->inuse;
            }
            return True;
        }

        /* memory allocation failed */
        return False;
    }

    /* both pointers point to the same bit set */
    return True;
}

/**
 * Performs a logical XOR of bs1 with bs2. 
 * bs2 is modified so that a bit in it has the value True 
 * if and only if one of the following statements holds: 
 * 
 * 1) the bit initially has the value True, and the 
 *     corresponding bit in bs2 has the value False.
 *
 * 2) the bit initially has the value False, and the 
 *     corresponding bit in bs2 has the value True.
 *
 */
Bool BITSET_Xor(BitSet * bs1, const BitSet * bs2)
{
    int i, unitsInCommon;
    BitUnit * bu1; 
    const BitUnit * bu2; 

    if (bs1->inuse >= bs2->inuse) {
        unitsInCommon = bs2->inuse;
    } else {
        if (!BITSET_EnsureCapacity(bs1, bs2->inuse)) {
            return False;
        }
        unitsInCommon = bs1->inuse;
        bs1->inuse = bs2->inuse;
    }

    bu1 = UNITS(bs1); 
    bu2 = UNITS(bs2); 

    /* perform logical XOR on bits in common */
    for (i=0; i<unitsInCommon; i++) {
        bu1[i] ^= bu2[i];
    }

    /* copy any remaining bits */
    if (i < bs2->inuse) {
        memcpy(bu1 + i, bu2 + i, (bs2->inuse - i) * sizeof(BitUnit));
    }

    BITSET_UpdateUnitsInUse(bs1);
    return True;
}

/**
 * Compares the contents of the two bitsets
 */
Bool BITSET_Equal(const BitSet * bs1, const BitSet * bs2)
{
    if (bs1 == bs2) {
        return True;
    } else if (bs1->inuse == bs2->inuse) {
        if (!bs1->inuse) {
            return True;
        } else {
            const BitUnit * bu1 = UNITS(bs1);
            const BitUnit * bu2 = UNITS(bs2);
            if (MEM_Equal(bu1, bu2, bs1->inuse * sizeof(BitUnit))) {
                return True;
            }
        }
    }
    return False;
}

/**
 * Returns the index of the next bit *after* the specified position.
 * Position -1 starts search from the first bit in the bitset. If no
 * bits are set after the specified position, return -1
 */
int BITSET_NextBit(const BitSet * bs, int startPos)
{
    int unitIndex;
    const BitUnit * bu = UNITS(bs); 
    
    /* check the unit containing the bit after which we will search */
    if (startPos >= 0) {
        if ((startPos & BITSET_BIT_INDEX_MASK) == BITSET_BIT_INDEX_MASK) {
            /* start with the next unit */
            startPos++;
        } else {
            unitIndex = UNIT_INDEX(startPos);
            if (unitIndex >= bs->inuse) {
                return -1;
            } else {
                BitUnit unit = bu[unitIndex];
                if (unit) {
                    int i = (startPos & BITSET_BIT_INDEX_MASK);
                    if (i < (BITSET_BITS_PER_UNIT-1)) {
                        BitUnit upperBits = ~((1UL << (i+1)) - 1UL);
                        if (unit & upperBits) {
                            for(i++; i<BITSET_BITS_PER_UNIT;i++) {
                                if (unit & (1 << i)) {
                                    startPos &= (~BITSET_BIT_INDEX_MASK);
                                    return (startPos + i);
                                }
                            }
                        }
                    }
                }
                /* start with the next unit */
                startPos = (startPos | BITSET_BIT_INDEX_MASK)+1;
            }
        }
    } else {
        startPos = 0;
    }

    /* now scan full units */
    for (unitIndex=UNIT_INDEX(startPos); unitIndex<bs->inuse; unitIndex++) {
        BitUnit unit = bu[unitIndex];
        if (unit) {
            int bitIndex = 0;
            BitUnit bit = (unit & (-((long)unit)));

            /*
             * reduce number of shifts that we need to do in order to
             * calculate the bit index 
             */
#ifdef __LONG_64__
            if (bit >= 0x0000000100000000UL) {
                if (bit >= 0x0001000000000000UL) {
                    bitIndex = 48;
                } else {
                    bitIndex = 32;
                }
            } else 
#endif /* __LONG_64__ */
            if (bit >= 0x00010000UL) {
                bitIndex = 16;
            }
            bit >>= bitIndex;
            while (bit > 1) {
                bit >>= 1;
                bitIndex++;
            }
            return unitIndex * BITSET_BITS_PER_UNIT + bitIndex;
        }
    }

    /* could not find anything */
    return -1;
}

/*
 * HISTORY:
 *
 * $Log: s_bitset.c,v $
 * Revision 1.18  2010/06/26 09:26:42  slava
 * o use MEM_Equal instead of memcmp to resolve linking problem with some
 *   versions of Windows DDK
 *
 * Revision 1.17  2006/09/25 06:31:24  slava
 * o protection against invalid input parameters (negative bit index)
 *
 * Revision 1.16  2006/09/23 23:30:51  slava
 * o added BITSET_Copy function
 *
 * Revision 1.15  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.14  2004/04/08 01:44:18  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.13  2004/03/23 13:31:12  slava
 * o fixed gcc compilation warning
 *
 * Revision 1.12  2004/03/16 06:23:23  slava
 * o added BITSET_BitCnt and BITSET_ClearAll functions
 *
 * Revision 1.11  2003/05/21 00:11:03  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.10  2003/04/29 18:04:09  slava
 * o added BITSET_Alloc function
 *
 * Revision 1.9  2003/03/11 08:12:25  slava
 * o added BITSET_Clone
 *
 * Revision 1.8  2003/02/24 22:58:22  slava
 * o added BITSET_Create, BITSET_Delete and BITSET_HashCode
 *
 * Revision 1.7  2002/08/12 05:20:37  slava
 * o fixed gcc compilation warning (unused variable)
 *
 * Revision 1.6  2002/08/09 11:01:31  slava
 * o added BITSET_NextBit and BITSET_Trim functions
 *
 * Revision 1.5  2002/06/20 12:55:24  slava
 * o added BITSET_Equal function
 *
 * Revision 1.4  2002/01/10 05:28:11  slava
 * o minor changes
 *
 * Revision 1.3  2001/12/28 02:48:34  slava
 * o fixed gcc compilation warning
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
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
