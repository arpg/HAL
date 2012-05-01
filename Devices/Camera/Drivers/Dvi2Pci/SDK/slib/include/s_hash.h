/*
 * $Id: s_hash.h,v 1.17 2010/12/19 17:56:39 slava Exp $
 *
 * Copyright (C) 2000-2010 by Slava Monich
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

#ifndef _SLAVA_HASH_H_
#define _SLAVA_HASH_H_

#include "s_itr.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* data types */
typedef I32s  HashCode;
typedef struct _HashTable HashTable;
typedef void * HashKey;
typedef const void * HashKeyC;
typedef void * HashValue;
typedef const void * HashValueC;

/* 
 * default value returned by HASH_Get() if key is not mapped. If this 
 * happens to be a valid value, you should either do HASH_Contains() 
 * before doing each HASH_Get() or fix the nullValue field of the 
 * HashTable structure
 */
#define NULL_VALUE ((HashValue)0)

/* 
 * callback functions. 
 *
 * HashCompare - returns True if hash keys are equal, False if they are not
 * HashProc    - returns a hash code for the specified key
 * HashCB      - callback for use with HASH_Examine(). Returns True to 
 *               continue, False to stop. The value returned by HASH_Examine()
 *               is the last value returned by the callback, or True if 
 *               hashtable is empty. The callback routine is allowed to 
 *               remove the current element. Removing any other element 
 *               is not allowed and may cause a crash. 
 *
 * HashFree    - cleanup procedure invoked when a key/value pair gets 
 *               removed from the hashtable.
 */
typedef Bool (*HashCompare)P_((HashKeyC key1, HashKeyC key2));
typedef HashCode (*HashProc)P_((HashKeyC key));
typedef void (*HashFree)P_((HashKey key, HashValue value));
typedef Bool (*HashCB)P_((HashKey key, HashValue value, void * ctx));

/*
 * A few simple callback functions.
 *
 * hashFreeNothingProc - HashFree callback that doesn't do anything
 * hashFreeKeyProc - HashFree callback that deallocates the key
 * hashFreeValueProc - HashFree callback that deallocates the value
 * hashFreeKeyValueProc - HashFree callback that deallocates key and value
 *
 * hashCompareStringKey - HashCompare callback, assumes keys are ASCIIZ strings
 *
 * stringHashProc - returns hash code for a string key
 */
extern void hashFreeNothingProc P_((HashKey key, HashValue value));
extern void hashFreeKeyProc P_((HashKey key, HashValue value));
extern void hashFreeValueProc P_((HashKey key, HashValue value));
extern void hashFreeKeyValueProc P_((HashKey key, HashValue value));
extern Bool hashDefaultCompare P_((HashKeyC key1, HashKeyC key2));
extern Bool hashCompareStringKey P_((HashKeyC key1, HashKeyC key2));
extern Bool hashCaseCompareStringKey P_((HashKeyC key1, HashKeyC key2));
extern HashCode hashDefaultHashProc P_((HashKeyC key));
extern HashCode stringHashProc P_((HashKeyC key));
extern HashCode stringCaseHashProc P_((HashKeyC key));

/*
 * the hashtable itself. 
 */
typedef struct _HashBucket HashBucket;
struct _HashTable {
    long  count;                /* number of key-value pairs in the table */
    short primeIndex;           /* current prime index */
    short loadFactor;           /* the load factor * 100 */
    HashValue    nullValue;     /* the NULL value */
    HashProc     hasher;        /* hash function */
    HashCompare  equals;        /* compare function */
    HashFree     free;          /* cleanup function */
    HashBucket** buckets;       /* hashPrimes[primeIndex] slots */
};

typedef struct _HashEntry {
    HashKey   key;
    HashValue value;
} HashEntry;

/* initialize/deinitialize global data */
extern void HASH_InitModule P_((void));
extern void HASH_Shutdown P_((void));

/* operation on hash tables */
extern HashTable * HASH_Create P_((long size, HashCompare c, 
                                   HashProc h, HashFree f));

extern Bool HASH_Init P_((HashTable * ht, long size, HashCompare c, 
                          HashProc h, HashFree f));
extern void HASH_Reinit P_((HashTable * ht, HashCompare c, 
                           HashProc h, HashFree f));

extern void HASH_Destroy  P_((HashTable * ht));
extern void HASH_Delete   P_((HashTable * ht));
extern void HASH_Clear    P_((HashTable * ht));
extern long HASH_Size     P_((const HashTable * ht));
extern void HASH_Rehash   P_((HashTable * ht, long size));
extern Bool HASH_Put      P_((HashTable * ht, HashKey key, HashValue val));
extern Bool HASH_TryPut   P_((HashTable * ht, HashKey key, HashValue val));
extern Bool HASH_Update   P_((HashTable * ht, HashKey key, HashValue val));
extern HashValue HASH_Get P_((const HashTable * ht, HashKeyC key));
extern Bool HASH_Contains P_((const HashTable * ht, HashKeyC key));
extern Bool HASH_Remove   P_((HashTable * ht, HashKeyC key));
extern Bool HASH_Copy     P_((HashTable * dest, const HashTable * src));
extern Bool HASH_Examine  P_((const HashTable * ht, HashCB cb, void * ctx));

/* iterators */
extern Iterator * HASH_Keys    P_((HashTable * ht)); /* returns HashKey */
extern Iterator * HASH_Values  P_((HashTable * ht)); /* returns HashValue */
extern Iterator * HASH_Entries P_((HashTable * ht)); /* returns HashEntry */

/* same as the above but don't support ITR_Remove */
extern Iterator * HASH_ConstKeys    P_((const HashTable * ht));
extern Iterator * HASH_ConstValues  P_((const HashTable * ht));
extern Iterator * HASH_ConstEntries P_((const HashTable * ht));

/* macros */
#define HASH_IsEmpty(_ht) (HASH_Size(_ht) == 0)
#define HASH_ContainsKey(_ht,_key) HASH_Contains(_ht,_key)

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_HASH_H_ */

/*
 * HISTORY:
 *
 * $Log: s_hash.h,v $
 * Revision 1.17  2010/12/19 17:56:39  slava
 * o added HASH_Update function
 *
 * Revision 1.16  2010/06/16 08:37:23  slava
 * o added HASH_Copy, HASH_ConstKeys, HASH_ConstValues and HASH_ConstEntries
 * o changed HASH_Examine prototype to take const HashTable pointer
 *
 * Revision 1.15  2006/09/22 19:19:08  slava
 * o changed the types of primeIndex and loadFactor fields from 'int'
 *   to 'short'. Given how these fields are used, even one byte would
 *   be enough but that might create alignment issues. 'short' is much
 *   safer and it still saves us 4 bytes on each instance of HashTable
 *
 * Revision 1.14  2004/03/17 20:58:31  slava
 * o added HASH_Reinit function
 * o renamed defaultHashCode into hashDefaultHashProc, defaultHashEquals into
 *   hashDefaultCompare and made them external
 *
 * Revision 1.13  2003/11/08 20:47:43  slava
 * o added iterators
 *
 * Revision 1.12  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.11  2002/08/12 05:13:27  slava
 * o use (void) rather than () to declare functions that have
 *   no arguments. The problem with () is that compilers interpret
 *   it as an *unspecified* argument list, but (void) clearly
 *   indicates that function takes no arguments. Improves compile
 *   time error checking
 *
 * Revision 1.10  2002/02/11 05:48:00  slava
 * o hash function no longer takes number of hash buckets in the hashtable
 *   as a parameter. hash code is a property of the object being used as
 *   a key in the hash table, and it does not need to know how big the
 *   hashtable(s) is(are). it would be quite strange if hash code depended
 *   on the size of the hash table. also, there's no more need to make
 *   any assumptions about the value being returned by the hash function.
 *   the fewer assumptions the better.
 *
 * Revision 1.9  2001/11/27 15:12:35  slava
 * o synchronize access to global data in the HASH module.
 * o added HASH_InitModule() function
 *
 * Revision 1.8  2001/06/29 04:13:17  slava
 * o allow HashCB callback (invoked by HASH_Examine) to remove the current
 *   element of the hashtable. Removing any other element is not supported
 *   and may cause a crash.
 *
 * Revision 1.7  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.6  2001/05/28 03:57:32  slava
 * o made load factor an integer because floating point may be a problem
 *
 * Revision 1.5  2001/04/16 12:48:47  slava
 * o added HASH_ContainsKey() function and HASH_IsEmpty() macro
 *
 * Revision 1.4  2001/02/22 04:26:29  slava
 * o cleanup comments
 *
 * Revision 1.3  2000/12/17 16:09:47  slava
 * o added extern "C" {} so that slib could be used from a C++ program
 *
 * Revision 1.2  2000/09/04 01:42:47  slava
 * o added stringCaseHashProc() - case-insensitive hash function for
 *   string keys
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
