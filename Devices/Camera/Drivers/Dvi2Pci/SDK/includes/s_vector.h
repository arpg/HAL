/*
 * $Id: s_vector.h,v 1.29 2006/11/20 18:49:41 slava Exp $
 *
 * Copyright (C) 2000-2006 by Slava Monich
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

#ifndef _SLAVA_VECTOR_H_
#define _SLAVA_VECTOR_H_

#include "s_def.h"
#include "s_itr.h"
#include "s_random.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* data types */
typedef struct _Vector Vector;
typedef void * VElement;
typedef const void * VElementC;

/* 
 * VectorCompare - compares its two arguments for order. Returns a negative 
 *                 integer, zero, or a positive integer as the first argument
 *                 is less than, equal to, or greater than the second. Used 
 *                 for sorting and searching.
 * VectorEquals  - returns TRUE if elements are equal, FALSE if they are not
 * VectorFree    - cleanup procedure invoked when element gets removed 
 *                 from the vector.
 */
typedef int  (*VectorCompare)P_((VElementC e1, VElementC e2));
typedef int  (*VectorCompare2)P_((VElementC e1, VElementC e2, void * ctx));
typedef Bool (*VectorEquals)P_((VElementC e1, VElementC e2));
typedef void (*VectorFree)P_((VElement e));

/*
 * Some useful VectorCompare functions
 *
 * vectorCompareString       - compares two strings, case-sensitive
 * vectorCompareStringNoCase - compares two strings, case-insensitive
 * vectorCompareDefault      - compares elements as numbers
 * vectorCompareDefault2      - compares elements as numbers
 */
extern int vectorCompareString P_((VElementC e1, VElementC e2));
extern int vectorCompareStringNoCase P_((VElementC e1, VElementC e2));
extern int vectorCompareDefault P_((VElementC e1, VElementC e2));
extern int vectorCompareDefault2 P_((VElementC e1, VElementC e2, void * ctx));

/*
 * Some useful VectorEquals functions
 *
 * vectorEqualsString       - compares two strings, case-sensitive
 * vectorEqualsStringNoCase - compares two strings, case-insensitive
 * vectorEqualsDefault      - compares VElements as numbers (default behavior)
 */
extern Bool vectorEqualsString P_((VElementC e1, VElementC e2));
extern Bool vectorEqualsStringNoCase P_((VElementC e1, VElementC e2));
extern Bool vectorEqualsDefault P_((VElementC e1, VElementC e2));

/*
 * Simple VectorFree callback functions.
 *
 * vectorFreeNothingProc - does not do anything
 * vectorFreeValueProc - deallocates the memory pointed to by VElement
 */
extern void vectorFreeNothingProc P_((VElement e));
extern void vectorFreeValueProc P_((VElement e));

/*
 * The vector itself. To not modify it directly.
 */
struct _Vector {
    int size;                   /* number of key-value pairs in the table */
    int alloc;                  /* number of allocated slots */
    VectorEquals equals;        /* compare function */
    VectorFree   free;          /* cleanup function */
    VElement*    data;          /* storage */
};

/* exported functions */
extern Vector * VECTOR_Create    P_((int n, VectorEquals c, VectorFree f));
extern Bool     VECTOR_Init      P_((Vector * v, int size, VectorEquals c, 
                                     VectorFree f));

extern void     VECTOR_Delete    P_((Vector * v));
extern void     VECTOR_Destroy   P_((Vector * v));
extern void     VECTOR_Clear     P_((Vector * v));
extern int      VECTOR_Size      P_((const Vector * v));
extern Bool     VECTOR_Equals    P_((const Vector * v, const Vector * v1));
extern void     VECTOR_Shuffle   P_((Vector * v, Random* r));
extern Bool     VECTOR_Add       P_((Vector * v, VElementC e));
extern Bool     VECTOR_TryAdd    P_((Vector * v, VElement e));
extern Bool     VECTOR_AddAll    P_((Vector * v, const Vector * v1));
extern Bool     VECTOR_InsertAll P_((Vector * v, int pos, const Vector * v1));
extern Bool     VECTOR_Insert    P_((Vector * v, int pos, VElementC e));
extern Bool     VECTOR_TryInsert P_((Vector * v, int pos, VElement e));
extern VElement VECTOR_Get       P_((const Vector * v, int pos));
extern void     VECTOR_Set       P_((Vector * v, int pos, VElementC e));
extern void     VECTOR_Swap      P_((Vector * v, int i1, int i2));
extern VElement VECTOR_Move      P_((Vector * v, int i1, int i2));
extern void     VECTOR_Fill      P_((Vector * v, VElementC e));
extern void     VECTOR_Trim      P_((Vector * v));
extern VElement VECTOR_Pop       P_((Vector * v));
extern VElement VECTOR_Remove    P_((Vector * v, int pos));
extern int      VECTOR_RemoveDups P_((Vector * v));
extern int      VECTOR_RemoveRange P_((Vector * v, int from, int to));
extern Bool     VECTOR_RemoveElement P_((Vector * v, VElement e));
extern Bool     VECTOR_Truncate  P_((Vector * v, int maxsiz));
extern int      VECTOR_RemoveAll P_((Vector * v, const Vector * v1));
extern void     VECTOR_RetainAll P_((Vector * v, const Vector * v1));
extern int      VECTOR_IndexOf   P_((const Vector * v, VElementC e));
extern Vector * VECTOR_SubVector P_((Vector * v, int from, int to));
extern Bool     VECTOR_Alloc     P_((Vector * v, int min));
extern Bool     VECTOR_ContainsAll P_((const Vector * v, const Vector * v1));
extern Bool     VECTOR_ContainsAny P_((const Vector * v, const Vector * v1));
extern int      VECTOR_LastIndexOf P_((const Vector * v, VElementC e));
extern Bool     VECTOR_CopyAll   P_((const Vector * v, Vector * v1));
extern int      VECTOR_Copy P_((const Vector * v,int i1,int i2,Vector * v1));
extern VElement * VECTOR_GetElements  P_((Vector * v));
extern VElement * VECTOR_ToArray  P_((const Vector * v));

/* iterators (the const iterator does not support remove operation) */
extern Iterator * VECTOR_Iterator       P_((Vector * v));
extern Iterator * VECTOR_ConstIterator  P_((const Vector * v));

/* searching and sorting */
extern Bool     VECTOR_Sort      P_((Vector * v, VectorCompare c));
extern Bool     VECTOR_SortPart  P_((Vector * v, 
                                    int from /*inclusive*/, 
                                    int to   /*exclusive*/,
                                    VectorCompare c));
extern int      VECTOR_Search    P_((const Vector * v, VElementC e, 
                                    VectorCompare c));

/* searching and sorting with context */
extern Bool     VECTOR_Sort2     P_((Vector * v,
                                     VectorCompare2 c, void * ctx));
extern Bool     VECTOR_SortPart2 P_((Vector * v, 
                                    int from /*inclusive*/, 
                                    int to   /*exclusive*/,
                                    VectorCompare2 c, void * ctx));
extern int      VECTOR_Search2   P_((const Vector * v, VElementC e, 
                                    VectorCompare2 c, void * ctx));

/* updating ordered vectors */
extern int VECTOR_UpdateOrder P_((Vector* v, int pos, VectorCompare c));
extern int VECTOR_InsertInOrder P_((Vector* v, VElementC e, VectorCompare c));

/* macros */
#define VECTOR_IsEmpty(v) (VECTOR_Size(v) == 0)
#define VECTOR_Contains(v,e) (VECTOR_IndexOf(v,e) >= 0)

/* legacy API */
#define VECTOR_EnsureCapacity(v,n) VECTOR_Alloc(v,n)

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_VECTOR_H_ */

/*
 * HISTORY:
 *
 * $Log: s_vector.h,v $
 * Revision 1.29  2006/11/20 18:49:41  slava
 * o added VECTOR_RemoveDups and VECTOR_RemoveRange functions
 *
 * Revision 1.28  2006/09/26 00:54:10  slava
 * o added VECTOR_Copy and VECTOR_CopyAll functions
 *
 * Revision 1.27  2006/09/24 13:11:11  slava
 * o added VECTOR_Pop and VECTOR_RemoveElement functions
 *
 * Revision 1.26  2006/03/16 23:56:29  slava
 * o added VECTOR_UpdateOrder and VECTOR_InsertInOrder functions
 *
 * Revision 1.25  2006/01/15 02:12:19  slava
 * o exported vectorCompareDefault, vectorCompareDefault2 and
 *   vectorEqualsDefault functions.
 * o renamed VECTOR_EnsureCapacity into VECTOR_Alloc for consistency with
 *   other slib modules. VECTOR_EnsureCapacity is now a macro that provides
 *   backward compatibility
 * o fixed the check for valid parameter range in VECTOR_Move. This doesn't
 *   affect well written code, but could cause a crash if one parameter was
 *   out of range.
 *
 * Revision 1.24  2005/01/03 03:33:36  slava
 * o VECTOR_Sort* functions now return a Bool value, True if the list has
 *   changed, False if it was alreay sorted.
 *
 * Revision 1.23  2005/01/02 16:14:00  slava
 * o changed some parameters of vector functions from VElement to VElementC
 *
 * Revision 1.22  2004/03/17 19:51:01  slava
 * o added VECTOR_ToArray
 *
 * Revision 1.21  2003/11/08 20:47:43  slava
 * o added iterators
 *
 * Revision 1.20  2003/09/19 23:30:43  slava
 * o added VECTOR_TryInsert, VECTOR_Search2 and VECTOR_Sort2
 *
 * Revision 1.19  2003/05/19 20:33:38  slava
 * o added vectorCompareString and vectorCompareStringNoCase
 *
 * Revision 1.18  2003/01/22 00:14:45  slava
 * o added VECTOR_TryAdd
 *
 * Revision 1.17  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.16  2002/05/29 05:16:04  slava
 * o added vectorEqualsString, vectorEqualsStringNoCase and
 *   VECTOR_ContainsAny functions
 *
 * Revision 1.15  2001/12/13 15:05:08  slava
 * added VECTOR_Move
 *
 * Revision 1.14  2001/12/10 09:47:24  slava
 * o added VECTOR_Truncate
 *
 * Revision 1.13  2001/12/06 13:16:00  slava
 * o VECTOR_Remove now returns the removed element
 *
 * Revision 1.12  2001/12/05 05:15:53  slava
 * o added VECTOR_Search() which implements binary search algorithm
 * o changed VectorCompare definition (removed context parameter)
 *
 * Revision 1.11  2001/10/08 05:17:52  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.10  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.9  2001/03/23 05:05:57  slava
 * o implemented VECTOR_RemoveAll() and VECTOR_RetainAll()
 *
 * Revision 1.8  2000/12/27 03:51:52  slava
 * o fixed VECTOR_Contains() macro. Its effect was totally opposite to
 *   what you would expect
 *
 * Revision 1.7  2000/12/26 15:45:22  slava
 * o added VECTOR_Shuffle
 *
 * Revision 1.6  2000/12/26 14:42:23  slava
 * o pass context parameter to VectorCompare function
 *
 * Revision 1.5  2000/12/26 13:50:13  slava
 * o new functions: VECTOR_Sort,VECTOR_SortPart,VECTOR_Swap and VECTOR_Fill
 * o fixed a bug in VECTOR_Delete which was not deallocating the buffer
 *
 * Revision 1.4  2000/12/26 12:35:57  slava
 * o use const modifier where appropriate
 * o renamed VectorCompare type into VectorEquals
 *
 * Revision 1.3  2000/12/17 16:09:47  slava
 * o added extern "C" {} so that slib could be used from a C++ program
 *
 * Revision 1.2  2000/08/27 14:06:28  slava
 * o minor cleanup
 *
 * Revision 1.1  2000/08/19 04:48:59  slava
 * o initial checkin
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
