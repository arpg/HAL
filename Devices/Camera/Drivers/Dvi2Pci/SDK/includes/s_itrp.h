/*
 * $Id: s_itrp.h,v 1.2 2006/03/19 09:35:21 slava Exp $
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

#ifndef _SLAVA_ITERATOR_PRIVATE_H_
#define _SLAVA_ITERATOR_PRIVATE_H_

#include "s_itr.h"

/* 
 * ItrNext    - returns the next element of the collection, NULL if none
 * ItrHasNext - tests if at least one more element is available
 * ItrRemove  - removes the current element
 * ItrDestroy - destroys the iterator context
 * ItrFree    - deallocates memory dynamically allocated for the iterator
 *
 * ItrRemove callback is guaranteed to be called no more than once per
 * each object. The implementation does not have to check that. Also, it's
 * guaranteed that ItrNext is invoked if and only if ItrHasNext returns
 * True.
 */
typedef IElement (*ItrNext)    P_((Iterator * itr));
typedef Bool     (*ItrHasNext) P_((Iterator * itr));
typedef Bool     (*ItrRemove)  P_((Iterator * itr));
typedef void     (*ItrDestroy) P_((Iterator * itr));
typedef void     (*ItrFree)    P_((Iterator * itr));

/* data structures */
typedef struct _Itr {       /* iterator type */
    Str         name;       /* name of the iterator for debugging purposes */
    ItrHasNext  hasNext;    /* checks if more elements are available */
    ItrNext     next;       /* returns next element of the collection */
    ItrRemove   remove;     /* removes current element from the collection */
    ItrDestroy  destroy;    /* destroys the iterator data (optional) */
    ItrFree     free;       /* deallocates the iterator context */
} Itr;

struct _Iterator {          /* common part of the iterator context */
    const Itr * type;       /* points to the iterator callback functions */
    IElement last;          /* last value returned by ITR_Next */
    int flags;              /* flags, see below */

#define ITR_ACTIVE  0x01    /* at least one value has been returned */
#define ITR_REMOVED 0x02    /* remove was called on the current element */
};

/* Initializes the common part */
extern void ITR_Init P_((Iterator * itr, const Itr * type));

#endif /* _SLAVA_ITERATOR_PRIVATE_H_ */

/*
 * HISTORY:
 *
 * $Log: s_itrp.h,v $
 * Revision 1.2  2006/03/19 09:35:21  slava
 * o store the last value returned by ITR_Next, needed by ITR_Last
 *
 * Revision 1.1  2003/11/08 20:47:43  slava
 * o added iterators
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
