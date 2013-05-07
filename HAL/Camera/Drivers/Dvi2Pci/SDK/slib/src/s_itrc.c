/*
 * $Id: s_itrc.c,v 1.3 2006/03/20 05:46:22 slava Exp $
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

#include "s_mem.h"
#include "s_itrp.h"

STATIC IElement ITR_CompositeNext P_((Iterator * itr));
STATIC Bool ITR_CompositeHasNext P_((Iterator * itr));
STATIC Bool ITR_CompositeRemove P_((Iterator * itr));
STATIC void ITR_CompositeDestroy P_((Iterator * itr));
STATIC void ITR_CompositeFree P_((Iterator * itr));

typedef struct _CompositeIterator {
    Iterator itr;           /* common part */
    int count;              /* number of child iterators */
    int current;            /* index of the current iterator */
    Iterator* iterators[1]; /* array of iterator */
} CompositeIterator;

STATIC const Itr compositeIterator = {
    TEXT("Composite"),      /* name     */
    ITR_CompositeHasNext,   /* hasNext  */
    ITR_CompositeNext,      /* next     */
    ITR_CompositeRemove,    /* remove   */
    ITR_CompositeDestroy,   /* destroy  */
    ITR_CompositeFree       /* free     */
};

/*==========================================================================*
 *              C O M P O S I T E    I T E R A T O R
 *==========================================================================*/

/**
 * Combines a number of iterators into one. Takes ownership of the child
 * iterators, i.e. destroys them after they have been used. Empty iterators
 * are destroyed immediately (only in case if this function succeeds).
 */
Iterator * ITR_Combine(Iterator * iterators[], int count)
{
    /* count non-empty iterators */
    int i, nonempty = 0;
    for (i=0; i<count; i++) {
        if (iterators[i] && ITR_HasNext(iterators[i])) {
            nonempty++;
        }
    }

    if (!nonempty) {
        /* special case - nothing interesting */
        for (i=0; i<count; i++) {
            ITR_Delete(iterators[i]);
        }
        return ITR_Empty();
    } else if (nonempty == 1) {
        /* only one iterator is not empty, nothing to combine */
        Iterator * itr = NULL;
        for (i=0; i<count; i++) {
            if (iterators[i]) {
                if (ITR_HasNext(iterators[i])) {
                    ASSERT(!itr);
                    itr = iterators[i];
                } else {
                    /* delete empty iterators */
                    ITR_Delete(iterators[i]);
                }
            }
        }
        ASSERT(itr);
        return itr;
    } else {
        /* we've got a real case here */
        int nbytes = sizeof(CompositeIterator)+sizeof(Iterator*)*(nonempty-1);
        CompositeIterator* ci = (CompositeIterator*)MEM_Alloc(nbytes);
        if (ci) {
            ITR_Init(&ci->itr, &compositeIterator);
            ci->count = 0;
            ci->current = 0;
            for (i=0; i<count; i++) {
                if (iterators[i]) {
                    if (ITR_HasNext(iterators[i])) {
                        ci->iterators[ci->count++] = iterators[i];
                    } else {
                        /* delete empty iterators */
                        ITR_Delete(iterators[i]);
                    }
                }
            }
            ASSERT(ci->count == nonempty);
            return &ci->itr;
        }
        return NULL;
    }
}

STATIC Bool ITR_CompositeHasNext(Iterator * itr)
{
    CompositeIterator * ci = CAST(itr,CompositeIterator,itr);
    while (ci->current < ci->count) {
        if (ITR_HasNext(ci->iterators[ci->current])) {
            return True;
        }
        ITR_Delete(ci->iterators[ci->current]);
        ci->iterators[ci->current++] = NULL;
    }
    return False;
}

STATIC IElement ITR_CompositeNext(Iterator * itr)
{
    CompositeIterator * ci = CAST(itr,CompositeIterator,itr);
    ASSERT(ci->current < ci->count);
    ASSERT(ci->iterators[ci->current]);
    return ITR_Next(ci->iterators[ci->current]);
}

STATIC Bool ITR_CompositeRemove(Iterator * itr)
{
    CompositeIterator * ci = CAST(itr,CompositeIterator,itr);
    ASSERT(ci->current < ci->count);
    ASSERT(ci->iterators[ci->current]);
    return ITR_Remove(ci->iterators[ci->current]);
}

STATIC void ITR_CompositeDestroy(Iterator * itr)
{
    CompositeIterator * ci = CAST(itr,CompositeIterator,itr);
    while (ci->current < ci->count) {
        ITR_Delete(ci->iterators[ci->current]);
        ci->iterators[ci->current++] = NULL;
    }
}

STATIC void ITR_CompositeFree(Iterator * itr)
{
    CompositeIterator * ci = CAST(itr,CompositeIterator,itr);
    MEM_Free(ci);
}

/*
 * HISTORY:
 *
 * $Log: s_itrc.c,v $
 * Revision 1.3  2006/03/20 05:46:22  slava
 * o delete unused empty iterators from the array passed to ITR_Combine. This
 *   hardly makes any difference because a) usually iterators are non empty;
 *   and b) those that are empty usually point to the static empty iterator,
 *   which is never really deleted (or even created, since it's static).
 *
 * Revision 1.2  2006/03/20 05:26:40  slava
 * o optimized composite iterator for the case of only one non-empty iterator
 *   in the list.
 *
 * Revision 1.1  2006/03/20 02:49:47  slava
 * o moved individual iterators into separate files so that they don't
 *   always get linked into each and every executable. Now they should
 *   only be linked if they are actually being used.
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
