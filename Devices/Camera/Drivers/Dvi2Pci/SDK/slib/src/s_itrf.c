/*
 * $Id: s_itrf.c,v 1.1 2006/03/20 02:49:47 slava Exp $
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

STATIC IElement ITR_FilterNext P_((Iterator * itr));
STATIC Bool ITR_FilterHasNext P_((Iterator * itr));
STATIC Bool ITR_FilterRemove P_((Iterator * itr));
STATIC void ITR_FilterDestroy P_((Iterator * itr));
STATIC void ITR_FilterFree P_((Iterator * itr));

typedef struct _FilterIterator {
    Iterator itr;           /* common part */
    Iterator * target;      /* the iterator being filtered */
    ItrFilterNext next;     /* next filter callback */
    ItrFilterRemove remove; /* remove filter callback */
    void * ctx;             /* context to pass to callbacks */
} FilterIterator;

STATIC const Itr filterIterator = {
    TEXT("Filter"),         /* name     */
    ITR_FilterHasNext,      /* hasNext  */
    ITR_FilterNext,         /* next     */
    ITR_FilterRemove,       /* remove   */
    ITR_FilterDestroy,      /* destroy  */
    ITR_FilterFree          /* free     */
};

/*==========================================================================*
 *              F I L T E R    I T E R A T O R
 *==========================================================================*/

/**
 * Creates the iterator that allows to filter the value returned by the
 * iterator and substitute it with something else. NULL callback means
 * no filtering, i.e. the default behavior provided by the target iterator.
 */
Iterator * ITR_Filter(Iterator * target, ItrFilterNext fnext, 
                      ItrFilterRemove fremove, void * ctx)
{
    ASSERT(target);
    if (target) {
        FilterIterator * fi = MEM_New(FilterIterator);
        if (fi) {
            ITR_Init(&fi->itr, &filterIterator);
            fi->target = target;
            fi->next = fnext;
            fi->remove = fremove;
            fi->ctx = ctx;
            return &fi->itr;
        }
    }
    return NULL;
}

STATIC Bool ITR_FilterHasNext(Iterator * itr)
{
    FilterIterator * fi = CAST(itr,FilterIterator,itr);
    return fi->target->type->hasNext(fi->target);
}

STATIC IElement ITR_FilterNext(Iterator * itr)
{
    FilterIterator * fi = CAST(itr,FilterIterator,itr);
    if (fi->next) {
        return fi->next(fi->target, fi->ctx);
    } else {
        return fi->target->type->next(fi->target);
    }
}

STATIC Bool ITR_FilterRemove(Iterator * itr)
{
    FilterIterator * fi = CAST(itr,FilterIterator,itr);
    if (fi->remove) {
        return fi->remove(fi->target, fi->ctx);
    } else {
        return fi->target->type->remove(fi->target);
    }
}

STATIC void ITR_FilterDestroy(Iterator * itr)
{
    FilterIterator * fi = CAST(itr,FilterIterator,itr);
    ITR_Delete(fi->target);
}

STATIC void ITR_FilterFree(Iterator * itr)
{
    FilterIterator * fi = CAST(itr,FilterIterator,itr);
    MEM_Free(fi);
}

/*
 * HISTORY:
 *
 * $Log: s_itrf.c,v $
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
