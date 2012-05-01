/*
 * $Id: s_itrs.c,v 1.2 2006/10/02 04:47:32 slava Exp $
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

STATIC IElement ITR_SingletonNext P_((Iterator * itr));
STATIC Bool ITR_SingletonHasNext P_((Iterator * itr));
STATIC void ITR_SingletonFree P_((Iterator * itr));

typedef struct _SingletonIterator {
    Iterator itr;           /* common part */
    IElement element;       /* the element */
    Bool hasNext;           /* False if we have returned our only element */
} SingletonIterator;

STATIC const Itr singletonIterator = {
    TEXT("Singleton"),      /* name     */
    ITR_SingletonHasNext,   /* hasNext  */
    ITR_SingletonNext,      /* next     */
    NULL,                   /* remove   */
    NULL,                   /* destroy  */
    ITR_SingletonFree       /* free     */
};

/*==========================================================================*
 *              S I N G L E T O N    I T E R A T O R
 *==========================================================================*/

/**
 * Creates the iterator containing one element. Does not support the remove
 * operation.
 */
Iterator * ITR_Singleton(IElementC element)
{
    SingletonIterator * si = MEM_New(SingletonIterator);
    if (si) {
        ITR_Init(&si->itr, &singletonIterator);
        si->element = (IElement)element;
        si->hasNext = True;
        return &si->itr;
    } else {
        return NULL;
    }
}

STATIC Bool ITR_SingletonHasNext(Iterator * itr)
{
    SingletonIterator * si = CAST(itr,SingletonIterator,itr);
    return si->hasNext;
}

STATIC IElement ITR_SingletonNext(Iterator * itr)
{
    SingletonIterator * si = CAST(itr,SingletonIterator,itr);
    IElement element = si->element;
    ASSERT(si->hasNext);
    si->hasNext = False;
    si->element = NULL;
    return element;
}

STATIC void ITR_SingletonFree(Iterator * itr)
{
    SingletonIterator * si = CAST(itr,SingletonIterator,itr);
    MEM_Free(si);
}

/*
 * HISTORY:
 *
 * $Log: s_itrs.c,v $
 * Revision 1.2  2006/10/02 04:47:32  slava
 * o ITR_Singleton now takes IElementC as a parameter
 *
 * Revision 1.1  2006/03/20 06:50:25  slava
 * o added singleton iterator to complete the picture
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
