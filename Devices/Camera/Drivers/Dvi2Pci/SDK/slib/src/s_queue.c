/*
 * $Id: s_queue.c,v 1.29 2008/12/17 12:02:03 slava Exp $
 *
 * Copyright (C) 2000-2008 by Slava Monich
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

#include "s_queue.h"
#include "s_itrp.h"
#include "s_mem.h"

/* Definition of the iterator */
STATIC IElement QUEUE_ItrNext P_((Iterator * itr));
STATIC Bool QUEUE_ItrHasNext P_((Iterator * itr));
STATIC Bool QUEUE_ItrRemove P_((Iterator * itr));
STATIC void QUEUE_ItrFree P_((Iterator * itr));

typedef struct _QueueIterator {
    Iterator itr;           /* common part */
    QEntry * entry;         /* the current entry */
    QEntry * next;          /* the next entry */
} QueueIterator;

STATIC const Itr queueIterator = {
    TEXT("Queue"),          /* name     */
    QUEUE_ItrHasNext,       /* hasNext  */
    QUEUE_ItrNext,          /* next     */
    QUEUE_ItrRemove,        /* remove   */
    NULL,                   /* destroy  */
    QUEUE_ItrFree           /* free     */
};

STATIC const Itr queueConstIterator = {
    TEXT("ConstQueue"),     /* name     */
    QUEUE_ItrHasNext,       /* hasNext  */
    QUEUE_ItrNext,          /* next     */
    NULL,                   /* remove   */
    NULL,                   /* destroy  */
    QUEUE_ItrFree           /* free     */
};

#if DEBUG

/* 
 * In debug build, we invalidate the entry when we remove it from the list,
 * so that it's more likely to crash if we screw up. In release build we 
 * are a litle bit more careful. 
 */
#define _InvalidateEntry(Entry) \
    (Entry)->prev = (Entry)->next = NULL;\
    (Entry)->queue = NULL;

/*
 * The QUEUE_CastEntry() and QUEUE_Cast functions do nothing but return their 
 * input parameter. The exist for the only purpose of controlling parameters
 * of CAST_ENTRY and CAST_QUEUE macros at compile time. In release build 
 * they are replaced with macros (i.e. no compile time check and no runtime
 * overhead)
 */
QEntry * QUEUE_CastEntry(const QEntry * e)
{ 
    ASSERT(e);
    return (QEntry*)e; 
}

Queue * QUEUE_Cast(const Queue * q)
{ 
    ASSERT(q);
    return (Queue*)q; 
}

#else  /* !DEBUG */

#define _InvalidateEntry(Entry) \
    (Entry)->prev = (Entry)->next = Entry;\
    (Entry)->queue = NULL;

#endif /* !DEBUG */

/* This macro removes an element from a double linked list */
#define _RemoveEntry(Entry) {\
    QEntry * _X_prev;\
    QEntry * _X_next;\
    _X_next = (Entry)->next;\
    _X_prev = (Entry)->prev;\
    _X_prev->next = _X_next;\
    _X_next->prev = _X_prev;\
    }

/* This macro inserts Entry between Prev and Next. 
   NOTE: it does not update the ->queue field. */
#define _InsertEntry(Prev,Entry,Next) {\
    (Entry)->next = Next;\
    (Entry)->prev = Prev;\
    Prev->next = (Entry);\
    Next->prev = (Entry);\
    }

/* This macro inserts specified element before the other one. 
   NOTE: it does not update the ->queue field. */
#define _InsertEntryBefore(Next,Entry) {\
    QEntry * _X_prev;\
    QEntry * _X_next;\
    _X_next = (Next);\
    _X_prev = _X_next->prev;\
    _InsertEntry(_X_prev,Entry,_X_next);\
    }

/* This macro inserts specified element after the other one. 
   NOTE: it does not update the ->queue field. */
#define _InsertEntryAfter(Prev,Entry) {\
    QEntry * _X_prev;\
    QEntry * _X_next;\
    _X_prev = (Prev);\
    _X_next = _X_prev->next;\
    _InsertEntry(_X_prev,Entry,_X_next);\
    }

/**
 * Initialize the queue.
 */
void QUEUE_Init(Queue * q)
{
    QEntry * head = &q->head;
    head->next = head->prev = head;
    head->queue = q;
    q->size = 0;
}

/**
 * Check if the queue is empty
 */
Bool QUEUE_IsEmpty(const Queue * q)
{
    const QEntry * head = &q->head;
    if (head->next == head) {
        ASSERT(head->prev == head);
        ASSERT(q->size == 0);
        return True;
    }
    return False;
}

/**
 * Tests if queue actually contains the specified entry and returns 
 * its index.
 *
 * For quick test you can use (e->queue == q) which is not a 100%
 * guarantee that entry is actually in the queue. This function scans 
 * the queue and makes sure that entry is there, but it's definitely 
 * slower that (e->queue == q).
 *
 * NOTE: this function does not do anything that would cause a core 
 * dump in case if e is an invalid pointer.
 */
int QUEUE_Find(const Queue * q, const QEntry * e)
{
    if (e) {
        int i;
        const QEntry * head = &q->head;
        const QEntry * entry = head->next;
        for (i=0; entry != head; i++, entry = entry->next) {
            if (entry == e) return i;
        }
    }
    return -1;
}

/**
 * Finds index of the specified entry in its queue, or -1 if the entry
 * is not queued. Unlike QUEUE_Find, this function would crash if entry 
 * pointer is invalid.
 */
int QUEUE_Index(const QEntry * e)
{
    if (e && e->queue) {
        int i = QUEUE_Find(e->queue, e);
        ASSERT(i != (-1));
        return i;
    }
    return -1;
}

/**
 * Insert new entry to the head of the queue.
 * Returns new size of the queue.
 */
int QUEUE_InsertHead(Queue * q, QEntry * e)
{
    return QUEUE_InsertAfter(&q->head, e);
}

/**
 * Insert new entry to the tail of the queue.
 * Returns new size of the queue.
 */
int QUEUE_InsertTail(Queue * q, QEntry * e)
{
    return QUEUE_InsertBefore(&q->head, e);
}

/**
 * Insert queue entry before the other one.
 * Returns new size of the queue.
 */
int QUEUE_InsertBefore(QEntry * next, QEntry * e)
{
    _InsertEntryBefore(next,e);
    return ++((e->queue = next->queue)->size);
}

/**
 * Insert queue entry after the other one.
 * Returns new size of the queue.
 */
int QUEUE_InsertAfter(QEntry * prev, QEntry * e)
{
    _InsertEntryAfter(prev,e);
    return ++((e->queue = prev->queue)->size);
}

/**
 * Returns the first queue entry or NULL if queue is empty
 */
QEntry * QUEUE_First(Queue * q)
{
    if (!QUEUE_IsEmpty(q)) {
        return q->head.next;
    }
    return NULL;
}

/**
 * Returns the last queue entry or NULL if queue is empty
 */
QEntry * QUEUE_Last(Queue * q)
{
    if (!QUEUE_IsEmpty(q)) {
        return q->head.prev;
    }
    return NULL;
}

/**
 * Returns the next entry in the queue or NULL if this is the last entry
 * or is not in any queue. The entry must not be the list head.
 */
QEntry * QUEUE_Next(QEntry * e)
{
    if (e->queue) {
        QEntry * head = &e->queue->head;
        ASSERT(e != head);
        if (e->next != head) {
            return e->next;
        }
    }
    return NULL;
}

/**
 * Returns the previous entry in the queue or NULL if this is the first entry
 * or is not in any queue. The entry must not be the list head.
 */
QEntry * QUEUE_Prev(QEntry * e)
{
    if (e->queue) {
        QEntry * head = &e->queue->head;
        ASSERT(e != head);
        if (e->prev != head) {
            return e->prev;
        }
    }
    return NULL;
}

/**
 * Returns the entry at specified index from the head of the queue.
 * This method is very inefficient as the search time grows linearly
 * with the index.
 */
QEntry * QUEUE_Get(Queue * q, int pos)
{
    ASSERT(q);
    if (q) {
        ASSERT(pos >= 0 && pos < q->size);
        if (pos >= 0 && pos < q->size) {

            int i;
            QEntry * e;
            QEntry * head = &q->head;

            /* take the shortest path */
            if (pos < q->size/2) {
                for (i = 0, e = head->next; e != head; e = e->next) {
                    if (i++ == pos) {
                        return e;
                    }
                }
            } else {
                for (i = q->size, e = head->prev; e != head; e = e->prev) {
                    if (--i == pos) {
                        return e;
                    }
                }
            }

            /* NOT REACHED */
            ASSERT(False);
        }
    }
    return NULL;
}

/**
 * Removes the first entry from the queue.
 * Returns NULL if queue is empty
 */
QEntry * QUEUE_RemoveHead(Queue * q)
{
    QEntry * e = QUEUE_First(q);
    if (e) {
        _RemoveEntry(e);
        _InvalidateEntry(e);
        q->size--;
    }
    return e;
}

/**
 * Removes the last entry from the queue.
 * Returns NULL if queue is empty
 */
QEntry * QUEUE_RemoveTail(Queue * q)
{
    QEntry * e = QUEUE_Last(q);
    if (e) {
        _RemoveEntry(e);
        _InvalidateEntry(e);
        q->size--;
    }
    return e;
}

/**
 * Removes the specified entry from its queue.
 *
 * Returns True if the entry has been removed from the queue, 
 *         False if it wasn't queued
 */
Bool QUEUE_RemoveEntry(QEntry * e)
{
    Queue * q = e->queue;
    if (q) {
        q->size--;
        ASSERT(q->size >= 0);
        ASSERT(q->size == 0 || (q->head.next != (&q->head)));
        _RemoveEntry(e);
        _InvalidateEntry(e);
        return True;
    }
    return False;
}

/**
 * Removes all entries from the list. It's more efficient than QUEUE_First/
 * QUEUE_Next/QUEUE_RemoveEntry loop or QUEUE_Examine call. Returns number
 * of removed entries (e.g. list size before this call).
 */
int QUEUE_Clear(Queue * q)
{
    int n = q->size;
    if (n > 0) {
        QEntry * head = &q->head;
        QEntry * e = head->next;
        ASSERT(e != head);
        while (e != head) {
            // Note that there's no need to do _RemoveEntry(e) because
            // we are going to remove and invalidate all entries
            QEntry * next = e->next;
            _InvalidateEntry(e);
            q->size--;
            e = next;
        }
        head->next = head->prev = head;
        ASSERT(q->size == 0);
    }
    return n;
}

/**
 * Moves all the entries from the source queue to the tail of the destination
 * queue. DOES NOT remove the entries that are already in the destination 
 * queue. Returns number of moved entries.
 */
int QUEUE_Move(Queue * dest, Queue * src)
{
    int n = 0;
    if (src && src != dest && QUEUE_Size(src) > 0) {
        QEntry * e;
        while ((e = QUEUE_RemoveHead(src)) != NULL) {
            QUEUE_InsertTail(dest, e);
            n++;
        }
        ASSERT(QUEUE_Size(src) == 0);
    }
    return n;
}

/**
 * Move entry closer to the head of the queue by specified number of positions.
 * If entry is already at the head of the queue, it stays there, i.e. there's
 * no rollover. Number of positions must not be negative, zero has no effect.
 *
 * Returns actual number of positions.
 */
int QUEUE_MoveToHead(QEntry * e, int npos)
{
    int n = 0;
    Queue * q = e->queue;
    if (q) {
        QEntry * head = &q->head;
        QEntry * prev = e->prev;
        ASSERT(npos >= 0);
        if (prev != head && npos > 0) {
            int i;
            _RemoveEntry(e);
            for (i=0; i<npos && prev != head; i++) {
                prev = prev->prev;
                n++;
            }
            _InsertEntryAfter(prev,e);
        }
    }
    return n;
}

/**
 * Move entry closer to the tail of the queue by specified number of positions.
 * If entry is already at the tail of the queue, it stays there, i.e. there's
 * no rollover. Number of positions must not be negative, zero has no effect.
 *
 * Returns actual number of positions.
 */
int QUEUE_MoveToTail(QEntry * e, int npos)
{
    int n = 0;
    Queue * q = e->queue;
    if (q) {
        QEntry * head = &q->head;
        QEntry * next = e->next;
        ASSERT(npos >= 0);
        if (next != head && npos > 0) {
            int i;
            _RemoveEntry(e);
            for (i=0; i<npos && next != head; i++) {
                next = next->next;
                n++;
            }
            _InsertEntryBefore(next,e);        
        }
    }
    return n;
}

/*
 * examine the queue, calling callback function on each entry. Stops
 * when either all entries get examined, or callback function returns
 * False. Returns the last value returned by the callback function, or
 * True if queue is empty. QUEUE_Examine() walks the queue from head 
 * to tail.
 */
Bool QUEUE_Examine(Queue * q, QueueCB cb, void * param)
{
    QEntry * head = &q->head;
    QEntry * e = head->next;
    while (e != head) {
        QEntry * next = e->next;
        if (!(*cb)(e, param)) return False;
        e = next;
    }
    return True;
}

/*
 * examine the queue, calling callback function on each entry. Stops
 * when either all entries get examined, or callback function returns
 * False. Returns the last value returned by the callback function, or
 * True if queue is empty. QUEUE_Examine() walks the queue from head 
 * to tail.
 */
Bool QUEUE_Examine2(Queue * q, QueueCB2 cb, void * param1, void * param2)
{
    QEntry * head = &q->head;
    QEntry * e = head->next;
    while (e != head) {
        QEntry * next = e->next;
        if (!(*cb)(e, param1, param2)) return False;
        e = next;
    }
    return True;
}

/*
 * examine the queue, calling callback function on each entry. Stops
 * when either all entries get examined, or callback function returns
 * False. Returns the last value returned by the callback function, or
 * True if queue is empty. Walks the queue from tail to head.
 */
Bool QUEUE_ExamineBack(Queue * q, QueueCB cb, void * ctx) 
{
    QEntry * head = &q->head;
    QEntry * e = head->prev;
    while (e != head) {
        QEntry * prev = e->prev;
        if (!(*cb)(e,ctx)) return False;
        e = prev;
    }
    return True;
}

/*
 * examine the queue, calling callback function on each entry. Stops
 * when either all entries get examined, or callback function returns
 * False. Returns the last value returned by the callback function, or
 * True if queue is empty. Walks the queue from tail to head.
 */
Bool QUEUE_ExamineBack2(Queue * q, QueueCB2 cb, void * param1, void * param2)
{
    QEntry * head = &q->head;
    QEntry * e = head->prev;
    while (e != head) {
        QEntry * prev = e->prev;
        if (!(*cb)(e, param1, param2)) return False;
        e = prev;
    }
    return True;
}

/*==========================================================================*
 *              F U L L    I T E R A T O R
 *==========================================================================*/

/**
 * Creates the iterator
 */
Iterator * QUEUE_Iterator(Queue * q)
{
    if (QUEUE_IsEmpty(q)) {
        return ITR_Empty();
    } else {
        QueueIterator * qi = MEM_New(QueueIterator);
        if (qi) {
            ITR_Init(&qi->itr, &queueIterator);
            qi->entry = NULL;
            qi->next = QUEUE_First(q);
            return &qi->itr;
        } else {
            return NULL;
        }
    }
}

/**
 * Creates the read-only iterator which doesn't support remove operation
 */
Iterator * QUEUE_ConstIterator(const Queue * q)
{
    if (QUEUE_IsEmpty(q)) {
        return ITR_Empty();
    } else {
        QueueIterator * qi = MEM_New(QueueIterator);
        if (qi) {
            ITR_Init(&qi->itr, &queueConstIterator);
            qi->entry = NULL;
            qi->next = q->head.next;
            return &qi->itr;
        } else {
            return NULL;
        }
    }
}

STATIC Bool QUEUE_ItrHasNext(Iterator * itr)
{
    QueueIterator * qi = CAST(itr,QueueIterator,itr);
    return BoolValue(qi->next != NULL);
}

STATIC IElement QUEUE_ItrNext(Iterator * itr)
{
    QueueIterator * qi = CAST(itr,QueueIterator,itr);
    qi->entry = qi->next;
    qi->next = QUEUE_Next(qi->entry);
    return qi->entry;
}

STATIC Bool QUEUE_ItrRemove(Iterator * itr)
{
    Bool ok = False;
    QueueIterator * qi = CAST(itr,QueueIterator,itr);
    if (qi->entry) {    
        ok = QUEUE_RemoveEntry(qi->entry);
        qi->entry = NULL;
    }
    return ok;
}

STATIC void QUEUE_ItrFree(Iterator * itr)
{
    QueueIterator * qi = CAST(itr,QueueIterator,itr);
    MEM_Free(qi);
}

/*
 * HISTORY:
 *
 * $Log: s_queue.c,v $
 * Revision 1.29  2008/12/17 12:02:03  slava
 * o added QUEUE_Examine2 and QUEUE_ExamineBack2 functions which take two
 *   context parameters rather than one
 *
 * Revision 1.28  2006/03/20 02:28:02  slava
 * o added QUEUE_ConstIterator function which creates iterator that doesn't
 *   support remove operation.
 *
 * Revision 1.27  2006/03/17 22:16:31  slava
 * o renamed QUEUE_RemoveAll into QUEUE_Clear for consistency with other
 *   collections (especially, the Vector object which already has RemoveAll
 *   function that does something entirely different)
 *
 * Revision 1.26  2006/03/17 21:42:51  slava
 * o added QUEUE_RemoveAll function
 *
 * Revision 1.25  2005/08/02 19:12:54  slava
 * o fixed a bug in queue iterator
 *
 * Revision 1.24  2005/07/10 21:44:51  slava
 * o don't allocate iterators for empty collections; return static
 *   empty iterator instead
 *
 * Revision 1.23  2004/04/08 01:44:17  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.22  2004/04/05 11:00:30  slava
 * o fixed Unicode build
 *
 * Revision 1.21  2003/11/08 20:47:43  slava
 * o added iterators
 *
 * Revision 1.20  2002/08/27 11:52:03  slava
 * o some reformatting
 *
 * Revision 1.19  2002/01/02 06:09:40  slava
 * o forgot to rename QUEUE_GetEntry into QUEUE_Get, got "unresolved
 *   external" from linker
 *
 * Revision 1.18  2001/12/31 09:25:49  slava
 * o added QUEUE_Find, QUEUE_Index, QUEUE_Next and QUEUE_Prev
 * o QUEUE_Contains is now a macro invoking QUEUE_Find
 * o renamed QUEUE_GetFirst, QUEUE_GetLast and QUEUE_GetEntry into
 *   QUEUE_First, QUEUE_Last and QUEUE_Get, respectively
 *
 * Revision 1.17  2001/12/28 15:22:58  slava
 * o renamed MustBeQEntry and MustBeQueue functions/macros into
 *   QUEUE_CastEntry and QUEUE_Cast, respectively
 *
 * Revision 1.16  2001/10/17 07:37:14  slava
 * o added QUEUE_Move function
 *
 * Revision 1.15  2001/10/08 05:17:52  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.14  2001/10/06 02:41:39  slava
 * o fixed comments
 *
 * Revision 1.13  2001/10/06 02:29:10  slava
 * o removed QUEUE_IsQueueHead() function
 * o QUEUE_Contains shouldn't do anything that would cause a core dump
 *   if given an invalid entry pointer
 *
 * Revision 1.12  2001/09/25 04:10:33  slava
 * o another use for MustBeQEntry() and MustBeQueue() macros - remove
 *   const modifier
 *
 * Revision 1.11  2001/09/16 16:48:55  slava
 * o added QUEUE_Contains() function
 *
 * Revision 1.10  2001/07/26 05:55:09  slava
 * o added couple ASSERTs for debug build
 *
 * Revision 1.9  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.8  2001/05/18 22:36:57  slava
 * o THIS_FILE variable is no longer being used
 *
 * Revision 1.7  2001/01/07 06:19:51  slava
 * o changed QUEUE_Examine() and QUEUE_ExamineBack() so that they work
 *   correctly in case if callback function removes the current entry
 *   from the list
 *
 * Revision 1.6  2000/12/27 03:49:53  slava
 * o fixed comments
 *
 * Revision 1.5  2000/11/01 13:25:05  slava
 * o replaced TRUE/FALSE with True/False
 *
 * Revision 1.4  2000/08/24 01:40:35  slava
 * o added QUEUE_GetFirst() and QUEUE_GetLast() fuctions
 *
 * Revision 1.3  2000/08/23 19:57:13  slava
 * o optimized QUEUE_GetEntry to start from the tail of the queue if index
 *   is in the second half, i.e. take the shortest path
 *
 * Revision 1.2  2000/08/23 18:52:11  slava
 * o added QUEUE_GetEntry() function
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
