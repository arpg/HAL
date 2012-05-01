/*
 * $Id: s_queue.h,v 1.24 2008/12/17 12:02:03 slava Exp $
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

#ifndef _SLAVA_QUEUE_H_
#define _SLAVA_QUEUE_H_

#include "s_itr.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* 
 * Doubly linked queue (list) entry
 */
typedef struct _Queue Queue;
typedef struct _QEntry QEntry;
struct _QEntry {
    QEntry * next;
    QEntry * prev;
    Queue *  queue;
};

/* 
 * Doubly linked queue (list)
 */
struct _Queue {
    int size;
    QEntry head;
};

/*
 * The QUEUE_Cast() and QUEUE_CastEntry() functions do nothing but return 
 * their input parameter. They exist for the single purpose of checking 
 * the type of the first parameter of the CAST_ENTRY and CAST_QUEUE 
 * macros (typical error - passing in QEntry** rather than QEntry*). 
 * In the release build they are replaced with macros (i.e. no compile 
 * time check of the pointer type and on runtime overhead). Another 
 * use for those macros - getting rid of 'const' modifier.
 */
#if DEBUG
extern QEntry * QUEUE_CastEntry P_((const QEntry * e));
extern Queue *  QUEUE_Cast P_((const Queue * q));
#else  /* !DEBUG */
#  define QUEUE_CastEntry(_e) ((QEntry*)(_e))
#  define QUEUE_Cast(_q)  ((Queue*)(_q))
#endif /* !DEBUG */

/* 
 * Calculate the address of the base of the structure given its type, and an
 * address of a QEntry field within the structure. 
 */
#define CAST_ENTRY(_a,_t,_f) CAST(QUEUE_CastEntry(_a),_t,_f)
#define QCAST(_a,_t,_f) CAST_ENTRY(_a,_t,_f)

/* 
 * Calculate the address of the base of the structure given its type, and an
 * address of a Queue field within the structure. 
 */
#define CAST_QUEUE(_a,_t,_f) CAST(QUEUE_Cast(_a),_t,_f)

/*
 * callback for examining the queue. 
 *
 * e - current queue entry.
 * ctx - context parameter provided by user.
 *
 * Returns True to continue, False to stop examining the queue.
 * The QueueCB callback function is allowed to remove the current 
 * entry from the queue and continue enumeration, but if it deletes 
 * other entries, it MUST return False to terminate the process of 
 * enumeration.
 */
typedef Bool (*QueueCB)  P_((QEntry * e, void * param));
typedef Bool (*QueueCB2) P_((QEntry * e, void * param1, void * param2));

/* functions to manipulate queues */
extern void     QUEUE_Init         P_((Queue * q));
extern Bool     QUEUE_IsEmpty      P_((const Queue * q));
extern int      QUEUE_Find         P_((const Queue * q, const QEntry * e));
extern int      QUEUE_Index        P_((const QEntry * e));
extern int      QUEUE_InsertHead   P_((Queue * q, QEntry * e));
extern int      QUEUE_InsertTail   P_((Queue * q, QEntry * e));
extern int      QUEUE_InsertBefore P_((QEntry * next, QEntry * e));
extern int      QUEUE_InsertAfter  P_((QEntry * prev, QEntry * e));
extern int      QUEUE_Clear        P_((Queue * q));
extern Bool     QUEUE_RemoveEntry  P_((QEntry * e));
extern QEntry * QUEUE_RemoveHead   P_((Queue * q));
extern QEntry * QUEUE_RemoveTail   P_((Queue * q));
extern QEntry * QUEUE_First        P_((Queue * q));
extern QEntry * QUEUE_Last         P_((Queue * q));
extern QEntry * QUEUE_Next         P_((QEntry * e));
extern QEntry * QUEUE_Prev         P_((QEntry * e));
extern QEntry * QUEUE_Get          P_((Queue * q, int pos));
extern int      QUEUE_Move         P_((Queue * dest, Queue * src));
extern int      QUEUE_MoveToHead   P_((QEntry * e, int npos));
extern int      QUEUE_MoveToTail   P_((QEntry * e, int npos));

/* function-like macros */
#define QUEUE_Size(_q)          (QUEUE_Cast(_q)->size)
#define QUEUE_Contains(_q,_e)   (QUEUE_Find(_q,_e) != (-1))

/*
 * examine the queue, calling callback function on each entry. Stops
 * when either all entries get examined, or callback function returns
 * False. Returns the last value returned by the callback function, or
 * True if queue is empty. QUEUE_Examine() walks the queue from head 
 * to tail, QUEUE_ExamineBack() - from tail to head.
 *
 * NOTE: the callback is allowed to delete the current entry from the 
 * list and continue enumeration, but if it deletes other entries, it 
 * MUST return False to terminate the process of enumeration.
 */
extern Bool QUEUE_Examine     P_((Queue * q, QueueCB cb, void * param));
extern Bool QUEUE_ExamineBack P_((Queue * q, QueueCB cb, void * param));
extern Bool QUEUE_Examine2     P_((Queue* q, QueueCB2 cb, void* p1,void* p2));
extern Bool QUEUE_ExamineBack2 P_((Queue* q, QueueCB2 cb, void* p1,void* p2));

/* iterator, returns pointers to QEntry */
extern Iterator * QUEUE_Iterator P_((Queue * q));
extern Iterator * QUEUE_ConstIterator P_((const Queue * q));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_QUEUE_H_ */

/*
 * HISTORY:
 *
 * $Log: s_queue.h,v $
 * Revision 1.24  2008/12/17 12:02:03  slava
 * o added QUEUE_Examine2 and QUEUE_ExamineBack2 functions which take two
 *   context parameters rather than one
 *
 * Revision 1.23  2006/03/20 02:28:02  slava
 * o added QUEUE_ConstIterator function which creates iterator that doesn't
 *   support remove operation.
 *
 * Revision 1.22  2006/03/17 22:16:31  slava
 * o renamed QUEUE_RemoveAll into QUEUE_Clear for consistency with other
 *   collections (especially, the Vector object which already has RemoveAll
 *   function that does something entirely different)
 *
 * Revision 1.21  2006/03/17 21:42:51  slava
 * o added QUEUE_RemoveAll function
 *
 * Revision 1.20  2003/11/08 20:47:43  slava
 * o added iterators
 *
 * Revision 1.19  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.18  2002/06/30 16:18:56  slava
 * o cleaned up comments
 *
 * Revision 1.17  2001/12/31 09:25:49  slava
 * o added QUEUE_Find, QUEUE_Index, QUEUE_Next and QUEUE_Prev
 * o QUEUE_Contains is now a macro invoking QUEUE_Find
 * o renamed QUEUE_GetFirst, QUEUE_GetLast and QUEUE_GetEntry into
 *   QUEUE_First, QUEUE_Last and QUEUE_Get, respectively
 *
 * Revision 1.16  2001/12/28 15:22:58  slava
 * o renamed MustBeQEntry and MustBeQueue functions/macros into
 *   QUEUE_CastEntry and QUEUE_Cast, respectively
 *
 * Revision 1.15  2001/10/17 07:37:14  slava
 * o added QUEUE_Move function
 *
 * Revision 1.14  2001/10/08 05:17:52  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.13  2001/10/06 02:26:32  slava
 * o removed QUEUE_IsQueueHead() function
 *
 * Revision 1.12  2001/09/25 04:10:33  slava
 * o another use for MustBeQEntry() and MustBeQueue() macros - remove
 *   const modifier
 *
 * Revision 1.11  2001/09/16 16:48:55  slava
 * o added QUEUE_Contains() function
 *
 * Revision 1.10  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.9  2001/01/28 09:46:50  slava
 * o updated comments
 *
 * Revision 1.8  2001/01/07 06:19:51  slava
 * o changed QUEUE_Examine() and QUEUE_ExamineBack() so that they work
 *   correctly in case if callback function removes the current entry
 *   from the list
 *
 * Revision 1.7  2001/01/03 06:57:58  slava
 * o moved CAST macro from s_queue.h to s_def.h
 *
 * Revision 1.6  2000/12/27 03:49:53  slava
 * o fixed comments
 *
 * Revision 1.5  2000/12/17 16:09:47  slava
 * o added extern "C" {} so that slib could be used from a C++ program
 *
 * Revision 1.4  2000/11/09 21:11:49  slava
 * o made QUEUE_Size() macro type-safe in debug build
 *
 * Revision 1.3  2000/08/24 01:40:35  slava
 * o added QUEUE_GetFirst() and QUEUE_GetLast() fuctions
 *
 * Revision 1.2  2000/08/23 18:52:11  slava
 * o added QUEUE_GetEntry() function
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
