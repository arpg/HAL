/*
 * $Id: s_wkq.h,v 1.10 2009/10/02 20:25:43 slava Exp $
 *
 * Copyright (C) 2000-2009 by Slava Monich
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

#ifndef _SLAVA_WORK_QUEUE_H_
#define _SLAVA_WORK_QUEUE_H_

#include "s_def.h"
#include "s_event.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/*
 * This module implements a "work queue" - a dedicated thread invoking
 * "work items" in the order they were submitted. Individual work items
 * can be created, queued (more than once), cancelled, destroyed, etc.
 */

/* opaque structures */
typedef struct _WorkQueue WorkQueue;
typedef struct _WorkItem  WorkItem;

/* the work item callback function */
typedef void (*WorkProc) P_((WorkItem * w, void*));
typedef void (*WorkProc2) P_((WorkItem * w, void*, void*));
typedef void (*IdleProc) P_((WorkQueue * q, void*));

/* init/shutdown */
extern void WKQ_InitModule P_((void));
extern void WKQ_Shutdown P_((void));

/* create objects */
extern WorkQueue * WKQ_CreatePool P_((int nthreads));
extern WorkQueue * WKQ_Create P_((void)); /* single work thread */
extern WorkItem *  WKI_Create P_((WorkQueue * q, WorkProc cb, void * par));

/* operations on work queue */
extern Bool WKQ_InvokeLater P_((WorkQueue * q, WorkProc cb, void * par));
extern Bool WKQ_InvokeAndWait P_((WorkQueue * q, WorkProc cb, void * par));
extern void WKQ_SetIdle P_((WorkQueue * q, long ms, IdleProc cb, void * par));
extern void WKQ_Cancel P_((WorkQueue * q));
extern void WKQ_Stop P_((WorkQueue * q, Bool canWait));
extern void WKQ_Delete P_((WorkQueue * q));

/* wait for work queue to stop */
extern void WKQ_Wait P_((WorkQueue * q));
extern WaitState WKQ_TimeWait P_((WorkQueue * q, long ms));

/* work items can be associated with 2 parameters */
extern WorkItem* WKI_Create2 P_((WorkQueue* q,WorkProc2 cb,void* p1,void* p2));
extern Bool WKQ_InvokeLater2 P_((WorkQueue* q,WorkProc2 cb,void* p1,void* p2));
extern Bool WKQ_InvokeAndWait2 P_((WorkQueue*q,WorkProc2 cb,void*p1,void* p2));

/*
 * A few words on the Detach operation. After WKI_Detach() the caller 
 * no longer owns the WorkItem pointer. The work queue will deallocate
 * it as soon is has been dequeued. If the work item is not currently 
 * queued, it will be destroyed immediately. Calling WKI_Detach() more
 * than once with the same parameter is a no-no
 */
extern Bool WKI_Submit P_((WorkItem * w));
extern Bool WKI_Cancel P_((WorkItem * w));
extern void WKI_Detach P_((WorkItem * w));
extern Bool WKI_IsDone P_((WorkItem * w));
extern Bool WKI_IsCanceled P_((WorkItem * w));

extern void WKI_Wait P_((WorkItem * w));
extern WaitState WKI_TimeWait P_((WorkItem * w, long ms));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_WORK_QUEUE_H_ */

/*
 * HISTORY:
 *
 * $Log: s_wkq.h,v $
 * Revision 1.10  2009/10/02 20:25:43  slava
 * o added WKI_Create2, WKQ_InvokeLater2 and WKQ_InvokeAndWait2 functions
 *   that create work items with two context parameters which it sometimes
 *   very convenient.
 *
 * Revision 1.9  2009/06/30 11:16:40  slava
 * o extended WorkQueue functionality so that it can use multuiple threads
 *   to run the work items.
 *
 * Revision 1.8  2004/11/27 17:41:24  slava
 * o added WKQ_SetIdle function (sets idle callback)
 *
 * Revision 1.7  2003/06/21 18:26:27  slava
 * o fixed compilation warning under HP-UX
 *
 * Revision 1.6  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.5  2002/08/29 04:50:09  slava
 * o added WKQ_Cancel (cancels all pending work items)
 *
 * Revision 1.4  2002/08/29 03:14:39  slava
 * o WKQ_Destroy has been renamed into WKQ_Delete for consistency with other
 *   modules
 *
 * Revision 1.3  2002/08/12 05:13:27  slava
 * o use (void) rather than () to declare functions that have
 *   no arguments. The problem with () is that compilers interpret
 *   it as an *unspecified* argument list, but (void) clearly
 *   indicates that function takes no arguments. Improves compile
 *   time error checking
 *
 * Revision 1.2  2001/12/18 05:49:50  slava
 * o WorkProc takes WorkItem * as another parameter. This allows a WorkProc
 *   function to determine that its work item has been canceled
 *
 * Revision 1.1  2001/11/27 06:06:58  slava
 * o added "work queue" module
 *
 * Local Variables:
 * mode:C
 * c-basic-offset:4
 * indent-tabs-mode: nil
 * End:
 */
