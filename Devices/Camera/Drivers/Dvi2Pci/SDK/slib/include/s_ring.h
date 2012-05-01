/*
 * $Id: s_ring.h,v 1.2 2009/10/15 12:58:39 slava Exp $
 *
 * Copyright (C) 2008-2009 by Slava Monich
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

#ifndef _SLAVA_RING_H_
#define _SLAVA_RING_H_

#include "s_def.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* data types */
typedef struct _Ring Ring;
typedef void * RElement;

/*
 * a ring buffer.
 */
struct _Ring {
    int alloc;                  /* allocated buffer space */
    int maxsiz;                 /* max entries to store, -1 if no limit */
    int start;                  /* start offset (inclusive) */
    int end;                    /* end offset (exclusive) */
    RElement* data;             /* storage */
};

/* operations on ring buffer */
extern Ring * RING_Create P_((void));
extern Ring * RING_Create2 P_((int init, int max));
extern void RING_Delete P_((Ring * r));
extern void RING_Init P_((Ring * r));
extern Bool RING_Init2 P_((Ring * r, int init, int max));
extern void RING_Destroy P_((Ring * r));
extern void RING_Clear P_((Ring * r));
extern int  RING_Size P_((const Ring * r));
extern Bool RING_IsEmpty P_((const Ring * r));
extern void RING_Compact P_((Ring * r));
extern Bool RING_EnsureCapacity P_((Ring * r, int minsize));
extern Bool RING_Put P_((Ring * r, RElement e));
extern Bool RING_PutFront P_((Ring * r, RElement e));
extern RElement RING_Get P_((Ring * r));
extern RElement RING_GetLast P_((Ring * r));
extern RElement RING_ElementAt P_((const Ring * r, int pos));
extern int RING_IndexOf P_((Ring * r, RElement e));

#define RING_Contains(r,e) (RING_IndexOf(r,e) >= 0)

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_RING_H_ */

/*
 * HISTORY:
 *
 * $Log: s_ring.h,v $
 * Revision 1.2  2009/10/15 12:58:39  slava
 * o added RING_IndexOf
 *
 * Revision 1.1  2008/09/03 09:24:35  slava
 * o added ring buffer object
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
