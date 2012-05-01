/*
 * $Id: s_ring.c,v 1.4 2009/10/15 12:58:39 slava Exp $
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

#include "s_ring.h"
#include "s_mem.h"

/*==========================================================================*
 *              R I N G    B U F F E R 
 *==========================================================================*/

/**
 * Creates a ring buffer that has no limit on number of entries
 */
Ring * RING_Create(void)
{
    Ring * r = MEM_New(Ring);
    if (r) {
        RING_Init(r);
    }
    return r;
}

/**
 * Creates a ring buffer preallocating certain amount of space and possibly
 * limiting maximum number of entries.
 */
Ring * RING_Create2(int init, int max)
{
    Ring * r = MEM_New(Ring);
    if (r) {
        if (RING_Init2(r, init, max)) {
            return r;
        }
        MEM_Free(r);
    }
    return NULL;
}

/**
 * Deallocates a buffer
 */
void RING_Delete(Ring * r)
{
    if (r) {
        RING_Destroy(r);
        MEM_Free(r);
    }
}

/**
 * Initializes the buffer that has no limit on number of entries
 */
void RING_Init(Ring * r)
{
    memset(r, 0, sizeof(*r));
    r->start = r->end = r->maxsiz = -1;
}

/**
 * Initializes the ring buffer preallocating certain amount of space and
 * possibly limiting maximum number of entries.
 */
Bool RING_Init2(Ring * r, int init, int max)
{
    ASSERT(init >= 0);
    ASSERT(max < 0 || init <= max);
    memset(r, 0, sizeof(*r));
    r->start = r->end = -1;
    r->maxsiz = max;
    if (init > 0 && max != 0) {
        if (max > 0 && init > max) init = max;
        r->data = MEM_NewArray(RElement,init);
        if (r->data) {
            r->alloc = init;
        } else {
            return False;
        }
    }
    return True;
}

/**
 * Destroys the ring buffer.
 */
void RING_Destroy(Ring * r)
{
    MEM_Free(r->data);
    r->data = NULL;
    r->alloc = 0;
    r->start = r->end = -1;
}

/**
 * Clears the ring buffer
 */
void RING_Clear(Ring * r)
{
    r->start = r->end = -1;
}

/**
 * Returns number of entries in the ring buffer
 */
int RING_Size(const Ring * r)
{
    if (r->start < 0) {
        return 0;
    } else if (r->start > r->end) {
        return (r->alloc + r->end - r->start);
    } else if (r->end > r->start) {
        return (r->end - r->start);
    } else {
        ASSERT(r->start >= 0 && r->start < r->alloc);
        ASSERT(r->end >= 0 && r->end < r->alloc);
        return r->alloc;
    }
}

/**
 * Returns True if the ring buffer is empty, False otherwise.
 */
Bool RING_IsEmpty(const Ring * r)
{
    return (r->start < 0);
}

/**
 * Compacts the ring buffer by freeing all unused memory.
 */
void RING_Compact(Ring * r)
{
    int size = RING_Size(r);
    if (r->alloc > size) {
        if (size > 0) {
            RElement * newbuf = MEM_NewArray(RElement,size);
            if (newbuf) {
                if (r->start < r->end) {
                    memcpy(newbuf, r->data + r->start, sizeof(RElement)*size);
                } else if (size > 0) {
                    int tail = r->alloc - r->start;
                    memcpy(newbuf, r->data + r->start, sizeof(RElement)*tail);
                    memcpy(newbuf + tail, r->data, sizeof(RElement) * r->end);
                }
                MEM_Free(r->data);
                r->data = newbuf;
                r->alloc = size;
                r->start = 0;
                r->end = 0;
            }
        } else if (r->data) {
            MEM_Free(r->data);
            r->data = NULL;
            r->alloc = 0;
        }
    }
}

/**
 * Makes sure that the buffer has enough room to contain at least minsize
 * entries.
 */
Bool RING_EnsureCapacity(Ring * r, int minsize)
{
    if (minsize > r->alloc) {
        RElement * newbuf;
        /* at least double the allocation size */
        minsize = MAX(minsize, r->alloc*2);
        newbuf = MEM_NewArray(RElement,minsize);
        if (newbuf) {
            if (r->start < r->end) {
                int n = r->end - r->start;
                memcpy(newbuf, r->data + r->start, sizeof(RElement) * n);
                r->start = 0;
                r->end = n;
            } else if (r->start >= 0) {
                int tail = r->alloc - r->start;
                memcpy(newbuf, r->data + r->start, sizeof(RElement) * tail);
                memcpy(newbuf + tail, r->data, sizeof(RElement) * r->end);
                r->start = 0;
                r->end += tail;
                ASSERT(r->end < minsize);
            }

            MEM_Free(r->data);
            r->data = newbuf;
            r->alloc = minsize;
            return True;
        }
        return False;
    }
    return True;
}

/**
 * Puts the entry to the tail of the ring buffer
 */
Bool RING_Put(Ring * r, RElement e)
{
    if (RING_EnsureCapacity(r, RING_Size(r) + 1)) {
        if (r->start < 0) {
            r->start = r->end = 0;
        }
        ASSERT(r->end < r->alloc);
        r->data[r->end++] = e;
        r->end %= r->alloc;
        return True;
    }
    return False;
}

/**
 * Puts the entry to the front of the ring buffer
 */
Bool RING_PutFront(Ring * r, RElement e)
{
    if (RING_EnsureCapacity(r, RING_Size(r) + 1)) {
        if (r->start >= 0) {
            r->start = (r->start ? r->start : r->alloc) - 1;
        } else {
            r->start = 0;
            r->end = 1;
        }
        r->data[r->start] = e;
        return True;
    }
    return False;
}

/**
 * Removes one entry from the front of the ring buffer
 */
RElement RING_Get(Ring * r)
{
    if (r->start >= 0) {
        RElement e = r->data[r->start++];
        if (r->start == r->end) {
            r->start = r->end = -1;
        } else {
            r->start %= r->alloc;
            if (r->start == r->end) {
                r->start = r->end = -1;
            }
        }
        return e;
    }
    return NULL;
}

/**
 * Removes one entry from the tail of the ring buffer
 */
RElement RING_GetLast(Ring * r)
{
    if (r->start >= 0) {
        RElement e;
        r->end = (r->end ? r->end : r->alloc) - 1;
        e = r->data[r->end];
        if (r->start == r->end) {
            r->start = r->end = -1;
        }
        return e;
    }
    return NULL;
}

/**
 * Returns the element at the specified position
 */
RElement RING_ElementAt(const Ring * r, int pos)
{
    if (pos < RING_Size(r)) {
        return r->data[(r->start + pos) % r->alloc];
    }
    return NULL;
}

/**
 * Finds the position of the specified element in the ring buffer.
 * Returns -1 if the specified element wasn't found.
 */
int RING_IndexOf(Ring * r, RElement e)
{
    const int n = RING_Size(r);
    int i;
    for (i=0; i<n; i++) {
        if (r->data[(r->start + i) % r->alloc] == e) {
            return i;
        }
    }
    return -1;
}

/*
 * HISTORY:
 *
 * $Log: s_ring.c,v $
 * Revision 1.4  2009/10/15 12:58:39  slava
 * o added RING_IndexOf
 *
 * Revision 1.3  2008/09/05 11:44:21  slava
 * o wrote a test and fixed a bunch of bugs
 *
 * Revision 1.2  2008/09/03 12:22:53  slava
 * o fixed a bug in RING_Get
 *
 * Revision 1.1  2008/09/03 09:24:35  slava
 * o added ring buffer object
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
