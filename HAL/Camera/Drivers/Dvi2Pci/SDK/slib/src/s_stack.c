/*
 * $Id: s_stack.c,v 1.9 2003/05/21 00:11:04 slava Exp $
 *
 * Copyright (C) 2000-2003 by Oleg Levin and Slava Monich
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

#include "s_stack.h"
#include "s_mem.h"

/**
 * Allocates a new stack and sets it up.
 */
Stack * STACK_Create(int n)
{
    Stack * s = MEM_New(Stack);
    if (s) {
        if (STACK_Init(s, n)) {
            return s;
        }
        MEM_Free(s);
    }
    return NULL;
}

/**
 * Initialize the stack, allocate initial storage if necessary.
 * Returns False if memory allocation fails.
 */
Bool STACK_Init(Stack * s, int n)
{
    s->size = 0;
    s->alloc = MAX(n,0);
    if (s->alloc > 0) {
        s->stack = MEM_NewArray(SElement, s->alloc);
        if (!s->stack) {
            return False;
        }
        DEBUG_ONLY(memset(s->stack, 0, sizeof(s->stack[0])*s->alloc));
    } else {
        s->stack = NULL;
    }
    return True;
}

/**
 * Destroy the contents of the stack and deallocate the stack itself.
 */
void STACK_Delete(Stack * s)
{
    if (s) {
        STACK_Destroy(s);
        MEM_Free(s);
    }
}

/**
 * Clear the stack.
 */
void STACK_Clear(Stack * s)
{
    DEBUG_ONLY(memset(s->stack, 0, sizeof(s->stack[0])*s->size));
    s->size = 0;
}

/**
 * Clear the stack and deallocate internal buffer.
 */
void STACK_Destroy(Stack * s)
{
    STACK_Clear(s);
    s->alloc = 0;
    if (s->stack) {
        MEM_Free(s->stack);
        s->stack = NULL;
    }
}

/**
 * Removes element from the top of the stack
 * and returns this value.
 */
SElement STACK_Pop(Stack * s)
{
    if(s->size) {
        return s->stack[--s->size];
    }
    ASSMSG("STACK: underflow");
    return NULL;
}

/**
 * Looks at the element on the top of the stack
 * without removing it.
 */
SElement STACK_Peek(const Stack * s)
{
    if(s->size) {
        return s->stack[s->size-1];
    }
    ASSMSG("STACK: stack is empty");
    return NULL;
}

/**
 * Returns True if the stack contains the specified element
 */
Bool STACK_Contains(const Stack * s, SElementC e)
{
    int i;
    for (i=s->size-1; i>=0; i--) {
        if (s->stack[i] == e) {
            return True;
        }
    }
    return False;
}

/**
 * Increases the capacity of the stack, if necessary, to ensure
 * that it can hold at least the number of elements specified by
 * the minimum capacity argument. Returns False if memory allocation
 * fails.
 */
STATIC Bool STACK_EnsureCapacity(Stack * s, int min)
{
    int oldlen = s->alloc;
    if (min > oldlen) {
        SElement* newbuf;
        int newlen = (oldlen * 3)/2 + 1;
        if (newlen < min) newlen = min;
#ifdef NO_REALLOC
        newbuf = (SElement*)MEM_Alloc(sizeof(SElement) * newlen);
        if (newbuf && s->stack) {
            memcpy(newbuf, s->stack, s->alloc * sizeof(SElement));
            MEM_Free(s->stack);
        }
#else
        newbuf = (SElement*)MEM_Realloc(s->stack, sizeof(SElement) * newlen);
#endif /* NO_REALLOC */
        if (!newbuf) return False;

        /* switch to the new buffer */
        DEBUG_ONLY(memset(newbuf+s->size,0,sizeof(SElement)*(newlen-s->size)));
        s->stack = newbuf;
        s->alloc = newlen;
    }
    return True;
}

/**
 * Adds element on the top of the stack.
 * Returns False if memory allocation failed
 */
Bool STACK_Push(Stack * s, SElement e)
{
    if( STACK_EnsureCapacity(s, s->size + 1) ) {
        s->stack[s->size++] = e;
        return True;
    }
    return False;
}

/**
 *
 * HISTORY:
 *
 * $Log: s_stack.c,v $
 * Revision 1.9  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.8  2002/08/27 11:52:03  slava
 * o some reformatting
 *
 * Revision 1.7  2002/01/02 07:51:52  slava
 * o added STACK_Contains
 *
 * Revision 1.6  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.5  2001/05/27 11:45:51  slava
 * o port to NT kernel mode environment
 *
 * Revision 1.4  2001/05/18 22:36:57  slava
 * o THIS_FILE variable is no longer being used
 *
 * Revision 1.3  2001/01/31 01:53:58  slava
 * o ASSERT that stack is not empty in STACK_Pop() and STACK_Peek()
 *
 * Revision 1.2  2001/01/08 01:41:08  slava
 * o removed Oleg's comment that was referring to the bug which is now fixed
 *
 * Revision 1.1  2000/12/31 02:55:47  oleg
 * o added Stack to slib
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
