/*
 * $Id: s_buf.c,v 1.51 2010/12/25 07:50:45 slava Exp $
 *
 * Copyright (C) 2000-2010 by Slava Monich
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

#include "s_buf.h"
#include "s_mem.h"

#define _DATA_Swap16(_data) (\
    (((I16u)(_data)) << 8)  | \
    (((I16u)(_data)) >> 8)  )

#define _DATA_Swap32(_data) \
    ((((I32u)(_data)) >> 24) | \
    ((((I32u)(_data)) & 0x00ff0000) >> 8) | \
    ((((I32u)(_data)) & 0x0000ff00) << 8) | \
    ((((I32u)(_data)) & 0x000000ff) << 24))

/*==========================================================================*
 *              B U F F E R 
 *==========================================================================*/

/**
 * Allocates a buffer
 */
Buffer * BUFFER_Create() 
{
    Buffer * b = MEM_New(Buffer);
    if (b) {
        BUFFER_Init(b);
    }
    return b;
}

/**
 * Creates a buffer
 */
Buffer * BUFFER_Create2(void * buf, size_t size)
{
    Buffer * b = MEM_New(Buffer);
    if (b) {
        BUFFER_Init2(b,buf,size);
    }
    return b;
}

/**
 * Creates a readonly buffer
 */
Buffer * BUFFER_CreateRead(const void * buf, size_t size)
{
    Buffer * b = MEM_New(Buffer);
    if (b) {
        BUFFER_InitRead(b,buf,size);
    }
    return b;
}

/**
 * Creates a writeable buffer that uses the memory provided by the caller.
 */
Buffer * BUFFER_CreateWrite(void * buf, size_t size, Bool ownBuf)
{
    Buffer * b = MEM_New(Buffer);
    if (b) {
        BUFFER_InitWrite(b,buf,size,ownBuf);
    }
    return b;
}

/**
 * Deallocates a buffer
 */
void BUFFER_Delete(Buffer * b)
{
    if (b) {
        BUFFER_Destroy(b);
        MEM_Free(b);
    }
}

/**
 * Initializes a buffer
 */
void BUFFER_Init(Buffer * b)
{
    memset(b,0,sizeof(*b));
    b->order = BYTE_ORDER;
    b->maxsiz = BUFFER_NO_LIMIT;
}

/**
 * Initializes a buffer. Assumes that the buffer owns the data, i.e. 
 * BUFFER_Destroy() or a resize will deallocate it
 */
void BUFFER_Init2(Buffer * b, void * buf, size_t size)
{
    ASSERT((buf && (size > 0)) || (!buf || (size == 0)));
    BUFFER_Init(b);
    b->data = (I8u*)buf;
    b->alloc = size;
    b->end = size;
    if (buf) b->flags |= BUFFER_OWN_DATA;
}

/**
 * Initializes a readonly buffer. Does not deallocate the internal buffer,
 * does not allow writing into it.
 */
void BUFFER_InitRead(Buffer * b, const void * buf, size_t size)
{
    ASSERT((buf && (size > 0)) || (!buf || (size == 0)));
    BUFFER_Init(b);
    b->data = (I8u*)buf;
    b->alloc = size;
    b->end = size;
    b->maxsiz = size;
    b->flags |= BUFFER_READONLY;
}

/**
 * Initializes Buffer for writing into the caller provided buffer.
 * If the last parameter is True, then the provided buffer will be 
 * deallocated by BUFFER_Destroy or when a resize happens. Otherwise,
 * it's assumed that caller owns the buffer and will deallocate it
 * (or if the buffer is located on stack). The buffer is initially 
 * empty.
 */
void BUFFER_InitWrite(Buffer * b, void * buf, size_t size, Bool ownBuf)
{
    ASSERT((buf && (size > 0)) || (!buf || (size == 0)));
    BUFFER_Init(b);
    b->data = (I8u*)buf;
    b->alloc = size;
    if (buf && ownBuf) b->flags |= BUFFER_OWN_DATA;
}

/**
 * Destroys a buffer
 */
void BUFFER_Destroy(Buffer * b)
{
    if (b->flags & BUFFER_OWN_DATA) {
        b->flags &= ~BUFFER_OWN_DATA;
        MEM_Free(b->data);
        b->data = NULL;
        b->alloc = 0;
        b->start = 0;
        b->end = 0;
    }
}

/**
 * Returns the amount of data stored in the buffer.
 */
size_t BUFFER_Size(const Buffer * b)
{
    ptrdiff_t size = (b->end - b->start);

    /* ASSERT that BUFFER_FULL flags is being used correctly */
    ASSERT(size == 0 || (!b->start && (b->end == b->alloc)) || 
          !(b->flags & BUFFER_FULL));

    /* handle "wrapped" buffers */
    if (size < 0 || ((size == 0) && (b->flags & BUFFER_FULL))) {
        size += b->alloc;
    }
    return size;
}

/**
 * Returns True if the buffer is readonly
 */
Bool BUFFER_IsReadOnly(const Buffer * b)
{
    return BoolValue((b->flags & BUFFER_READONLY) != 0);
}

/**
 * Clears the buffer
 */
void BUFFER_Clear(Buffer * b)
{
    b->start = b->end = 0;
    b->flags &= ~BUFFER_FULL;
}

/**
 * Deallocates unused memory
 */
void BUFFER_Trim(Buffer * b)
{
    size_t size = BUFFER_Size(b);
    if (size < b->alloc) {
        if (size > 0) {
            I8u * newbuf = MEM_NewArray(I8u, size);
            if (newbuf) {
                if (b->end > b->start) {
                    memcpy(newbuf, b->data + b->start, size);
                } else {
                    memcpy(newbuf, b->data + b->start, b->alloc - b->start);
                    memcpy(newbuf + (b->alloc - b->start), b->data, b->end);
                }
                if (b->flags & BUFFER_OWN_DATA) {
                    MEM_Free(b->data);
                }
                b->data = newbuf;
                b->start = b->end = 0;
                b->alloc = size;
                b->flags |= (BUFFER_FULL | BUFFER_OWN_DATA);
            }
        } else if (b->flags & BUFFER_OWN_DATA) {
            b->flags &= ~BUFFER_OWN_DATA;
            MEM_Free(b->data);
            b->data = NULL;
            b->alloc = 0;
        }
    }
}

/**
 * Prepares buffer for direct access. Makes sure that there's no rollover,
 * all the data is available as a single continuus block of memory. If the 
 * buffer is empty, the return value is undefined.
 */
void * BUFFER_Access(Buffer * b)
{
    if ((b->end < b->start) || 
       ((b->end == b->start) && (b->flags & BUFFER_FULL))) {

        size_t n = b->alloc - b->start;
        size_t size = n;
        if (b->end != b->alloc) size += b->end;
        ASSERT(size == BUFFER_Size(b));
        if ((n + 2 * b->end) <= b->alloc) {

            /* we can quickly move the data, no reallocation is necessary */
            memmove(b->data + b->end, b->data + b->start, n);
            memmove(b->data + b->end + n, b->data, b->end);

            /* update offsets */
            b->start = b->end;
            b->end = b->start + size;

        } else if (b->flags & BUFFER_OWN_DATA) {

            /* we better reallocate the whole thing */
            I8u * newbuf = MEM_NewArray(I8u,size);
            if (newbuf) {
                memcpy(newbuf, b->data + b->start, n);
                memcpy(newbuf + n, b->data, b->end);

                /* switch to the new buffer */
                MEM_Free(b->data);
                b->flags |= BUFFER_FULL;
                b->start = 0;
                b->end = size;
                b->alloc = size;
                b->data = newbuf;
            } else {
                return NULL;
            }

        } else {

            /* don't reallocate the buffer if we don't own it */
            if (n > b->end) {

                /* the "head" is longer that the "tail" */
                size_t off = b->alloc - size;
                size_t tmpsize = b->end - off;
                I8u * tmpbuf = MEM_NewArray(I8u,tmpsize);
                if (tmpbuf) {
                    memcpy(tmpbuf, b->data + off, tmpsize);
                    memmove(b->data + off, b->data + b->start, n);
                    memcpy(b->data + off + n, b->data, off);
                    memcpy(b->data + off + n + off, tmpbuf, tmpsize);
                    MEM_Free(tmpbuf);

                    b->start = off;
                    b->end = b->alloc;
                } else {
                    return NULL;
                }
            } else {

                /* the "tail" is longer that the "head" */
                size_t tmpsize = size - b->start;
                I8u * tmpbuf = MEM_NewArray(I8u,tmpsize);
                if (tmpbuf) {
                    memcpy(tmpbuf, b->data + b->start, tmpsize);
                    memmove(b->data + size - b->end, b->data, b->end);
                    memcpy(b->data, tmpbuf, tmpsize);
                    memcpy(b->data + tmpsize, b->data + size, n - tmpsize);
                    MEM_Free(tmpbuf);

                    b->start = 0;
                    b->end = size;
                } else {
                    return NULL;
                }

            }
        }
    }
    return (b->data + b->start);
}

/**
 * Makes sure that the buffer has enough room to contain at least n bytes 
 * of data. If partOK is True and the buffer has a limit on the amounf of 
 * allocated data, this function will (try to) resize the buffer to maximum 
 * allowed size even if that is still not enough to hold the required amount
 * of data.
 */
Bool BUFFER_EnsureCapacity(Buffer * b, size_t minsize, Bool partOK)
{
    if (minsize > b->alloc) {
        I8u * newbuf;
        if (minsize > b->maxsiz){
            if (partOK) {
                minsize = b->maxsiz;
            } else {
                return False;
            }
        } else {
            /* at least double the allocation size */
            minsize = MAX(minsize, b->alloc*2);
        }

        /* should use Realloc to minimize copying? */
        newbuf = MEM_NewArray(I8u,minsize);
        if (newbuf) {
            if (b->end < b->start || (b->flags & BUFFER_FULL)) {
                memcpy(newbuf, b->data + b->start, b->alloc - b->start);
                memcpy(newbuf + b->alloc - b->start, b->data, b->end);
                b->end = b->alloc - b->start + b->end;
            } else if (b->end > b->start) {
                memcpy(newbuf, b->data + b->start, b->end - b->start);
                b->end = b->end - b->start;
            } else {
                b->end = 0;
            }

            if (b->flags & BUFFER_OWN_DATA) MEM_Free(b->data);
            b->flags |= BUFFER_OWN_DATA;
            b->flags &= ~BUFFER_FULL;
            b->data = newbuf;
            b->alloc = minsize;
            b->start = 0;

        } else {
            return False;
        }
    }
    return True;
}

/**
 * Pushes n bytes of opaque data *before* the current position in the buffer,
 * no conversion. Returns True if all data have been pushed back to the 
 * buffer, False if an error ocured and no data have been pushed back.
 * If the data pointer if NULL, this function just moves the start pointer
 * n positions back. This is useful if you are pushing back the same data
 * that you just read from this buffer - allows you to avoid unnecessary 
 * copying.
 */
Bool BUFFER_PushBack(Buffer * b, const void * data, size_t n)
{
    if (b->flags & BUFFER_READONLY) {
        return False;
    } else {
        size_t written = 0;
        size_t avail, size = BUFFER_Size(b);
        if (BUFFER_EnsureCapacity(b, size + n, False)) {
            avail = b->alloc - size;
            if (n > avail) n = avail;
            if (n > 0) {
                size_t nbytes, n1, n2;
                if (b->start > b->end) {
                    n2 = b->start - b->end;
                    n1 = 0;
                } else {
                    n1 = b->start;
                    n2 = b->alloc - b->end;
                }
                if (n1 > 0) {
                    nbytes = MIN(n1,n);
                    b->start -= nbytes;
                    if (data) {
                        const I8u * src = ((I8u*)data) + n - written - nbytes;
                        memcpy(b->data + b->start, src, nbytes);
                    }
                    written += nbytes;
                    n -= nbytes;
                }
                if (n > 0 && n2 > 0) {
                    nbytes = MIN(n2,n);
                    b->start = b->alloc - nbytes;
                    if (data) {
                        const I8u * src = ((I8u*)data) + n - written - nbytes;
                        memcpy(b->data + b->start, src, nbytes);
                    }
                    written += nbytes;
                    n -= nbytes;
                }
                if (b->end == b->start) {
                    b->flags |= BUFFER_FULL;
                }
            }
        }
        return BoolValue(written == n);
    }
}

/**
 * Puts n bytes of opaque data into the buffer, no conversion.
 * Returns number of bytes stored in the buffer. If partOK is True,
 * it may put only some of the input data into the buffer and returns
 * the amount of data it was able to put in; if partOK is False, then 
 * this function stores all or nothing. In the former case, the function
 * can return any number between zero and n, inclusive. In the latter case,
 * the return value will be either n (success) or zero (failure).
 */
size_t BUFFER_Put(Buffer * b, const void * data, size_t n, Bool partOK)
{
    size_t written = 0;
    if (n > 0 && !(b->flags & BUFFER_READONLY)) {
        size_t avail, size = BUFFER_Size(b);
        if (BUFFER_EnsureCapacity(b, size + n, partOK)) {
            avail = b->alloc - size;
            if (n > avail) n = avail;
            if (n > 0) {
                size_t nbytes, n1, n2;
                if (b->start > b->end) {
                    n1 = b->start - b->end;
                    n2 = 0;
                } else {
                    n1 = b->alloc - b->end;
                    n2 = b->start;
                }
                if (n1 > 0) {
                    nbytes = MIN(n1,n);
                    memcpy(b->data + b->end, data, nbytes);
                    written += nbytes;
                    n -= nbytes;
                }
                if (n > 0 && n2 > 0) {
                    nbytes = MIN(n2,n);
                    memcpy(b->data, ((I8u*)data) + written, nbytes);
                    written += nbytes;
                    n -= nbytes;
                }

                /* update the end position */
                b->end += written;
                if (b->end > b->alloc) {
                    b->end %= b->alloc;
                }
                if (b->end == b->start) {
                    b->flags |= BUFFER_FULL;
                }
            }
        }
    }
    return written;
}

/**
 * Descards the last n bytes put into the buffer. Returns the actual number
 * of characters that have been discarded (obviously, you cannot discard more
 * than you have put in). This function can be used to undo the damage to the
 * buffer.
 */
size_t BUFFER_Unput(Buffer * b, size_t n)
{
    size_t nbytes;
    ASSERT(n >= 0);
    if (n <= 0) {
        nbytes = 0;
    } else if (n >= BUFFER_Size(b)) {
        nbytes = BUFFER_Size(b);
        BUFFER_Clear(b);
    } else {
        nbytes = n;
        if (b->start < b->end || n < b->end) {
            b->end -= n;
        } else {
            n -= b->end;
            b->end = b->alloc;
            if (n > 0) {
                ASSERT(n < (b->end - b->start));
                b->end -= n;
            }
        }
    }
    return nbytes;
}

/**
 * Retrieves some amount of opaque data from the buffer, no conversion.
 * Returns number of bytes retrieved from the buffer. The data parameter
 * can be NULL, in which case this function just moves the current buffer 
 * position forward by at most size bytes.
 */
size_t BUFFER_Get(Buffer * b, void * data, size_t size)
{
    size_t nread = 0;
    if (size > 0) {
        size_t avail = BUFFER_Size(b);
        if (avail > 0) {
            size_t n, n1, n2;
            if (b->end > b->start) {
                n1 = b->end - b->start;
                n2 = 0;
            } else {
                n1 = b->alloc - b->start;
                n2 = b->end;
            }
            if (n1 > 0) {
                n = MIN(n1,size);
                n = MIN(n,avail);
                if (data) memcpy(data, b->data + b->start, n);
                nread += n;
                avail -= n;
                size -= n;
            }
            if (size > 0 && avail > 0 && n2 > 0) {
                n = MIN(n2,size);
                n = MIN(n,avail);
                if (data) memcpy(((I8u*)data) + nread, b->data, n);
                nread += n;
                avail -= n;
            }

            /* update the start position */
            b->start += nread;
            if (b->start == b->end) {
                b->start = b->end = 0;
            } else if (b->start >= b->alloc) {
                b->start %= b->alloc;
            }

            /* 
             * since we have taken out some data, buffer cannot be 
             * full anymore 
             */
            ASSERT(nread > 0);
            b->flags &= ~BUFFER_FULL;
        }
    }
    return nread;
}

/**
 * Moves up to size bytes from one buffer to another. Returns number of bytes
 * moved, or zero if it fails to reallocate the destination buffer.
 */
size_t BUFFER_Move(Buffer * b, Buffer * dest, size_t size)
{
    size_t moved = 0;
    size_t avail = BUFFER_Size(b);
    if (size > avail) size = avail;
    if (size && BUFFER_EnsureCapacity(dest,BUFFER_Size(dest)+size,False)) {
        size_t n, n1, n2;
        if (b->end > b->start) {
            n1 = b->end - b->start;
            n2 = 0;
        } else {
            n1 = b->alloc - b->start;
            n2 = b->end;
        }
        if (n1 > 0) {
            n = MIN(n1,size);
            n = MIN(n,avail);
            VERIFY(BUFFER_Put(dest, b->data + b->start, n, False));
            moved += n;
            avail -= n;
            size -= n;
        }
        if (size > 0 && avail > 0 && n2 > 0) {
            n = MIN(n2,size);
            n = MIN(n,avail);
            VERIFY(BUFFER_Put(dest, b->data, n, False));
            moved += n;
            avail -= n;
        }

        /* update the start position */
        b->start += moved;
        if (b->start == b->end) {
            ASSERT(avail == 0);
            b->start = b->end = 0;
        } else if (b->start >= b->alloc) {
            b->start %= b->alloc;
        }

        /* 
         * since we have taken out some data, buffer cannot be 
         * full anymore 
         */
        b->flags &= ~BUFFER_FULL;
        ASSERT(avail == BUFFER_Size(b));
        ASSERT(moved > 0);
    }
    return moved;

}

/* put words of different sizes to the buffer */

Bool BUFFER_PutI8(Buffer * b, I8s data)
{
    return BoolValue(BUFFER_Put(b,&data,sizeof(data),False) == sizeof(data));
}

Bool BUFFER_PutI16(Buffer * b, I16s data)
{
    data = DATA_Conv16(data,BYTE_ORDER,b->order);
    return BoolValue(BUFFER_Put(b,&data,sizeof(data),False) == sizeof(data));
}

Bool BUFFER_PutI32(Buffer * b, I32s data)
{
    data = DATA_Conv32(data,BYTE_ORDER,b->order);
    return BoolValue(BUFFER_Put(b,&data,sizeof(data),False) == sizeof(data));
}

Bool BUFFER_PutI64(Buffer * b, I64s data)
{
    data = DATA_Conv64(data,BYTE_ORDER,b->order);
    return BoolValue(BUFFER_Put(b,&data,sizeof(data),False) == sizeof(data));
}

/* get words of different sizes from the buffer */

Bool BUFFER_GetI8(Buffer * b, I8s * data)
{
    return BUFFER_GetU8(b, (I8u*)data);
}

Bool BUFFER_GetU8(Buffer * b, I8u * data)
{
    size_t avail = BUFFER_Size(b);
    if (avail >= sizeof(*data)) {
        I8u buf = 0;
        BUFFER_Get(b,&buf,sizeof(buf));
        if (data) *data = buf;
        return True;
    }
    return False;
}

Bool BUFFER_GetI16(Buffer * b, I16s * data)
{
    return BUFFER_GetU16(b, (I16u*)data);
}

Bool BUFFER_GetU16(Buffer * b, I16u * data)
{
    size_t avail = BUFFER_Size(b);
    if (avail >= sizeof(*data)) {
        I16u buf = 0;
        BUFFER_Get(b,&buf,sizeof(buf));
        if (data) *data = DATA_Conv16(buf,b->order,BYTE_ORDER);
        return True;
    }
    return False;
}

Bool BUFFER_GetI32(Buffer * b, I32s * data)
{
    return BUFFER_GetU32(b, (I32u*)data);
}

Bool BUFFER_GetU32(Buffer * b, I32u * data)
{
    size_t avail = BUFFER_Size(b);
    if (avail >= sizeof(*data)) {
        I32u buf = 0;
        BUFFER_Get(b,&buf,sizeof(buf));
        if (data) *data = DATA_Conv32(buf,b->order,BYTE_ORDER);
        return True;
    }
    return False;
}

Bool BUFFER_GetI64(Buffer * b, I64s * data)
{
    return BUFFER_GetU64(b, (I64u*)data);
}

Bool BUFFER_GetU64(Buffer * b, I64u * data)
{
    size_t avail = BUFFER_Size(b);
    if (avail >= sizeof(*data)) {
        I64u buf = 0;
        BUFFER_Get(b,&buf,sizeof(buf));
        if (data) *data = DATA_Conv64(buf,b->order,BYTE_ORDER);
        return True;
    }
    return False;
}

/* 
 * Byte order converter for 16-bit numbers
 */
I16u DATA_Conv16(I16u data, int from, int to)
{
    /* we only have two byte orders to convert between */
    ASSERT(from == BIG_ENDIAN || from == LITTLE_ENDIAN);
    ASSERT(to == BIG_ENDIAN || to == LITTLE_ENDIAN);
    if (from != to) {
        return _DATA_Swap16(data);
    } else {
        return data;
    }
}

/* 
 * Byte order converter for 32-bit numbers
 */
I32u DATA_Conv32(I32u data, int from, int to)
{
    /* we only have two byte orders to convert between */
    ASSERT(from == BIG_ENDIAN || from == LITTLE_ENDIAN);
    ASSERT(to == BIG_ENDIAN || to == LITTLE_ENDIAN);
    if (from != to) {
        return _DATA_Swap32(data);
    } else {
        return data;
    }
}

/* 
 * Byte order converter for 64-bit numbers
 */
I64u DATA_Conv64(I64u data, int from, int to)
{
    /* we only have two byte orders to convert between */
    ASSERT(from == BIG_ENDIAN || from == LITTLE_ENDIAN);
    ASSERT(to == BIG_ENDIAN || to == LITTLE_ENDIAN);
    if (from != to) {
        const I32u low  = (I32u)data;
        const I32u high = (I32u)(data >> 32);
        return (((I64u)_DATA_Swap32(low)) << 32) | ((I64u)_DATA_Swap32(high));
    } else {
        return data;
    }
}

/* 
 * BIG_ENDIAN <-> LITTLE_ENDIAN conversion
 */
I16u DATA_Swap16(I16u data)
{
    return _DATA_Swap16(data);
}

I32u DATA_Swap32(I32u data)
{
    return _DATA_Swap32(data);
}

I64u DATA_Swap64(I64u data)
{
    const I32u low  = (I32u)data;
    const I32u high = (I32u)(data >> 32);
    return (((I64u)_DATA_Swap32(low)) << 32) | ((I64u)_DATA_Swap32(high));
}

/*
 * HISTORY:
 *
 * $Log: s_buf.c,v $
 * Revision 1.51  2010/12/25 07:50:45  slava
 * o added DATA_Swap16, DATA_Swap32 and DATA_Swap64 functions
 *
 * Revision 1.50  2009/12/26 12:21:32  slava
 * o working on 64-bit issues
 *
 * Revision 1.49  2008/12/12 15:16:04  slava
 * o added BUFFER_Trim function
 *
 * Revision 1.48  2006/03/28 00:01:07  slava
 * o clear b->alloc field in BUFFER_Destroy
 *
 * Revision 1.47  2005/10/19 21:11:10  slava
 * o added BUFFER_Move function
 *
 * Revision 1.46  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.45  2004/07/19 22:49:30  slava
 * o added BUFFER_Unput function
 *
 * Revision 1.44  2004/04/08 01:44:18  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.43  2004/04/05 03:33:55  slava
 * o fixed another pedantic compilation warning
 *
 * Revision 1.42  2004/04/01 19:03:52  slava
 * o added BUFFER_CreateWrite and BUFFER_InitWrite
 *
 * Revision 1.41  2004/03/25 19:06:00  slava
 * o fixed a few pedantic compilation warnings
 *
 * Revision 1.40  2003/05/21 00:11:03  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.39  2003/04/21 06:16:15  slava
 * o fixed a bug in BUFFER_Get which didn't quite handle rollovers. It only
 *   handled the cases if the reads (i.e. gets) are nicely aligned in respect
 *   to the end of the buffer, which I guess is most often the case; otherwise
 *   I would have noticed it long time ago. However, if the requested data
 *   wrap around the end of the buffer (i.e. start at the end of the buffer
 *   and end at the beginning), this function would break.
 *
 * Revision 1.38  2002/09/23 02:05:52  slava
 * o updated comments
 *
 * Revision 1.37  2002/08/23 03:43:26  slava
 * o added BUFFER_PushBack
 *
 * Revision 1.36  2002/07/08 02:08:23  slava
 * o fixed a bug in BUFFER_Init which didn't initialize the byte order
 *
 * Revision 1.35  2002/06/08 17:02:11  slava
 * o support for readonly buffers
 *
 * Revision 1.34  2002/05/22 05:02:41  slava
 * o fixed broken logic in BUFFER_Get (thanks to gcc for compilation warning)
 *
 * Revision 1.33  2002/05/22 04:13:30  slava
 * o Buffer of binary data. Function are provided to store the data in
 *   or retrieve from the buffer in FIFO (first in - first out) manner.
 *   The buffer can have internal storage of fixed or variable (possibly,
 *   limited) size. Buffer may automatically resize itself if necessary,
 *   but only if it's allowed to do so.
 *
 * Local Variables:
 * c-basic-offset:4
 * indent-tabs-mode: nil
 * End:
 */
