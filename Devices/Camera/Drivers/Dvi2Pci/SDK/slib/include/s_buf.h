/*
 * $Id: s_buf.h,v 1.30 2010/12/25 07:50:45 slava Exp $
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

#ifndef _SLAVA_BUFFER_H_
#define _SLAVA_BUFFER_H_

#include "s_def.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

typedef short ByteOrder;

/*
 * Buffer of binary data. Function are provided to store the data in
 * or retrieve from the buffer in FIFO (first in - first out) manner.
 * The buffer can have internal storage of fixed or variable (possibly,
 * limited) size. Buffer will automatically resize itself if necessary,
 * and it's allowed to do so. If the buffer is not allowed to resize,
 * a put (store) operation will fail if the buffer is full.
 *
 * NOTE that start offset is INCLUSIVE, while end offset is EXCLUSIVE. 
 * If start offset equals the end offset, it means that buffer is either
 * empty or full. To distinguish between these two cases, we use the 
 * BUFFER_FULL flag. There's one case though, when buffer is full but 
 * start and end offsets MAY NOT be equal; that's when start offset is 
 * zero. In that case, end offset MAY be equal the buffer's allocated 
 * size, and BUFFER_FULL flag may or may not be set.
 */
typedef struct _Buffer {
    I8u * data;                 /* data storage */
    size_t alloc;               /* allocated buffer space */
    size_t maxsiz;              /* max space to allocate or BUFFER_NO_LIMIT */
    size_t start;               /* start offset (inclusive) */
    size_t end;                 /* end offset (exclusive) */
    ByteOrder order;            /* byte order */
    short flags;                /* flags, see below: */

#define BUFFER_OWN_DATA 0x0001  /* BUFFER_Destroy() deallocates the data */
#define BUFFER_READONLY 0x0002  /* cannot write into the buffer */
#define BUFFER_FULL     0x0004  /* buffer is full */

} Buffer;

#define BUFFER_NO_LIMIT ((size_t)-1) /* value of maxsiz if there's no limit */

/* operations on buffer */
extern Buffer * BUFFER_Create P_((void));
extern Buffer * BUFFER_Create2 P_((void * buf, size_t size));
extern Buffer * BUFFER_CreateRead P_((const void * buf, size_t size));
extern Buffer * BUFFER_CreateWrite P_((void * buf, size_t size, Bool ownBuf));
extern void BUFFER_Delete P_((Buffer * b));

extern void BUFFER_Init P_((Buffer * b));
extern void BUFFER_Init2 P_((Buffer * b, void * buf, size_t size));
extern void BUFFER_InitRead P_((Buffer * b, const void * buf, size_t size));
extern void BUFFER_InitWrite P_((Buffer* b, void* buf, size_t size,Bool own));
extern void BUFFER_Destroy P_((Buffer * b));
extern void BUFFER_Clear P_((Buffer * b));
extern void BUFFER_Trim P_((Buffer * b));
extern Bool BUFFER_EnsureCapacity P_((Buffer* b, size_t size, Bool partOK));
extern Bool BUFFER_IsReadOnly P_((const Buffer * b));
extern size_t BUFFER_Size P_((const Buffer * b));

/* prepares buffer for direct access */
extern void * BUFFER_Access P_((Buffer * b));

/* opaque data, no conversion */
extern size_t BUFFER_Get P_((Buffer * b, void * data, size_t size));
extern size_t BUFFER_Put P_((Buffer* b, const void* d, size_t n, Bool part));
extern size_t BUFFER_Unput P_((Buffer * b, size_t n));
extern size_t BUFFER_Move P_((Buffer * b, Buffer * dest, size_t size));
extern Bool   BUFFER_PushBack P_((Buffer * b, const void * data, size_t n));

/* moving start position back and forth */
#define BUFFER_Skip(_b,_size)  BUFFER_Get(_b,NULL,_size)
#define BUFFER_Unget(_b,_size) BUFFER_PushBack(_b,NULL,_size)

/* byte order conversion */
extern Bool BUFFER_PutI8 P_((Buffer * b, I8s data));
extern Bool BUFFER_PutI16 P_((Buffer * b, I16s data));
extern Bool BUFFER_PutI32 P_((Buffer * b, I32s data));
extern Bool BUFFER_PutI64 P_((Buffer * b, I64s data));
extern Bool BUFFER_GetI8 P_((Buffer * b, I8s * data));
extern Bool BUFFER_GetU8 P_((Buffer * b, I8u * data));
extern Bool BUFFER_GetI16 P_((Buffer * b, I16s * data));
extern Bool BUFFER_GetU16 P_((Buffer * b, I16u * data));
extern Bool BUFFER_GetI32 P_((Buffer * b, I32s * data));
extern Bool BUFFER_GetU32 P_((Buffer * b, I32u * data));
extern Bool BUFFER_GetI64 P_((Buffer * b, I64s * data));
extern Bool BUFFER_GetU64 P_((Buffer * b, I64u * data));

/* aliases for native types */
#define BUFFER_PutByte(_b,_d)   BUFFER_PutI8(_b,_d)
#define BUFFER_PutChar(_b,_d)   BUFFER_PutI8(_b,_d)
#define BUFFER_PutShort(_b,_d)  BUFFER_PutI16(_b,_d)
#define BUFFER_PutInt(_b,_d)    BUFFER_PutI32(_b,_d)

#define BUFFER_GetByte(_b,_d)   BUFFER_GetI8(_b,_d)
#define BUFFER_GetChar(_b,_d)   BUFFER_GetI8(_b,_d)
#define BUFFER_GetUChar(_b,_d)  BUFFER_GetU8(_b,_d)
#define BUFFER_GetShort(_b,_d)  BUFFER_GetI16(_b,_d)
#define BUFFER_GetUShort(_b,_d) BUFFER_GetU16(_b,_d)
#define BUFFER_GetInt(_b,_d)    BUFFER_GetI32(_b,_d)
#define BUFFER_GetUInt(_b,_d)   BUFFER_GetU32(_b,_d)

#ifdef __LONG_64__
#  define BUFFER_PutLong(_b,_d) BUFFER_PutI64(_b,_d)
#  define BUFFER_GetLong(_b,_d) BUFFER_GetI64(_b,_d)
#  define BUFFER_GetULong(_b,d) BUFFER_GetU64(_b,d)
#else  /* !__LONG_64__ */
#  define BUFFER_PutLong(_b,_d) BUFFER_PutInt(_b,_d)
#  define BUFFER_GetLong(_b,_d) BUFFER_GetInt(_b,_d)
#  define BUFFER_GetULong(_b,d) BUFFER_GetUInt(_b,d)
#endif /* !__LONG_64__ */

/* byte order converters */
I16u DATA_Conv16 P_((I16u data, int from, int to));
I32u DATA_Conv32 P_((I32u data, int from, int to));
I64u DATA_Conv64 P_((I64u data, int from, int to));
I16u DATA_Swap16 P_((I16u data)); /* LE <-> BE */
I32u DATA_Swap32 P_((I32u data)); /* LE <-> BE */
I64u DATA_Swap64 P_((I64u data)); /* LE <-> BE */

/* redefine ntohs and ntohl as a noop if no convertion is necessary */
#undef ntohs
#undef ntohl
#undef htons
#undef htonl
#if BYTE_ORDER == BIG_ENDIAN
#  define ntohs(_x)  (_x)
#  define ntohl(_x)  (_x)
#  define htons(_x)  (_x)
#  define htonl(_x)  (_x)
#else
#  define ntohs(_x)  DATA_Swap16(_x)
#  define ntohl(_x)  DATA_Swap32(_x)
#  define htons(_x)  DATA_Swap16(_x)
#  define htonl(_x)  DATA_Swap32(_x)
#endif
 
#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_BUFFER_H_ */

/*
 * HISTORY:
 *
 * $Log: s_buf.h,v $
 * Revision 1.30  2010/12/25 07:50:45  slava
 * o added DATA_Swap16, DATA_Swap32 and DATA_Swap64 functions
 *
 * Revision 1.29  2009/12/26 12:22:40  slava
 * o working on 64-bit issues; some APIs have changed.
 *
 * Revision 1.28  2008/12/12 15:16:04  slava
 * o added BUFFER_Trim function
 *
 * Revision 1.27  2005/10/19 21:11:09  slava
 * o added BUFFER_Move function
 *
 * Revision 1.26  2004/07/19 22:49:30  slava
 * o added BUFFER_Unput function
 *
 * Revision 1.25  2004/04/01 19:03:52  slava
 * o added BUFFER_CreateWrite and BUFFER_InitWrite
 *
 * Revision 1.24  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.23  2002/09/15 19:35:34  slava
 * o before defining ntohs, ntohl, etc. macros, first undefine them. otherwise
 *   we may get compilation warnings on some platforms that also define those
 *   as macros
 *
 * Revision 1.22  2002/08/23 03:43:25  slava
 * o added BUFFER_PushBack
 *
 * Revision 1.21  2002/08/12 05:13:27  slava
 * o use (void) rather than () to declare functions that have
 *   no arguments. The problem with () is that compilers interpret
 *   it as an *unspecified* argument list, but (void) clearly
 *   indicates that function takes no arguments. Improves compile
 *   time error checking
 *
 * Revision 1.20  2002/06/08 17:02:11  slava
 * o support for readonly buffers
 *
 * Revision 1.19  2002/05/22 04:13:30  slava
 * o Buffer of binary data. Function are provided to store the data in
 *   or retrieve from the buffer in FIFO (first in - first out) manner.
 *   The buffer can have internal storage of fixed or variable (possibly,
 *   limited) size. Buffer may automatically resize itself if necessary,
 *   but only if it's allowed to do so.
 *
 *
 * Local Variables:
 * mode:C
 * c-basic-offset:4
 * indent-tabs-mode: nil
 * End:
 */
