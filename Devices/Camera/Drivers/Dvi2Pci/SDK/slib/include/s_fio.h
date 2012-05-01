/*
 * $Id: s_fio.h,v 1.18 2010/12/19 15:57:19 slava Exp $
 *
 * Copyright (C) 2001-2010 by Slava Monich
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

#ifndef _SLAVA_FILE_IO_H_
#define _SLAVA_FILE_IO_H_ 1

#include "s_file.h"
#include "s_strbuf.h"

/*
 * "private" flags
 */
#define FILE_IS_OPEN      0x1000    /* set until closed or detached */
#define FILE_IS_ATTACHED  0x2000    /* attached to existing low-level file */

/* The value returned by FILE_Getc if it reaches the end of file */
#ifndef EOF
#  define EOF (-1)
#endif /* EOF */

/* assert that these are not public */
COMPILE_ASSERT((FILE_IS_OPEN & FILE_PUBLIC_FLAGS) == 0)
COMPILE_ASSERT((FILE_IS_ATTACHED & FILE_PUBLIC_FLAGS) == 0)

/* Max length of the open mode string (even 7 is too much...) */
#define MAX_MODE_LEN 7

/*
 * File functions. Some notes:
 *
 * 1. each FileOpen allocates File structure (with some additional fields) 
 *    FileClose closes the file (but does not deallocate the context) and 
 *    FileFree finally deallocates the context. The Close functionality
 *    is separated from Free in order to implement Finish semantics.
 *
 * 2. FileOpen only initializes its private portion of the file context.
 *    The shared portion (i.e. File structure) is initialized by FILE_Open.
 *    FileOpen may, however, initialize the whole thing to zero.
 *
 * 3. FileWrite and FileRead return number of bytes written or read. 
 *    In case of error they return -1
 *
 * 4. it's guaranteed that FileFree is the the only call that may occur 
 *    after FileClose or FileDetach
 *
 * 5. it's guaranteed that FileClose or FileDetach will always be invoked 
 *    before FileFree and that it happens no more than once per context
 */
typedef File * (*FileOpen)     P_((Str path, const char * mode));
typedef Bool   (*FileReopen)   P_((File * f, Str path, const char * mode));
typedef Bool   (*FileSetParam) P_((File * f, Str name, void * value));
typedef int    (*FileRead)     P_((File * f, void * buf, int skip));
typedef int    (*FileWrite)    P_((File * f, const void * buf, int len));
typedef int    (*FileSkip)     P_((File * f, int len));
typedef Bool   (*FileFlush)    P_((File * f));
typedef Bool   (*FileEof)      P_((File * f));
typedef int    (*FileFd)       P_((File * f));
typedef File * (*FileTarget)   P_((File * f));
typedef void   (*FileDetach)   P_((File * f));
typedef void   (*FileClose)    P_((File * f));
typedef void   (*FileFree)     P_((File * f));

/*
 * A set of handlers that perform I/O
 */
struct _FileIO {
    FileOpen     open;          /* open the stream */
    FileReopen   reopen;        /* reopen the stream */
    FileSetParam setparam;      /* set stream parameters */
    FileRead     read;          /* read bytes */
    FileWrite    write;         /* write bytes */
    FileSkip     skip;          /* skip bytes */
    FileFlush    flush;         /* flush output buffer */
    FileEof      eof;           /* test for end-of-file condition */
    FileFd       fd;            /* returns the underlying fd, -1 if none */
    FileTarget   target;        /* returns the File where I/O is redirected */
    FileDetach   detach;        /* detach from the lower level stream */
    FileClose    close;         /* close the stream */
    FileFree     free;          /* deallocate the stream context */
    int          flags;         /* flags, see below: */

#define FIO_FILE_BASED    0x01  /* set if this is a file based I/O */
};

/*
 * A common context associated with an open file.
 */
struct _File {
    IODesc io;                  /* I/O functions */
    Char * name;                /* the file name */
    StrBuf buf;                 /* temporary buffer for printf */
    I8u    pushBack[8];         /* minimal pushback buffer */
    I16u   pushed;              /* number of bytes pushed back */
    I16u   flags;               /* flags */
    size_t bytesRead;           /* number of bytes read from this stream */
    size_t bytesWritten;        /* number of bytes written to this stream */
};

#define CAN_READ(_f) \
    ((_f) && \
    ((_f)->flags & FILE_IS_OPEN) && \
    ((_f)->flags & FILE_CAN_READ) && \
     (_f)->io->read) 

#define CAN_WRITE(_f) \
    ((_f) && \
    ((_f)->flags & FILE_IS_OPEN) && \
    ((_f)->flags & FILE_CAN_WRITE) && \
     (_f)->io->write) 

/* 
 * Internal functions 
 */
extern Bool FILE_Init P_((File * f, Str path, Bool attach, IODesc io));
extern void FILE_Destroy P_((File * f));

#endif /* _SLAVA_FILE_IO_H_ */

/*
 * HISTORY:
 *
 * $Log: s_fio.h,v $
 * Revision 1.18  2010/12/19 15:57:19  slava
 * o increased the size of the bushback buffer to 8
 *
 * Revision 1.17  2009/10/08 14:32:11  slava
 * o added FILE_Fd() and FILE_TargetFd() functions
 *
 * Revision 1.16  2008/09/05 08:43:33  slava
 * o added MAX_MODE_LEN back. It's used by the UNICODE build
 *
 * Revision 1.15  2008/09/03 09:37:02  slava
 * o removed static FIO_BLOCKING_READ flag and added FILE_CAN_BLOCK flag that
 *   can be set on individual streams. This is a lot more flexible. Added
 *   FILE_CanBlock function.
 *
 * Revision 1.14  2008/09/01 12:08:15  slava
 * o replaced FileIO::fileIO field with flags. Added FIO_BLOCKING_READ flag
 *   which marks I/O methods whose read callback can block before the end of
 *   stream is reached (i.e. socket I/O).
 *
 * Revision 1.13  2008/09/01 10:57:21  slava
 * o added 'skip' I/O callback which can be implemented by those I/O methods
 *   which can efficiently skip input data without actually reading and
 *   copying anything. Currently, such callback is only implemented for
 *   in-memory I/O, but it can also be done for file I/O using fseek.
 *
 * Revision 1.12  2007/02/08 17:43:59  slava
 * o added internal function FILE_Destroy
 *
 * Revision 1.11  2006/10/13 16:14:22  slava
 * o count number of bytes written to and read from the stream
 *
 * Revision 1.10  2006/10/12 16:23:24  slava
 * o made ZipIO static
 *
 * Revision 1.9  2006/10/12 16:16:58  slava
 * o made WrapIO static
 *
 * Revision 1.8  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.7  2005/02/19 02:37:10  slava
 * o include SLIB headers as user includes (i.e. "s_lib.h") rather than
 *   system includes (i.e. <s_lib.h>). This helps with generation of
 *   dependencies
 *
 * Revision 1.6  2003/11/30 02:49:58  slava
 * o port to Linux kernel mode environment
 *
 * Revision 1.5  2003/06/05 02:42:25  slava
 * o added FILE_PushBack and FILE_Ungetc functions
 *
 * Revision 1.4  2003/01/08 17:15:43  slava
 * o removed ZIP_GetTarget and WRAP_GetTarget functions, replaced them
 *   with generic FILE_Target
 *
 * Revision 1.3  2002/12/30 21:42:21  slava
 * o added fileIO field to the FileIO structure. This field is set to
 *   True if the File stream performs file based I/O (as opposed to socket
 *   or in-memory I/O)
 *
 * Revision 1.2  2002/05/22 04:19:10  slava
 * o fixed include statements after s_sbuf.h was renamed into s_strbuf.h
 *
 * Revision 1.1  2001/12/23 21:17:00  slava
 * o moved each file I/O implemenetation into a separate file. This prevents
 *   unnecessary linkage with zlib and socket library and makes executables
 *   smaller. Most linkers are not very good in removing dead references
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
