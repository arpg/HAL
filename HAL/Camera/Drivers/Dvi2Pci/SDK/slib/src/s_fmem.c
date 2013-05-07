/*
 * $Id: s_fmem.c,v 1.16 2009/12/26 12:21:32 slava Exp $
 *
 * Copyright (C) 2001-2009 by Slava Monich
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

#include "s_fio.h"
#include "s_mem.h"

/*==========================================================================*
 *              I N - M E M O R Y    I O
 *==========================================================================*/

typedef struct _MemFile {
    File file;          /* shared File structure */
    Buffer * buf;       /* input/output buffer */
    int mflags;         /* flags, see below */

#define MEM_EOF  0x0001 /* end of file on input stream */
#define MEM_ERR  0x0002 /* output buffer full */
#define MEM_IN   0x0004 /* can read from the buffer */
#define MEM_OUT  0x0008 /* can write to the buffer */
#define MEM_BUF  0x0010 /* we deallocate the buffer */

} MemFile;

STATIC int  MemFileRead P_((File * f, void * buf, int len));
STATIC int  MemFileWrite P_((File * f, const void * buf, int len));
STATIC int  MemFileSkip P_((File * f, int skip));
STATIC Bool MemFileFlush P_((File * f));
STATIC Bool MemFileEof P_((File * f));
STATIC void MemFileClose P_((File * f));
STATIC void MemFileFree P_((File * f));

/*
 * I/O type definition
 */
STATIC const FileIO MemFileIO = {
    NULL                /* open     */,
    NULL                /* reopen   */,
    NULL                /* setparam */,
    MemFileRead         /* read     */,
    MemFileWrite        /* write    */,
    MemFileSkip         /* skip     */,
    MemFileFlush        /* flush    */,
    MemFileEof          /* eof      */,
    NULL                /* fd       */,
    NULL                /* target   */,
    NULL                /* detach   */,
    MemFileClose        /* close    */,
    MemFileFree         /* free     */,
    0                   /* flags    */
};

/*
 * Implementation
 */
STATIC MemFile * MemFileCast(File * f) 
{
    ASSERT(f);
    if (f) {
        ASSERT(f->io == &MemFileIO);
        if (f->io == &MemFileIO) {
            return CAST(f,MemFile,file);
        }
    }
    return NULL;
}

STATIC const MemFile * MemFileCastC(const File * f) 
{
    ASSERT(f);
    if (f) {
        ASSERT(f->io == &MemFileIO);
        if (f->io == &MemFileIO) {
            return CAST(f,MemFile,file);
        }
    }
    return NULL;
}

STATIC int MemFileRead(File * f, void * buf, int len)
{
    int nbytes = -1;
    MemFile * m = MemFileCast(f);
    if (m && (m->mflags & MEM_IN)) {
        if (!(m->mflags & MEM_EOF)) {
            nbytes = (int)BUFFER_Get(m->buf, buf, len);
            if (!nbytes && BUFFER_Size(m->buf) == 0) {
                m->mflags |= MEM_EOF;
            }
        }
    }
    return nbytes;
}

STATIC int MemFileSkip(File * f, int skip)
{
    return MemFileRead(f, NULL, skip);
}

STATIC int MemFileWrite(File * f, const void * buf, int len) 
{
    int nbytes = -1;
    MemFile * m = MemFileCast(f);
    if (m && (m->mflags & MEM_OUT)) {
        if (!(m->mflags & MEM_ERR)) {
            nbytes = (int)BUFFER_Put(m->buf, buf, len, True);
            if (!nbytes && len > 0) {
                nbytes = -1;
                m->mflags |= MEM_ERR;
            } else {
                m->mflags &= ~MEM_EOF;
            }
        }
    }
    return nbytes;
}

STATIC Bool MemFileFlush(File * f) 
{
    MemFile * m = MemFileCast(f);
    return BoolValue(m && (m->mflags & MEM_OUT));
}

STATIC Bool MemFileEof(File * f) 
{
    MemFile * m = MemFileCast(f);
    return BoolValue(m && (!(m->mflags & MEM_IN) || (m->mflags & MEM_EOF)));
}

STATIC void MemFileClose(File * f) 
{
    RELEASE_UNREF(f);
    ASSERT(MemFileCast(f));
}

STATIC void MemFileFree(File * f) 
{
    MemFile * m = MemFileCast(f);
    if (m) {
        ASSERT(!(f->flags & FILE_IS_OPEN));
        if (m->mflags & MEM_BUF) BUFFER_Delete(m->buf);
        MEM_Free(m);
    }
}

/**
 * Creates the default in-memory I/O context. Allows writing to and 
 * reading from the internal buffer. Has no limit on the amount of
 * allocated data.
 */
File * FILE_Mem()
{
    MemFile * m = MEM_New(MemFile);
    if (m) {
        memset(m, 0, sizeof(*m));
        m->buf = BUFFER_Create();
        if (m->buf) {
            m->mflags = MEM_IN | MEM_OUT | MEM_BUF;
            if (FILE_Init(&m->file, TEXT("memio"), False, &MemFileIO)) {
                return &m->file;
            }
            BUFFER_Delete(m->buf);
        }
        MEM_Free(m);
    }
    return NULL;
}

/**
 * Creates a context for reading from the specified buffer.
 */
File * FILE_MemIn(const void * data, size_t size) 
{
    MemFile * m = MEM_New(MemFile);
    if (m) {
        memset(m, 0, sizeof(*m));
        m->buf = BUFFER_Create2((void*)data, size);
        if (m->buf) {
            m->mflags = MEM_IN | MEM_BUF;
            m->buf->flags &= ~BUFFER_OWN_DATA;
            if (FILE_Init(&m->file, TEXT("memin"), False, &MemFileIO)) {
                return &m->file;
            }
            BUFFER_Delete(m->buf);
        }
        MEM_Free(m);
    }
    return NULL;
}

/**
 * Creates a context for writing to the internal buffer.
 * If maxsize is negative, there's no limit.
 */
File * FILE_MemOut(int maxsize)
{
    MemFile * m = MEM_New(MemFile);
    if (m) {
        memset(m, 0, sizeof(*m));
        m->buf = BUFFER_Create();
        if (m->buf) {
            m->mflags = MEM_OUT | MEM_BUF;
            m->buf->maxsiz = maxsize;
            if (FILE_Init(&m->file, TEXT("memout"), False, &MemFileIO)) {
                return &m->file;
            }
            BUFFER_Delete(m->buf);
        }
        MEM_Free(m);
    }
    return NULL;
}

/**
 * Creates a context for writing to the specified buffer.
 * Will not resize the buffer.
 */
File * FILE_MemOut2(void * data, size_t size) 
{
    MemFile * m = MEM_New(MemFile);
    if (m) {
        memset(m, 0, sizeof(*m));
        m->buf = BUFFER_Create2(data, size);
        if (m->buf) {
            m->mflags = MEM_OUT | MEM_BUF;
            m->buf->maxsiz = size;  /* never resize */
            m->buf->flags &= ~BUFFER_OWN_DATA;
            BUFFER_Clear(m->buf);
            if (FILE_Init(&m->file, TEXT("memout"), False, &MemFileIO)) {
                return &m->file;
            }
            BUFFER_Delete(m->buf);
        }
        MEM_Free(m);
    }
    return NULL;
}

/**
 * Creates an in-memory I/O context associated with an external buffer.
 * Does not deallocate the buffer if freeBuf is False. Allows both reads 
 * and writes.
 */
File * FILE_MemBuf(Buffer * buf, Bool freeBuf)
{
    MemFile * m = MEM_New(MemFile);
    ASSERT(buf);
    if (m) {
        memset(m, 0, sizeof(*m));
        m->buf = buf;
        m->mflags = MEM_IN | MEM_OUT;
        if (freeBuf) m->mflags |= MEM_BUF;
        if (FILE_Init(&m->file, TEXT("membuf"), False, &MemFileIO)) {
            return &m->file;
        }
        MEM_Free(m);
    }
    return NULL;
}

/**
 * Returns pointer to the in-memory data, NULL if this is not a MemFile
 */
void * FILE_MemData(File * f)
{
    MemFile * m = MemFileCast(f);
    if (m) {
        return BUFFER_Access(m->buf);
    } else {
        return NULL;
    }
}

/**
 * Returns the size of the data stored in the MemFile's buffer,
 * zero if this is not a MemFile
 */
size_t FILE_MemSize(const File * f)
{
    const MemFile * m = MemFileCastC(f);
    if (m) {
        return BUFFER_Size(m->buf);
    } else {
        return -1;
    }
}

/**
 * Clears the internal buffer
 */
void FILE_MemClear(File * f)
{
    MemFile * m = MemFileCast(f);
    if (m) {
        BUFFER_Clear(m->buf);
        m->mflags &= ~(MEM_ERR|MEM_EOF);
    }
}

/**
 * Checks if this file is a memory I/O file
 */
Bool FILE_IsMem(const File * f)
{
    return BoolValue(f && f->io == &MemFileIO);
}

/*
 * HISTORY:
 *
 * $Log: s_fmem.c,v $
 * Revision 1.16  2009/12/26 12:21:32  slava
 * o working on 64-bit issues
 *
 * Revision 1.15  2009/10/08 14:32:11  slava
 * o added FILE_Fd() and FILE_TargetFd() functions
 *
 * Revision 1.14  2009/04/09 21:54:57  slava
 * o use size_t instead of int where it's appropriate
 *
 * Revision 1.13  2008/12/19 15:27:26  slava
 * o need to empty the buffer in FILE_MemOut2
 *
 * Revision 1.12  2008/09/01 12:08:15  slava
 * o replaced FileIO::fileIO field with flags. Added FIO_BLOCKING_READ flag
 *   which marks I/O methods whose read callback can block before the end of
 *   stream is reached (i.e. socket I/O).
 *
 * Revision 1.11  2008/09/01 10:57:21  slava
 * o added 'skip' I/O callback which can be implemented by those I/O methods
 *   which can efficiently skip input data without actually reading and
 *   copying anything. Currently, such callback is only implemented for
 *   in-memory I/O, but it can also be done for file I/O using fseek.
 *
 * Revision 1.10  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.9  2004/04/08 01:44:18  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.8  2004/03/25 19:06:00  slava
 * o fixed a few pedantic compilation warnings
 *
 * Revision 1.7  2003/09/10 00:23:46  slava
 * o added FILE_MemClear and FILE_IsMem functions
 * o clear MEM_EOF flag on write
 *
 * Revision 1.6  2003/06/05 03:55:24  slava
 * o handle reads and writes of zero size
 *
 * Revision 1.5  2003/05/26 18:30:57  slava
 * o implemented FILE_MemData and FILE_MemSize functions
 *
 * Revision 1.4  2003/05/21 00:11:03  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.3  2003/01/20 19:59:35  slava
 * o added missing I/O handler
 *
 * Revision 1.2  2002/12/30 21:42:21  slava
 * o added fileIO field to the FileIO structure. This field is set to
 *   True if the File stream performs file based I/O (as opposed to socket
 *   or in-memory I/O)
 *
 * Revision 1.1  2002/05/22 04:22:33  slava
 * o in-memory I/O
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
