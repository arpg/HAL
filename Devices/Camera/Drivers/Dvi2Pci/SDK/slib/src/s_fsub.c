/*
 * $Id: s_fsub.c,v 1.6 2009/12/26 12:21:32 slava Exp $
 *
 * Copyright (C) 2006-2009 by Slava Monich
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

#include "s_util.h"
#include "s_fio.h"
#include "s_mem.h"

/*==========================================================================*
 *              S U B   S T R E A M
 *==========================================================================*/

typedef struct _SubFile {
    File file;      /* shared File structure */
    File * target;  /* the file actually performing the I/O */
    size_t maxread; /* maximum number of bytes to read from the stream */
    Bool eof;       /* True if each next read will return (-1) */
} SubFile;

STATIC Bool   SubFileReopen P_((File * f, Str path, const char * mode));
STATIC int    SubFileRead   P_((File * f, void * buf, int len));
STATIC int    SubFileWrite  P_((File * f, const void * buf, int len));
STATIC Bool   SubFileEof    P_((File * f));
STATIC Bool   SubFileFlush  P_((File * f));
STATIC File * SubFileTarget P_((File * f));
STATIC void   SubFileDetach P_((File * f));
STATIC void   SubFileFree   P_((File * f));

/*
 * Table of I/O handlers
 */
STATIC const FileIO SubFileIO = {
    NULL                /* open     */,
    SubFileReopen       /* reopen   */,
    NULL                /* setparam */,
    SubFileRead         /* read     */,
    SubFileWrite        /* write    */,
    NULL                /* skip     */,
    SubFileFlush        /* flush    */,
    SubFileEof          /* eof      */,
    NULL                /* fd       */,
    SubFileTarget       /* target   */,
    SubFileDetach       /* detach   */,
    SubFileDetach       /* close    */,
    SubFileFree         /* free     */,
    0                   /* flags    */
};

/*
 * I/O handlers
 */
STATIC SubFile * SubFileCast(File * f)
{
    ASSERT(f);
    if (f) {
        ASSERT(f->io == &SubFileIO);
        if (f->io == &SubFileIO) {
            SubFile * s = CAST(f,SubFile,file);
            ASSERT(s->maxread >= s->file.bytesRead);
            return s;
        }
    }
    return NULL;
}

STATIC Bool SubFileReopen(File * f, Str path, const char * mode)
{
    SubFile * s = SubFileCast(f);
    if (s) {
        s->eof = False;
        return FILE_Reopen(s->target, path, mode);
    }
    return False;
}

STATIC int SubFileRead(File * f, void * buf, int len)
{
    int nbytes = -1;
    SubFile * s  = SubFileCast(f);
    if (s && !s->eof) {
        if (s->maxread == s->file.bytesRead) {
            s->eof = True;
            nbytes = 0;
        } else {
            if ((size_t)len > (s->maxread - s->file.bytesRead)) {
                len = (int)(s->maxread - s->file.bytesRead);
            }
            nbytes = FILE_Read(s->target, buf, len);
            if (nbytes < len) s->eof = True;
        }
    }
    return nbytes;
}

STATIC int SubFileWrite(File * f, const void * buf, int len)
{
    SubFile * s  = SubFileCast(f);
    return (s ? FILE_Write(s->target, buf, len) : (-1));
}

STATIC Bool SubFileEof(File * f)
{
    SubFile * s  = SubFileCast(f);
    if (s) {
        if (s->eof || (s->maxread == s->file.bytesRead)) {
            return True;
        } else {
            return FILE_Eof(s->target);
        }
    } else {
        return True;
    }
}

STATIC Bool SubFileFlush(File * f)
{
    SubFile * s = SubFileCast(f);
    return (s ? FILE_Flush(s->target) : False);
}

STATIC File * SubFileTarget(File * f)
{
    SubFile * s = SubFileCast(f);
    return (s ? s->target : NULL);
}

STATIC void SubFileDetach(File * f)
{
    /* NOTE: this callback handles both detach() and close() */
    SubFile * s  = SubFileCast(f);
    if (s) s->target = NULL;
}

STATIC void SubFileFree(File * f)
{
    SubFile * s  = SubFileCast(f);
    if (s) {
        MEM_Free(s);
    }
}

/**
 * Create a File that reads no more than specified number of bytes from
 * the target stream. May simplify parsing of a stream consisting of well
 * defined blocks of data of known size. Writes are passed transparently to
 * the underlying stream. Closing this file doesn't close the underlying
 * stream.
 */
File * FILE_SubStream(File * f, size_t maxread)
{
    SubFile * s = MEM_New(SubFile);
    if (s) {
        memset(s, 0, sizeof(*s));
        s->target = f;
        s->maxread = maxread;
        if (FILE_Init(&s->file, NULL, True, &SubFileIO)) {
            return &s->file;
        }
        MEM_Free(s);
    }
    return NULL;
}

/**
 * Resets the substream object and configure it to read no more than maxread
 * bytes from the current point.
 */
Bool FSUB_Reset(File * sub, size_t maxread)
{
    if (sub && sub->io == &SubFileIO) {
        SubFile * s = CAST(sub,SubFile,file);
        s->file.bytesRead = 0;
        s->file.bytesWritten = 0;
        s->maxread = maxread;
        s->eof = False;
        return True;
    }
    return False;
}

/**
 * Skips remaining bytes from the underlying stream to make sure that we have
 * read exactly maxread bytes from the last reset.
 */
Bool FSUB_SkipRest(File * sub)
{
    if (sub && sub->io == &SubFileIO) {
        SubFile * s = CAST(sub,SubFile,file);
        ASSERT(s->maxread >= s->file.bytesRead);
        if (s->maxread > s->file.bytesRead) {
            size_t skip = s->maxread - s->file.bytesRead;
            return (FILE_Skip(s->target, skip) == skip);
        }
        return True;
    }
    return False;
}

/*
 * HISTORY:
 *
 * $Log: s_fsub.c,v $
 * Revision 1.6  2009/12/26 12:21:32  slava
 * o working on 64-bit issues
 *
 * Revision 1.5  2009/10/08 14:32:11  slava
 * o added FILE_Fd() and FILE_TargetFd() functions
 *
 * Revision 1.4  2008/09/01 12:08:15  slava
 * o replaced FileIO::fileIO field with flags. Added FIO_BLOCKING_READ flag
 *   which marks I/O methods whose read callback can block before the end of
 *   stream is reached (i.e. socket I/O).
 *
 * Revision 1.3  2008/09/01 10:57:21  slava
 * o added 'skip' I/O callback which can be implemented by those I/O methods
 *   which can efficiently skip input data without actually reading and
 *   copying anything. Currently, such callback is only implemented for
 *   in-memory I/O, but it can also be done for file I/O using fseek.
 *
 * Revision 1.2  2008/09/01 09:40:16  slava
 * o added FSUB_Reset and FSUB_SkipRest functions that extend the functionality
 *   of "substreams""
 *
 * Revision 1.1  2006/10/13 23:35:37  slava
 * o added "sub-stream" I/O which reads no more than specified number of bytes
 *   from another stream. It can be created by FILE_SubStream() function.
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
