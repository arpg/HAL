/*
 * $Id: s_fsplit.c,v 1.9 2009/10/08 14:32:11 slava Exp $
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
 *              S P L I T T E R
 *==========================================================================*/

typedef struct _SplitFile {
    File file;      /* shared File structure */
    int n;          /* number target streams */
    File * f[1];    /* streams to redirect to */
} SplitFile;

STATIC int  SplitFileWrite P_((File * f, const void * buf, int len));
STATIC Bool SplitFileFlush P_((File * f));
STATIC Bool SplitFileEof P_((File * f));
STATIC void SplitFileDetach P_((File * f));
STATIC void SplitFileClose P_((File * f));
STATIC void SplitFileFree P_((File * f));

/*
 * I/O type definition
 */
STATIC const FileIO SplitFileIO = {
    NULL                /* open     */,
    NULL                /* reopen   */,
    NULL                /* setparam */,
    NULL                /* read     */,
    SplitFileWrite      /* write    */,
    NULL                /* skip     */,
    SplitFileFlush      /* flush    */,
    SplitFileEof        /* eof      */,
    NULL                /* fd       */,
    NULL                /* target   */,
    SplitFileDetach     /* detach   */,
    SplitFileClose      /* close    */,
    SplitFileFree       /* free     */,
    0                   /* flags    */
};

/*
 * Implementation
 */
STATIC SplitFile * SplitFileCast(File * f)
{
    ASSERT(f);
    if (f) {
        ASSERT(f->io == &SplitFileIO);
        if (f->io == &SplitFileIO) {
            return CAST(f,SplitFile,file);
        }
    }
    return NULL;
}

STATIC int SplitFileWrite(File * f, const void * buf, int len)
{
    SplitFile * sf = SplitFileCast(f);
    if (sf) {
        int i;
        int written = len;
        for (i=0; i<sf->n; i++) {
            File * ff = sf->f[i];
            if (CAN_WRITE(ff)) {
                int n = (*(ff->io->write))(ff, buf, len);
                if (n < written) written = n;
            }
        }
        return written;
    }
    return (-1);
}

STATIC Bool SplitFileFlush(File * f)
{
    SplitFile * sf = SplitFileCast(f);
    if (sf) {
        int i;
        Bool ok = True;
        for (i=0; i<sf->n; i++) {
            File * ff = sf->f[i];
            if (!(*(ff->io->flush))(ff)) ok = False;
        }
        return ok;
    }
    return False;
}

STATIC Bool SplitFileEof(File * f)
{
    SplitFile * sf = SplitFileCast(f);
    if (sf) {
        int i;
        Bool eof = False;
        for (i=0; i<sf->n; i++) {
            File * ff = sf->f[i];
            if ((*(ff->io->eof))(ff)) eof = True;
        }
        return eof;
    }
    return False;
}

STATIC void SplitFileDetach(File * f)
{
    SplitFile * sf = SplitFileCast(f);
    if (sf) sf->n = 0;
}

STATIC void SplitFileClose(File * f)
{
    SplitFile * sf = SplitFileCast(f);
    if (sf) {
        int i;
        for (i=0; i<sf->n; i++) {
            FILE_Finish(sf->f[i]);
        }
    }
}

STATIC void SplitFileFree(File * f)
{
    SplitFile * sf = SplitFileCast(f);
    if (sf) {
        int i;
        for (i=0; i<sf->n; i++) {
            FILE_Close(sf->f[i]);
        }
        sf->n = 0;
        ASSERT(!(f->flags & FILE_IS_OPEN));
        MEM_Free(sf);
    }
}

/**
 * Creates a splitter that splits one stream into two.
 * If name is NULL, takes the name of the first file.
 */
File * FILE_Split2(Str name, File * f1, File * f2)
{
    File * f[2];
    f[0] = f1;
    f[1] = f2;
    return FILE_Split(name, f, 2);
}

/**
 * Creates a splitter that splits one stream into many. 
 * If name is NULL, takes the name of the first file.
 */
File * FILE_Split(Str name, File * f[], int n)
{
    ASSERT(n > 0);
    if (n > 0) {
        size_t size = sizeof(SplitFile) + sizeof(File*)*(n-1);
        SplitFile * sf = (SplitFile*)MEM_Alloc(size);
        if (sf) {
            int i;
            memset(sf, 0, size);
            for (i=0; i<n; i++) {
                ASSERT(f[i]);
                if (f[i]) {
                    sf->f[sf->n++] = f[i];
                }
            }
            if (!name) name = FILE_Name(f[0]);
            if (FILE_Init(&sf->file, name, True, &SplitFileIO)) {
                return &sf->file;
            }
        }
    }
    return NULL;
}

/*
 * HISTORY:
 *
 * $Log: s_fsplit.c,v $
 * Revision 1.9  2009/10/08 14:32:11  slava
 * o added FILE_Fd() and FILE_TargetFd() functions
 *
 * Revision 1.8  2008/09/01 12:08:15  slava
 * o replaced FileIO::fileIO field with flags. Added FIO_BLOCKING_READ flag
 *   which marks I/O methods whose read callback can block before the end of
 *   stream is reached (i.e. socket I/O).
 *
 * Revision 1.7  2008/09/01 10:57:21  slava
 * o added 'skip' I/O callback which can be implemented by those I/O methods
 *   which can efficiently skip input data without actually reading and
 *   copying anything. Currently, such callback is only implemented for
 *   in-memory I/O, but it can also be done for file I/O using fseek.
 *
 * Revision 1.6  2004/03/25 19:06:00  slava
 * o fixed a few pedantic compilation warnings
 *
 * Revision 1.5  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
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
 * Revision 1.2  2002/08/27 11:52:02  slava
 * o some reformatting
 *
 * Revision 1.1  2001/12/23 21:17:00  slava
 * o moved each file I/O implemenetation into a separate file. This prevents
 *   unnecessary linkage with zlib and socket library and makes executables
 *   smaller. Most linkers are not very good in removing dead references
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
