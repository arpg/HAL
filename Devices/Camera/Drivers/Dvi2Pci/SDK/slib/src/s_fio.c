/*
 * $Id: s_fio.c,v 1.11 2009/10/08 14:32:11 slava Exp $
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

#include "s_buf.h"
#include "s_fio.h"
#include "s_mem.h"

/*==========================================================================*
 *              P L A I N     F I L E    I O
 *==========================================================================*/

#undef PlainFile
typedef struct _PlainFile {
    File file;  /* shared File structure */
    FILE * f;   /* a C runtime file structure */
} PlainFile;

STATIC PlainFile * PlainFileCast(File * f)
{
    ASSERT(f);
    if (f) {
        ASSERT(f->io == &PlainFileIO);
        if (f->io == &PlainFileIO) {
            return CAST(f,PlainFile,file);
        }
    }
    return NULL;
}

STATIC File * PlainFileOpen(Str path, const char * mode)
{
    FILE * ff;
    PlainFile * pf = NULL; /* assume failure */

#undef _mode
#ifdef UNICODE
    const char * c = mode;
    int i = 0;
    wchar_t _m[MAX_MODE_LEN+1];
    while (*c && i<MAX_MODE_LEN) _m[i++] = *c++;
    ASSERT(i<MAX_MODE_LEN);
    _m[i] = 0;
#  define _mode _m
#else /* UNICODE */
#  define _mode mode
#endif /* UNICODE */

    ff = Fopen(path, _mode);
    if (ff) {
        pf = MEM_New(PlainFile);
        if (pf) {
            memset(pf, 0, sizeof(*pf));
            pf->f = ff;
        } else {
            fclose(ff);
        }
    }
    return (pf ? &pf->file : NULL);
}

#ifdef _WIN32_WCE
#  define PlainFileReopen NULL  /* not implemented on CE */
#else /* !_WIN32_WCE */
STATIC Bool PlainFileReopen(File * f, Str path, const char * mode)
{
    PlainFile * pf = PlainFileCast(f);
    if (pf) {

#undef _mode
#ifdef UNICODE
        const char * c = mode;
        int i = 0;
        wchar_t _m[MAX_MODE_LEN+1];
        while (*c && i<MAX_MODE_LEN) _m[i++] = *c++;
        ASSERT(i<MAX_MODE_LEN);
        _m[i] = 0;
#  define _mode _m
#else /* UNICODE */
#  define _mode mode
#endif /* UNICODE */

        pf->f = Freopen(path, _mode, pf->f);
        return BoolValue(pf->f != NULL);
    }
    return False;
}
#endif /* !_WIN32_WCE */

STATIC int PlainFileRead(File * f, void * buf, int len)
{
    PlainFile * pf = PlainFileCast(f);
    if (pf) {
        size_t nbytes = fread(buf, 1, len, pf->f);
        return (ferror(pf->f) ? (-1) : (int)nbytes);
    }
    return (-1);
}

STATIC int PlainFileWrite(File * f, const void * buf, int len)
{
    PlainFile * pf = PlainFileCast(f);
    if (pf) {
        size_t nbytes = fwrite(buf, 1, len, pf->f);
        return (ferror(pf->f) ? (-1) : (int)nbytes);
    }
    return (-1);
}

STATIC Bool PlainFileFlush(File * f)
{
    PlainFile * pf = PlainFileCast(f);
    return BoolValue(pf && (fflush(pf->f) != EOF));
}

STATIC Bool PlainFileEof(File * f)
{
    PlainFile * pf = PlainFileCast(f);
    return BoolValue(pf && feof(pf->f));
}

STATIC int PlainFileFd(File * f)
{
    PlainFile * pf = PlainFileCast(f);
    return (pf ? fileno(pf->f) : -1);
}

STATIC void PlainFileDetach(File * f)
{
    PlainFile * pf = PlainFileCast(f);
    if (pf) pf->f = NULL;
}

STATIC void PlainFileClose(File * f)
{
    PlainFile * pf = PlainFileCast(f);
    if (pf) {
        if (pf->f) {
            fclose(pf->f);
            pf->f = NULL;
        }
    }
}

STATIC void PlainFileFree(File * f)
{
    PlainFile * pf = PlainFileCast(f);
    if (pf) {
        ASSERT(!(f->flags & FILE_IS_OPEN));
        MEM_Free(pf);
    }
}

/**
 * Creates a File object for the given C runtime FILE structure.
 */
File * FILE_AttachToFile(FILE * file, Str name)
{
    if (file) {
        PlainFile * pf = MEM_New(PlainFile);
        if (pf) {
            memset(pf, 0, sizeof(*pf));
            pf->f = file;
            if (FILE_Init(&pf->file, name, True, &PlainFileIO)) {
                return &pf->file;
            }
            MEM_Free(pf);
        }
    }
    return NULL;
}

/*
 * A set of handlers that perform file I/O
 */
const FileIO PlainFileIO = {
    PlainFileOpen       /* open     */,
    PlainFileReopen     /* reopen   */,
    NULL                /* setparam */,
    PlainFileRead       /* read     */,
    PlainFileWrite      /* write    */,
    NULL                /* skip     */,
    PlainFileFlush      /* flush    */,
    PlainFileEof        /* eof      */,
    PlainFileFd         /* fd       */,
    NULL                /* target   */,
    PlainFileDetach     /* detach   */,
    PlainFileClose      /* close    */,
    PlainFileFree       /* free     */,
    FIO_FILE_BASED      /* flags    */
};

/*
 * HISTORY:
 *
 * $Log: s_fio.c,v $
 * Revision 1.11  2009/10/08 14:32:11  slava
 * o added FILE_Fd() and FILE_TargetFd() functions
 *
 * Revision 1.10  2008/09/01 12:08:15  slava
 * o replaced FileIO::fileIO field with flags. Added FIO_BLOCKING_READ flag
 *   which marks I/O methods whose read callback can block before the end of
 *   stream is reached (i.e. socket I/O).
 *
 * Revision 1.9  2008/09/01 10:57:21  slava
 * o added 'skip' I/O callback which can be implemented by those I/O methods
 *   which can efficiently skip input data without actually reading and
 *   copying anything. Currently, such callback is only implemented for
 *   in-memory I/O, but it can also be done for file I/O using fseek.
 *
 * Revision 1.8  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.7  2004/04/08 01:06:15  slava
 * o fixed return value from PlainFileFlush
 *
 * Revision 1.6  2003/05/21 00:11:03  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.5  2003/01/08 17:15:43  slava
 * o removed ZIP_GetTarget and WRAP_GetTarget functions, replaced them
 *   with generic FILE_Target
 *
 * Revision 1.4  2002/12/30 21:42:21  slava
 * o added fileIO field to the FileIO structure. This field is set to
 *   True if the File stream performs file based I/O (as opposed to socket
 *   or in-memory I/O)
 *
 * Revision 1.3  2002/08/27 11:52:02  slava
 * o some reformatting
 *
 * Revision 1.2  2002/06/17 00:13:06  slava
 * o made it compile for older versions of Windows CE
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
