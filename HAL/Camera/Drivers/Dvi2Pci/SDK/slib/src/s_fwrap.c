/*
 * $Id: s_fwrap.c,v 1.14 2009/10/08 14:32:11 slava Exp $
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

#include "s_util.h"
#include "s_fio.h"
#include "s_mem.h"

/*==========================================================================*
 *              L I N E    W R A P P E R
 *==========================================================================*/

#define WRAP_DISABLE    0x0001  /* wrapping intentionally disabled */
#define WRAP_SKIP_EOL   0x0002  /* need to skip next '\n' from the stream */
#define WRAP_EMPTY_LINE 0x0004  /* currently at the beginning of new line  */
#define WRAP_SCAN_WORD  0x0008  /* scanning a "word" */

#undef WrapFile
typedef struct _WrapFile {
    File file;      /* shared File structure */
    File * target;  /* the target stream */
    int  margin;    /* the right margin */
    int  level;     /* the indentation level */
    int  step;      /* the indentation step */
    int  pos;       /* current position */
    int  flags;     /* state flags */
} WrapFile;

STATIC Bool   WrapReopen P_((File * f, Str path, const char * mode));
STATIC int    WrapRead   P_((File * f, void * buf, int len));
STATIC int    WrapWrite  P_((File * f, const void * buf, int len));
STATIC Bool   WrapEof    P_((File * f));
STATIC Bool   WrapFlush  P_((File * f));
STATIC File * WrapTarget P_((File * f));
STATIC void   WrapDetach P_((File * f));
STATIC void   WrapClose  P_((File * f));
STATIC void   WrapFree   P_((File * f));

/*
 * Table of I/O handlers
 */
STATIC const FileIO WrapIO = {
    NULL                /* open     */,
    WrapReopen          /* reopen   */,
    NULL                /* setparam */,
    WrapRead            /* read     */,
    WrapWrite           /* write    */,
    NULL                /* skip     */,
    WrapFlush           /* flush    */,
    WrapEof             /* eof      */,
    NULL                /* fd       */,
    WrapTarget          /* target   */,
    WrapDetach          /* detach   */,
    WrapClose           /* close    */,
    WrapFree            /* free     */,
    0                   /* flags    */
};

/*
 * I/O handlers
 */
STATIC WrapFile * WrapFileCast(File * f)
{
    ASSERT(f);
    if (f) {
        ASSERT(f->io == &WrapIO);
        if (f->io == &WrapIO) {
            return CAST(f,WrapFile,file);
        }
    }
    return NULL;
}

STATIC Bool WrapReopen(File * f, Str path, const char * mode)
{
    WrapFile * w = WrapFileCast(f);
    if (w) {
        w->pos = 0;
        return FILE_Reopen(w->target, path, mode);
    }
    return False;
}

STATIC int WrapRead(File * f, void * buf, int len)
{
    WrapFile * w = WrapFileCast(f);
    return (w ? FILE_Read(w->target, buf, len) : (-1));
}

STATIC int WrapWrite(File * f, const void * buf, int len)
{
    int nbytes = -1;
    WrapFile * w = WrapFileCast(f);
    if (w) {
        const char * s = (char*)buf;
        if (len == 0 || w->margin <= 0) {
            nbytes = FILE_Write(w->target, buf, len);
        } else if (w->flags & WRAP_DISABLE) {
            int i;

            /* keep track of current position and state */
            for (i=0; i<len; i++) {
                char c = s[i];
                switch (c) {
                case '\n':
                case '\r':
                    w->pos = 0;
                    w->flags |= WRAP_EMPTY_LINE;
                    w->flags &= ~WRAP_SCAN_WORD;
                    break;
                default:
                    w->pos++;
                    if (IsSpace(c)) {
                        w->flags &= ~WRAP_SCAN_WORD;
                    } else {
                        w->flags |= WRAP_SCAN_WORD;
                        w->flags &= ~WRAP_EMPTY_LINE;
                    }
                }
            }
            nbytes = FILE_Write(w->target, buf, len);

        } else {
            int i, commit = 0, marker = -1, start = w->pos;
            nbytes = 0;
            for (i=0; i<len; i++) {
                char c = s[i];
                ASSERT(c); /* binary content? */

                /* reset current position when we write  '\n' or '\r' */
                if (c == '\n' || c == '\r') {

                    if (w->flags & WRAP_SKIP_EOL) {
                    
                        /* reset the WRAP_SKIP_EOL flag */
                        ASSERT(w->flags & WRAP_EMPTY_LINE);
                        w->flags &= ~WRAP_SKIP_EOL;
                    
                    } else {

                        /* write the uncommited part of the string */
                        int n = FILE_Write(w->target, s+commit, i-commit+1);
                        if (n < 0) {
                            return ((nbytes > 0) ? nbytes : -1);
                        }
                        
                        /* reset local state */
                        nbytes += n;
                        commit = i+1; 
                        marker = -1;

                        /* reset persistent state */
                        w->flags |= WRAP_EMPTY_LINE;
                        w->flags &= ~WRAP_SCAN_WORD;
                        w->pos = 0;
                    }

                } else {

                    /* are we still within the margin? */
                    if (w->pos < w->margin) {

                        if (IsSpace(c)) {

                            /* skip spaces if WRAP_EMPTY_LINE is set */
                            if (!(w->flags & WRAP_EMPTY_LINE)) {
                                if (w->flags & WRAP_SCAN_WORD) {
                                    ASSERT((i == 0) || !IsSpace(s[i-1]));
                                    marker = i-1;
                                    w->flags &= ~WRAP_SCAN_WORD;
                                }
                                w->pos++;
                            }
                        
                        } else {

                            /* insert spaces */
                            if (w->flags & WRAP_EMPTY_LINE) {
                                int j;
                                int n = w->level * w->step;
                                ASSERT(w->pos == 0);
                                for (j=0; j<n; j++) {
                                    if (FILE_Putc(w->target,' ')) {
                                        nbytes++;
                                        w->pos++;
                                    } else {
                                        return ((nbytes > 0) ? nbytes : -1);
                                    }
                                }
                            
                                /* update the state */
                                w->flags &= ~(WRAP_SKIP_EOL|WRAP_EMPTY_LINE);
                                commit = i;
                            } 

                            w->flags |= WRAP_SCAN_WORD;
                            w->pos++;
                        }

                    } else {

                        if ((marker >= 0) || 
                            (IsSpace(c)) || 
                            (start > 0 && IsSpace(s[0]))) {
                            
                            int n = 0;

                            if (marker >= 0) {
                                int count = marker - commit + 1;
                                ASSERT(count > 0);

                                /* write part of string up to the marker */
                                n = FILE_Write(w->target, s+commit, count);
                                if (n < 0) {
                                    return ((nbytes > 0) ? nbytes : -1);
                                }

                                /* move the loop variable back */
                                if (marker >= 0) {
                                    i = marker;
                                }

                            /* the first word may exceed the margin... */
                            } else if ((w->flags & WRAP_SCAN_WORD) && 
                                       (IsSpace(c))) {

                                int count = i - commit;

                                /* write the first word */
                                if (count > 0) {
                                    n = FILE_Write(w->target, s+commit, count);
                                    if (n < 0) {
                                        return ((nbytes > 0) ? nbytes : -1);
                                    }
                                }

                            } else {

                                /* restart the loop */
                                i = -1;

                            }
                        
                            /* terminate the line */
                            if (!FILE_Putc(w->target, '\n')) {
                                return ((nbytes > 0) ? nbytes : -1);
                            }

                            /* reset local state */
                            nbytes += n;
                            commit = marker + 1;
                            marker = -1;

                            /* reset persistent state */
                            w->flags |= (WRAP_EMPTY_LINE|WRAP_SKIP_EOL);
                            w->flags &= ~WRAP_SCAN_WORD;
                            w->pos = 0;
                        
                        } else {           
                            ASSERT(!IsSpace(c));
                            w->flags |= WRAP_SCAN_WORD;
                            w->pos++;
                        }
                    }
                }
            }

            /* 
             * write the uncommited part of the string unless it only 
             * contains ignorable whitespaces 
             */
             if (commit < len && !(w->flags & (WRAP_EMPTY_LINE))) {
                int n = FILE_Write(w->target, s + commit, len - commit);
                if (n < 0) {
                    return ((nbytes > 0) ? nbytes : -1);
                }
                nbytes += n;
            }
        }
    }

    /* pretend we have written all the data */
    return ((nbytes >= 0) ? len : nbytes);
}

STATIC Bool WrapEof(File * f)
{
    WrapFile * w = WrapFileCast(f);
    return (w ? FILE_Eof(w->target) : True);
}

STATIC Bool WrapFlush(File * f)
{
    WrapFile * w = WrapFileCast(f);
    return (w ? FILE_Flush(w->target) : True);
}

STATIC File * WrapTarget(File * f)
{
    WrapFile * wrap = WrapFileCast(f);
    return (wrap ? wrap->target : NULL);
}

STATIC void WrapDetach(File * f)
{
    WrapFile * w = WrapFileCast(f);
    if (w) w->target = NULL;
}

STATIC void WrapClose(File * f)
{
    WrapFile * w = WrapFileCast(f);
    if (w) FILE_Finish(w->target);
}

STATIC void WrapFree(File * f)
{
    WrapFile * w = WrapFileCast(f);
    if (w) {
        ASSERT(!(f->flags & FILE_IS_OPEN));
        if (w->target) {
            FILE_Close(w->target);
            w->target = NULL;
        }
        MEM_Free(w);
    }
}

/**
 * Creates a "wrapper" for the specified file
 */
File * FILE_Wrap(File * f, int step, int margin)
{
    WrapFile * w = MEM_New(WrapFile);
    if (w) {
        memset(w, 0, sizeof(*w));
        w->target = f;
        w->step   = step;
        w->margin = margin;
        w->flags  = WRAP_EMPTY_LINE;
        if (FILE_Init(&w->file, (Str)NULL, True, &WrapIO)) {
            return &w->file;
        }
        MEM_Free(w);
    }
    return NULL;
}

/**
 * Checks if the file is a line wrapper
 */
Bool FILE_IsWrap(const File * f)
{
    return BoolValue(f && f->io == &WrapIO);
}

/**
 * Returns current indentation level or -1 if this is not a WrapIO file
 */
int WRAP_GetIndent(File * f)
{
    /* not calling WrapFileCast() to avoid ASSERTs */
    if (FILE_IsWrap(f)) {
        WrapFile * w = CAST(f,WrapFile,file);
        ASSERT(w->level >= 0);
        return w->level;
    }
    return -1;
}

/**
 * Sets current indentation level.
 * Returns True if the indentation level has been changed, False 
 * if the File object is not a "wrapper"
 */
Bool WRAP_SetIndent(File * f, int level)
{
    /* not calling WrapFileCast() to avoid ASSERTs */
    if (FILE_IsWrap(f)) {
        WrapFile * w = CAST(f,WrapFile,file);
        w->level = MAX(level,0);
        return True;
    }
    return False;
}

/**
 * Changes current indentation level. The change is typically +1 or -1.
 * Returns True if the indentation level has been changed, False 
 * if the File object is not a "wrapper"
 */
Bool WRAP_Indent(File * f, int change)
{
    /* not calling WrapFileCast() to avoid ASSERTs */
    if (FILE_IsWrap(f)) {
        WrapFile * w = CAST(f,WrapFile,file);
        w->level = MAX(w->level+change,0);
        return True;
    }
    return False;
}

/**
 * Tests if wrapping is enabled
 */
Bool WRAP_IsEnabled(const File * f)
{
    /* not calling WrapFileCast() to avoid ASSERTs */
    if (FILE_IsWrap(f)) {
        const WrapFile * w = CAST(f,WrapFile,file);
        return BoolValue(!(w->flags & WRAP_DISABLE));
    }

    return False;
}

/**
 * Enable/disable wrapping
 */
Bool WRAP_Enable(File * f, Bool enable)
{
    /* not calling WrapFileCast() to avoid ASSERTs */
    if (FILE_IsWrap(f)) {
        WrapFile * w = CAST(f,WrapFile,file);
        if (enable) {
            w->flags &= ~WRAP_DISABLE;
        } else {
            w->flags |= WRAP_DISABLE;
        }
        return True;
    }
    return False;
}

/*
 * HISTORY:
 *
 * $Log: s_fwrap.c,v $
 * Revision 1.14  2009/10/08 14:32:11  slava
 * o added FILE_Fd() and FILE_TargetFd() functions
 *
 * Revision 1.13  2008/09/01 12:08:15  slava
 * o replaced FileIO::fileIO field with flags. Added FIO_BLOCKING_READ flag
 *   which marks I/O methods whose read callback can block before the end of
 *   stream is reached (i.e. socket I/O).
 *
 * Revision 1.12  2008/09/01 10:57:21  slava
 * o added 'skip' I/O callback which can be implemented by those I/O methods
 *   which can efficiently skip input data without actually reading and
 *   copying anything. Currently, such callback is only implemented for
 *   in-memory I/O, but it can also be done for file I/O using fseek.
 *
 * Revision 1.11  2006/10/12 16:16:58  slava
 * o made WrapIO static
 *
 * Revision 1.10  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.9  2004/04/08 01:44:18  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.8  2003/09/10 00:38:03  slava
 * o changed FILE_IsWrap and WRAP_IsEnabled to take const pointer
 *
 * Revision 1.7  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.6  2003/01/08 17:37:48  slava
 * o allow to pass NULL path argument to FILE_Init. In that case, FILE_Name
 *   will return the name of the target file. This rule recursively applies
 *   to the target file as well.
 *
 * Revision 1.5  2003/01/08 17:15:43  slava
 * o removed ZIP_GetTarget and WRAP_GetTarget functions, replaced them
 *   with generic FILE_Target
 *
 * Revision 1.4  2003/01/06 03:09:31  slava
 * o keep track of WRAP_EMPTY_LINE state when wrapping is disabled
 *
 * Revision 1.3  2002/12/30 21:42:21  slava
 * o added fileIO field to the FileIO structure. This field is set to
 *   True if the File stream performs file based I/O (as opposed to socket
 *   or in-memory I/O)
 *
 * Revision 1.2  2002/08/27 11:52:03  slava
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
