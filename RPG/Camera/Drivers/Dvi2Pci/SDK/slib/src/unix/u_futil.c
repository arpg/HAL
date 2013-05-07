/*
 * $Id: u_futil.c,v 1.6 2010/10/01 16:22:49 slava Exp $
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

#include "s_util.h"
#include "s_iop.h"
#include "s_mem.h"

#include <dirent.h>
#include <termios.h>
#include <sys/ioctl.h>
#ifndef TIOCGWINSZ
#  ifdef TIOCGSIZE
#    define TIOCGWINSZ TIOCGSIZE
#    define winsize    ttysize
#    define ws_row     ts_lines
#    define ws_col     ts_cols
#  else
#    error "Neither TIOCGSIZE nor TIOCGWINSZ is defined"
#  endif /* TIOCGSIZE */
#endif /* TIOCGWINSZ */

/* Definition of the iterator */
STATIC IElement DIR_ItrNext P_((Iterator * itr));
STATIC void DIR_ItrFree P_((Iterator * itr));

typedef struct _UnixDirIterator {
    DirIterator common;     /* common part */
    DIR* dir;               /* directiry handle */
    struct dirent * dirent; /* next directory entry */
} UnixDirIterator;

STATIC const Itr dirIterator = {
    TEXT("ListDir"),        /* name     */
    DIR_ItrHasNext,         /* hasNext  */
    DIR_ItrNext,            /* next     */
    DIR_ItrRemove,          /* remove   */
    NULL,                   /* destroy  */
    DIR_ItrFree             /* free     */
};

/*
 * Returns number of lines and columns on the output console attached to 
 * this process.
 */

Bool IO_TermSize(int * rows, int * cols)
{
    int fd = fileno(stdout);
    if (isatty(fd)) {
        struct winsize win;
        memset(&win, 0, sizeof(win));
        if (ioctl(fd, TIOCGWINSZ, &win) == 0) {
            if (win.ws_row > 0 && win.ws_col > 0) {
                if (rows) *rows = win.ws_row;
                if (cols) *cols = win.ws_col;
                return True;
            }
        }
        TRACE2("tty ioctl failed, err %d (%s)\n",errno, strerror(errno));
    }
    return False;
}

/**
 * Returns True if given character is a platform specific file separator 
 * character.
 */
Bool FILE_IsFileSeparator(Char c) 
{
    return (c == FILE_SEPARATOR_CHAR);
}

/**
 * Finds file separator in the path, returning -1 if separator
 * is not found
 */
int FILE_FindSeparator(Str s) 
{
    Str sep = strchr(s, FILE_SEPARATOR_CHAR);
    return sep ? (sep-s) : (-1);
}

/**
 * Returns pointer to the filename portion of the given path. While this is 
 * trivial under Unix (i.e. strrchr(path,'/')), it's not so easy under Win32.
 * Returns NULL only if path is NULL. If the path ends with a file separator
 * character, returns an empty string.
 */
Str FILE_FilePart(Str path) 
{
    if (path) {
        Str sep = StrrChr(path, FILE_SEPARATOR_CHAR);
        if (sep) {
            return (sep + 1);
        }
    }
    return path;
}

/**
 * Returns the file size in bytes, -1 if the file doesn't exists or if it's
 * not a regular file.
 */
I64s FILE_Size(Str fname)
{
    struct stat st;
    int err = stat(fname, &st);
    return (!err && S_ISREG(st.st_mode)) ? st.st_size : -1;
}

/**
 * Returns True if the specified file or directory exists and accessible 
 * in context of the current process. Returns False if file does not exist 
 * or otherwise not accessible (no permissions, file locked, etc.)
 */
Bool FILE_Exist(Str fname) 
{
    struct stat buf;
    int err = stat(fname, &buf);
    return BoolValue(err == 0);
}

/**
 * Returns True if file or directory does not exist. Returns False if file 
 * exists although not necessarily accessible in context of the current 
 * process.
 */
Bool FILE_NonExist(Str fname) 
{
    struct stat buf;
    int err = stat(fname, &buf);
    if (err != 0) {
        switch (errno) {
        case ENOENT:
        case ENOTDIR:
            return True;
        }
    }
    return False; 
}

/**
 * Returns True if the specified file exists and is not a directory.
 */
Bool FILE_IsFile(Str fname) 
{
    struct stat buf;
    int err = stat(fname, &buf);
    return BoolValue(err == 0 && S_ISREG(buf.st_mode));
}

/**
 * Returns True if the specified file is a directory.
 */
Bool FILE_IsDir(Str fname) 
{
    struct stat buf;
    int err = stat(fname, &buf);
    return BoolValue(err == 0 && S_ISDIR(buf.st_mode));
}

/**
 * Checks whether the file name is absolute.
 */
Bool FILE_IsAbs(Str fname)
{
    return fname && fname[0] == '/';
}

/**
 * Deletes a file
 */
Bool FILE_Delete(Str fname)
{
    return (unlink(fname) == 0);
}

/**
 * Renames a file
 */
Bool FILE_Rename(Str oldn, Str newn) 
{
    return (rename(oldn, newn) == 0);
}

/**
 * Internal function for creating a directory.
 */
Bool FILE_CreateDir(Str dir) 
{
    if (mkdir(dir, ALLPERMS) == 0) {
        return True;
    } else if (errno != EEXIST) {
        return False;
    }
    return FILE_IsDir(dir);
}

/**
 * Calls a callback on each entry in the directory.
 * Returns number of directory entries read, 0 if directory 
 * was empty, -1 if error occurs of callback function returns False. 
 * The current and parent directories ("." and "..") are not included 
 * in the count, and callback is not invoked on those directory entries 
 * because these are always valid file names in any directory.
 */
int FILE_List(Str dir, FileListCB cb, void * ctx) 
{
    int n = -1;
    DIR* d;
    if (!dir || !dir[0]) dir = ".";
    d = opendir(dir);
    if (d) {
        struct dirent * entry;
        n = 0;
        while ((entry = readdir(d)) != NULL) {
            if (strcmp(entry->d_name, ".") && strcmp(entry->d_name, "..")) {
                if (cb && !(*cb)(dir, entry->d_name, ctx)) {
                    n = -1;
                    break;
                }
                n++;
            }
        }
        closedir(d);
    }
    return n; 
}

/**
 * Enumerates the contents of the directory.
 */
Iterator * FILE_ListDir(Str dir)
{
    UnixDirIterator * u = MEM_New(UnixDirIterator);
    if (u) {
        DirIterator * di = &u->common;
        memset(u, 0, sizeof(*u));
        DIR_ItrInit(di, &dirIterator);
        if (!dir || !dir[0]) dir = TEXT(".");
        if (STRBUF_Copy(&di->dirName, dir)) {
            u->dir = opendir(dir);
            if (u->dir) {
                while ((u->dirent = readdir(u->dir)) != NULL) {
                    if (StrCmp(u->dirent->d_name, TEXT(".")) && 
                        StrCmp(u->dirent->d_name, TEXT(".."))) {
                        di->hasNext = True;
                        break;
                    }
                }
                /* 
                 * preallocate space for the file name. Note that if
                 * di->hasNext is False, u->dirent is NULL.
                 */
                if (!di->hasNext ||
                    STRBUF_Alloc(&di->fileName, StrLen(u->dirent->d_name))) {
                    return &di->itr;
                }
                closedir(u->dir);
            }
        }
        DIR_ItrDestroy(&u->common);
        MEM_Free(u);
    }
    return NULL;
}

STATIC IElement DIR_ItrNext(Iterator * itr)
{
    UnixDirIterator * u = CAST(itr,UnixDirIterator,common.itr);
    DirIterator * di = &u->common;

    /* this must succeed because we have preallocated the memory */
    VERIFY(STRBUF_Copy(&di->fileName, u->dirent->d_name));
    di->entry.name = STRBUF_Text(&di->fileName);

    /*
     * dirent->d_type field is not required by POSIX and does not exist on some 
     * platforms such as Solaris. DT_DIR macro is a good indicator of whether
     * this field is present (otherwise, why define it?)
     */
#  ifdef DT_DIR
    switch (u->dirent->d_type) {
    case DT_FIFO: di->entry.type = DTypePipe;    break;
    case DT_CHR:  di->entry.type = DTypeChar;    break;
    case DT_DIR:  di->entry.type = DTypeDir;     break;
    case DT_BLK:  di->entry.type = DTypeBlock;   break;
    case DT_REG:  di->entry.type = DTypeFile;    break;
    case DT_LNK:  di->entry.type = DTypeLink;    break;
    case DT_SOCK: di->entry.type = DTypeSocket;  break;
    default:      di->entry.type = DTypeUnknown; break;
    }
#  else /* !DT_DIR */
    di->entry.type = DTypeUnknown;
#  endif /* !DT_DIR */
    /* try to determine the file if it's unknown */
    if (di->entry.type == DTypeUnknown) {
        int dirlen = STRBUF_Length(&di->dirName);
        if (STRBUF_Alloc(&di->dirName,dirlen+STRBUF_Length(&di->fileName)+1)) {
            struct stat st;
            int err;
            STRBUF_Append(&di->dirName, FILE_SEPARATOR);
            STRBUF_AppendBuf(&di->dirName, &di->fileName);
            err = stat(STRBUF_Text(&di->dirName), &st);
            if (err == 0) {
                if (S_ISREG(st.st_mode)) {
                    di->entry.type = DTypeFile;
                } else if (S_ISDIR(st.st_mode)) {
                    di->entry.type = DTypeDir;
                } else if (S_ISLNK(st.st_mode)) {
                    di->entry.type = DTypeLink;
                } else if (S_ISCHR(st.st_mode)) {
                    di->entry.type = DTypeChar;
                } else if (S_ISBLK(st.st_mode)) {
                    di->entry.type = DTypeBlock;
                } else if (S_ISFIFO(st.st_mode)) {
                    di->entry.type = DTypePipe;
                } else if (S_ISSOCK(st.st_mode)) {
                    di->entry.type = DTypeSocket;
                }
            }
            STRBUF_SetLength(&di->dirName, dirlen);
        }
    }        

    /* find the next entry */
    di->hasNext = False;
    while ((u->dirent = readdir(u->dir)) != NULL) {
        if (StrCmp(u->dirent->d_name, TEXT(".")) && 
            StrCmp(u->dirent->d_name, TEXT(".."))) {
            /* preallocate space for the file name */
            if (STRBUF_Alloc(&di->fileName, StrLen(u->dirent->d_name))) {
                di->hasNext = True;
            }
            break;
        }
    }

    di->entry.dir = STRBUF_Text(&di->dirName);
    di->entry.name = STRBUF_Text(&di->fileName);
    return &di->entry;
}

STATIC void DIR_ItrFree(Iterator * itr)
{
    UnixDirIterator * u = CAST(itr,UnixDirIterator,common.itr);
    DIR_ItrDestroy(&u->common);
    closedir(u->dir);
    MEM_Free(u);
}

/*
 * HISTORY:
 *
 * $Log: u_futil.c,v $
 * Revision 1.6  2010/10/01 16:22:49  slava
 * o added FILE_Size function
 *
 * Revision 1.5  2006/04/05 05:30:56  slava
 * o fixed a bug in FILE_IsFile function
 *
 * Revision 1.4  2006/03/30 06:52:17  slava
 * o added FILE_IsAbs function
 *
 * Revision 1.3  2005/02/21 17:42:29  slava
 * o cleanup
 *
 * Revision 1.2  2005/02/19 01:52:00  slava
 * o fixed a bug in FILE_ListDir
 *
 * Revision 1.1  2005/02/19 01:11:21  slava
 * o Unix specific code
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C ../.."
 * End:
 */
