/*
 * $Id: w_futil.c,v 1.7 2010/10/01 17:20:23 slava Exp $
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
#include "s_itrp.h"
#include "s_iop.h"
#include "s_mem.h"

/**
 * File utilities for Win32 environment
 */

#ifndef _WIN32
#  error "_WIN32 must be defined to compile Win32 code"
#endif

/* Definition of the iterator */
STATIC IElement DIR_ItrNext P_((Iterator * itr));
STATIC void DIR_ItrFree P_((Iterator * itr));

typedef struct _Win32DirIterator {
    DirIterator common;     /* common part */
    HANDLE handle;          /* find handle */
    WIN32_FIND_DATA data;   /* find data for the next file */
} Win32DirIterator;

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
#ifndef _WIN32_WCE
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE); 
    CONSOLE_SCREEN_BUFFER_INFO screenBufferInfo;
    if (GetConsoleScreenBufferInfo(hConsole, &screenBufferInfo)) {
        if (rows) *rows = screenBufferInfo.dwSize.Y;
        if (cols) *cols = screenBufferInfo.dwSize.X;
        return True;
    }
    TRACE1("GetConsoleScreenBufferInfo() failed, error %lu\n",GetLastError());
#endif /* !_WIN32_WCE */
    return False;
}

/**
 * Returns True if given character is a platoform specific file separator 
 * character. NOTE: under Windoze, both '\\' and '/' are treated as a file 
 * separator character.
 */
Bool FILE_IsFileSeparator(Char c) 
{
    switch (c) {
    case TEXT('/'):
    case TEXT('\\'):
        return True;
    default:
        return False;
    }
}

/**
 * Finds file separator in the path, returning -1 if separator
 * is not found
 */
int FILE_FindSeparator(Str s) 
{
    /* 
     * In Windows both the backslash ( \) and the forward slash (/) are 
     * valid path delimiters 
     */
    Str sep1 = StrChr(s, TEXT('\\'));
    Str sep2 = StrChr(s, TEXT('/'));
    size_t offset;
    if (sep1 && sep2) {
        offset = MIN(sep1-s, sep2-s);
    } else if (sep1) {
        offset = (sep1-s);
    } else if (sep2) {
        offset = (sep2-s);
    } else {
        return -1;
    }
    ASSERT(offset <= INT_MAX);
    return (int)offset;
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
        Str sep = StrrChr(path,TEXT('/'));
        /*
         * Under Win32 we have to handle
         * 1. backslashes
         * 2. syntax like C:foo.bar
         */
        Str sep2 = StrrChr(path,TEXT('\\'));
        if (sep && sep2) {
            if (sep > sep2) {
               return (sep + 1);
            } else {
               return (sep2 + 1);
            }
        } else if (sep) {
            return (sep + 1);
        } else if (sep2) {
            return (sep2 + 1);
        } else if (path[0] != 0 && path[1] == TEXT(':')) {
            return (path + 2);
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
    I64s size = -1;
    HANDLE hFile = CreateFile(fname, GENERIC_READ, FILE_SHARE_READ, NULL,
        OPEN_EXISTING, FILE_ATTRIBUTE_READONLY, NULL);
    if (hFile != INVALID_HANDLE_VALUE) {
        DWORD dwSizeHigh = 0;
        DWORD dwSizeLow = GetFileSize(hFile, &dwSizeHigh);
        if (dwSizeLow != (DWORD)-1) {
            size = (((I64s)dwSizeHigh) << 32) | dwSizeLow;
        }
        CloseHandle(hFile);
    }
    return size;
}

/**
 * Returns True if the specified file or directory exists and accessible 
 * in context of the current process. Returns False if file does not exist 
 * or otherwise not accessible (no permissions, file locked, etc.)
 */
Bool FILE_Exist(Str fname) 
{
    DWORD attrs = GetFileAttributes(fname);
    return (attrs != FILE_ATTRIBUTE_FAILURE);
}

/**
 * Returns True if file or directory does not exist. Returns False if file 
 * exists although not necessarily accessible in context of the current 
 * process.
 */
Bool FILE_NonExist(Str fname) 
{
    return !FILE_Exist(fname);
}

/**
 * Returns True if the specified file exists and is not a directory.
 */
Bool FILE_IsFile(Str fname) 
{
    DWORD attrs = GetFileAttributes(fname);
    return ((attrs != FILE_ATTRIBUTE_FAILURE) && 
            !(attrs & FILE_ATTRIBUTE_DIRECTORY));
}

/**
 * Returns True if the specified file is a directory.
 */
Bool FILE_IsDir(Str fname) 
{
    DWORD attrs = GetFileAttributes(fname);
    return ((attrs != FILE_ATTRIBUTE_FAILURE) && 
            (attrs & FILE_ATTRIBUTE_DIRECTORY));
}

/**
 * Checks whether the file name is absolute. Note that "absolute" here means
 * "not relative" which is not exactly the same as fully qualified name. On
 * Windows, absolute file names are not necessarily fully qualifiled, for
 * example \foo\bar.txt - it does not allow you to distinguish beetween
 * c:\foo\bar.txt and d:\foo\bar.txt and yet it's definitely not a relative
 * path name.
 */
Bool FILE_IsAbs(Str fname)
{
    Char c;
    if (!fname) return False;
    c = fname[0];
    if (!c) return False;
    if (c == '.') return False;
    if (c == '/' || c == '\\') return True;
    if (fname[1]==':' && ((c>='a' && c<='z')||(c>='A' && c<='Z'))) return True;
    return False;
}

/**
 * Deletes a file
 */
Bool FILE_Delete(Str fname)
{
    if (DeleteFile(fname)) {
        return True;
    } else {
        TRACE2("DeleteFile(%s) failed, error %d\n",fname,GetLastError());
        return False;
    }
}

/**
 * Renames a file
 */
Bool FILE_Rename(Str oldn, Str newn) 
{
    if (MoveFile(oldn,newn)) {
        return True;
    } else {
        TRACE3("MoveFile(%s,%s) failed, error %d\n",oldn,newn,GetLastError());
        return False;
    }
}

/**
 * Internal function for creating a directory.
 */
Bool FILE_CreateDir(Str dir) 
{
    if (CreateDirectory(dir, NULL)) {
        return True;
    } else if (GetLastError() != ERROR_ALREADY_EXISTS) {
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
    StrBuf64 pattern;
    STRBUF_InitBufXXX(&pattern);
    if (!dir || !dir[0]) dir = TEXT(".");
    if (STRBUF_Copy(&pattern.sb, dir) && 
        STRBUF_Append(&pattern.sb, TEXT("\\*.*"))) {
        WIN32_FIND_DATA data;
        HANDLE hfind = FindFirstFile(pattern.sb.s, &data);
        if (hfind != INVALID_HANDLE_VALUE) {
            Str fname = data.cFileName;
            n = 0;
            do {
                if (StrCmp(fname, TEXT(".")) && 
                    StrCmp(fname, TEXT(".."))) {
                    if (cb && !(*cb)(dir, fname, ctx)) {
                        n = -1;
                        break;
                    }
                    n++;
                }
            } while (FindNextFile(hfind, &data));
            FindClose(hfind);
        }
    }
    STRBUF_Destroy(&pattern.sb);
    return n; 
}

/**
 * Enumerates the contents of the directory.
 */
Iterator * FILE_ListDir(Str dir)
{
    Win32DirIterator * w = MEM_New(Win32DirIterator);
    if (w) {
        size_t len;
        DirIterator * di = &w->common;
        memset(w, 0, sizeof(*w));
        DIR_ItrInit(di, &dirIterator);
        if (!dir || !dir[0]) dir = TEXT(".");
        len = StrLen(dir);
        if (STRBUF_Alloc(&di->dirName, len + 4)) { 
            VERIFY(STRBUF_Copy(&di->dirName, dir));
            VERIFY(STRBUF_Append(&di->dirName, TEXT("\\*.*")));
            w->handle = FindFirstFile(di->dirName.s, &w->data);
            STRBUF_SetLength(&di->dirName, len);
            if (w->handle != INVALID_HANDLE_VALUE) {
                if (StrCmp(w->data.cFileName,TEXT(".")) && 
                    StrCmp(w->data.cFileName,TEXT(".."))) {
                    di->hasNext = True;
                } else {
                    while (FindNextFile(w->handle, &w->data)) {
                        if (StrCmp(w->data.cFileName,TEXT(".")) && 
                            StrCmp(w->data.cFileName,TEXT(".."))) {
                            di->hasNext = True;
                            break;
                        }
                    }
                }

                /* preallocate space for the file name */
                if (!di->hasNext ||
                    STRBUF_Alloc(&di->fileName, StrLen(w->data.cFileName))) {
                    return &di->itr;
                }
                FindClose(w->handle);
            }
        }
        DIR_ItrDestroy(&w->common);
        MEM_Free(w);
    }
    return NULL;
}

STATIC IElement DIR_ItrNext(Iterator * itr)
{
    Win32DirIterator * w = CAST(itr,Win32DirIterator,common.itr);
    DirIterator * di = &w->common;

    /* this must succeed because we have preallocated the memory */
    VERIFY(STRBUF_Copy(&di->fileName, w->data.cFileName));
    if (w->data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
        di->entry.type = DTypeDir;
    } else {
        di->entry.type = DTypeFile;
    }

    /* find the next entry */
    di->hasNext = False;
    while (FindNextFile(w->handle, &w->data)) {
        if (StrCmp(w->data.cFileName,TEXT(".")) && 
            StrCmp(w->data.cFileName,TEXT(".."))) {
            /* preallocate space for the file name */
            if (STRBUF_Alloc(&di->fileName, StrLen(w->data.cFileName))) {
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
    Win32DirIterator * w = CAST(itr,Win32DirIterator,common.itr);
    DIR_ItrDestroy(&w->common);
    FindClose(w->handle);
    MEM_Free(w);
}

/*
 * HISTORY:
 *
 * $Log: w_futil.c,v $
 * Revision 1.7  2010/10/01 17:20:23  slava
 * o implemented FILE_Size function on Windows
 *
 * Revision 1.6  2009/04/09 21:53:24  slava
 * o fixed debug build (MAX_INT -> INT_MAX)
 *
 * Revision 1.5  2009/04/09 21:36:26  slava
 * o fixed a warning in 64-bit Windows build
 *
 * Revision 1.4  2006/03/30 06:52:17  slava
 * o added FILE_IsAbs function
 *
 * Revision 1.3  2005/02/19 01:58:08  slava
 * o FILE_ListDir was sometimes allocating memory without necessity (if the
 *   directory was empty)
 *
 * Revision 1.2  2005/02/19 01:12:37  slava
 * o cleanup
 *
 * Revision 1.1  2005/02/19 00:37:41  slava
 * o separated platform specific code from platform independent code
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
