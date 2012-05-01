/*
 * $Id: s_futil.c,v 1.25 2009/04/09 21:59:20 slava Exp $
 *
 * Copyright (C) 2000-2009 by Slava Monich
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
#include "s_random.h"
#include "s_fio.h"
#include "s_iop.h"
#include "s_mem.h"

#ifndef __KERNEL__

/**
 * Returns pointer to buffer containing directory part of the given path.
 * The returned string is either empty or ends with file separator character. 
 * Under Win32, if may also look like "C:", but in either case file name 
 * can be simply appended to it. The return buffer has 'extra' additional 
 * characters allocated past the end of the string. The function only 
 * returns NULL if input parameter is NULL or memory allocation fails. 
 */
Char * FILE_DirName(Str path, int extra) 
{
    Char * dir = NULL;
    ASSERT(path);
    ASSERT(extra >= 0);
    if (path) {
        size_t len = FILE_FilePart(path) - path;
        size_t size = len + 1;
        if (extra >= 0) size += extra;
        dir = MEM_NewArray(Char,size);

        /* copy portion of the source path */
        if (dir) {
            if (len > 0) StrnCpy(dir, path, len);
            dir[len] = 0;
        }
    }
    return dir;
}

/**
 * Generates random file name consisting of n random characters.
 * Returns pointer to the provided buffer. The buffer must be at 
 * least (n+1) characters big.
 */
Char * FILE_TempName(Char * buf, size_t bufsize) 
{
    ASSERT(buf);
    if (buf) {
        size_t i;
        int nc = 'Z'-'A'+1;
        int nc2 = 2 * nc;
        for (i=0; i<bufsize; i++) {
            int r = RAND_NextInt(nc2);
            buf[i] = (Char)(((r >= nc) ? ('A'+r-nc) : ('a'+r)));
        }
        buf[i] = 0;
    }
    return buf;
}

/**
 * Attempts to open a regular file in specified mode. 
 * PORTABILITY NOTE: on Unix it's possible to open a directory file,
 * while on Win32 fopen() cannot read directories. Therefore, this
 * function cannot be used as a check for existence or non-existence 
 * of a file. Use FILE_Exist() and FILE_NonExist() for that purpose
 */
Bool FILE_CanOpen(Str fname, const char * mode) 
{
    FILE * f;
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

    f = Fopen(fname, _mode);
    if (f) {
        fclose(f);
        return True;
    }
    return False;
}

/**
 * FILE_RmDir callback. Recursively destroys the directory contents
 */
STATIC Bool RmDirCB(Str dir, Str fname, void * ctx) 
{
    Bool ok = False;
    StrBuf64 path;
    UNREF(ctx);
    STRBUF_InitBufXXX(&path);
    if (STRBUF_Copy(&path.sb, dir) && 
        STRBUF_AppendChar(&path.sb, FILE_SEPARATOR_CHAR) &&
        STRBUF_Append(&path.sb, fname)) {
        ok = True;
        if (FILE_IsDir(path.sb.s)) {
            FILE_RmDir(path.sb.s, True);
        } else {
            FILE_Delete(path.sb.s);
        }
    }
    STRBUF_Destroy(&path.sb);
    return ok;
}

/**
 * Creates a directory hierarhy.
 */
Bool FILE_MkDir(Str dir) 
{
    Bool ok = False;
    StrBuf64 entry;
    StrBuf* sb = &entry.sb;
    STRBUF_InitBufXXX(&entry);

    if (STRBUF_Copy(sb, dir)) {
        while (sb->len > 0 && FILE_IsFileSeparator(STRBUF_LastChar(sb))) {
            STRBUF_SetLength(sb, sb->len-1);
        }

        /* check if the directory already exists */
        if (FILE_IsDir(STRBUF_Text(sb)) ||
            FILE_CreateDir(STRBUF_Text(sb))) {
            ok = True;

        } else {

            /* directory does not exists, walk the hierarhy */
            int pos = 0;
            int next = 0;
            while ((next = FILE_FindSeparator(dir+pos)) >= 0) {
                STRBUF_Clear(sb);
                if (next == 0) {
                    pos++;
                    continue;
                } else {
                    if (!STRBUF_CopyN(sb,dir,pos+next) || 
                        !FILE_CreateDir(STRBUF_Text(sb))) {
                        break;
                    }
                    pos += next + 1;
                }
            }
                
            /* final test */
            if (STRBUF_Copy(sb, dir)) {
                while (sb->len && FILE_IsFileSeparator(STRBUF_LastChar(sb))) {
                    STRBUF_SetLength(sb, sb->len-1);
                }
                if (FILE_IsDir(STRBUF_Text(sb)) ||
                    FILE_CreateDir(STRBUF_Text(sb))) {
                    ok = True;
                }
            }
        }
    }

    STRBUF_Destroy(sb);
    return ok;
}

/**
 * Removes the directory. If recurse parameter is True, also removes 
 * all files and subdirectories, otherwise it fails if directory is not
 * empty.
 */
Bool FILE_RmDir(Str dir, Bool recurse) 
{
    if (FILE_IsDir(dir)) {
        if (recurse) {
            FILE_List(dir, RmDirCB, NULL);
        }
#ifdef _WIN32
        return RemoveDirectory(dir);
#else
        return BoolValue(Rmdir(dir) == 0);
#endif /* WIN32 */
    }
    return False;
}

/**
 * Generates unique file name. Makes sure such file does not exist.
 * There's still non-zero probability that two processes would step 
 * on each other, but hell with it :)
 */
void FILE_MakeUnique(Char * name, size_t fixedPart, size_t randomPart) 
{
    Bool exists;
    do {
        FILE_TempName(name + fixedPart, randomPart);
        exists = BoolValue(!FILE_NonExist(name));
    } while (exists);
}

/**
 * Save something into the file. This function does it carefully, saving 
 * data into temporary file X, then renaming X into the specified file.
 * This way we never lose *both* old and new versions of the file.
 *
 * NOTE: Win32 implementation of rename() fails if target file exists,
 * while on Linux for example, it does not not. Since this code is 
 * supposed to be portable, we have to assume the worst (i.e. Win32) 
 * and do WRITE(tmp1)-RENAME(file,tmp2)-RENAME(tmp1,file)-DELETE(tmp2)
 * instead of just WRITE(tmp)-RENAME(tmp,file)
 *
 * This function opens file in a text mode ("t" mode on Win32).
 */
Bool FILE_Save(Str fname, FileSaveCB cb, void * ctx, IODesc io)
{
    return FILE_Save2(fname, cb, ctx, True, io);
}

/**
 * Same as the above, only allows you to specify file mode (text vs binary)
 * on those platforms where it matters (e.g. Windows).
 */
Bool FILE_Save2(Str fname, FileSaveCB cb, void * ctx, Bool txt, IODesc io)
{
    Bool success = False;
    ASSERT(fname && fname[0]);
    if (fname && fname[0]) {
        Char * tmp1 = FILE_DirName(fname, TEMP_FILE_NAME_LEN);
        if (tmp1 && (!tmp1[0] || FILE_MkDir(tmp1))) {
            const char * mode = txt ? WRITE_TEXT_MODE : WRITE_BINARY_MODE;
            size_t dirlen = StrLen(tmp1);
            File * f;

            /* 
             * write the temp file 
             */
            FILE_MakeUnique(tmp1, dirlen, TEMP_FILE_NAME_LEN);
            f = FILE_Open(tmp1, mode, io);
            if (f) {

                Bool saved = (*cb)(f,fname,ctx);
                FILE_Close(f);
                if (saved) {
                    
                    if (FILE_CanRead(fname)) {

                        /* 
                         * generate another unique file name
                         */
                        size_t nchars = dirlen + 1 + TEMP_FILE_NAME_LEN;
                        Char * tmp2 = MEM_NewArray(Char,nchars);
                        if (tmp2) {
                            StrCpy(tmp2, tmp1);
                            FILE_MakeUnique(tmp2,dirlen,TEMP_FILE_NAME_LEN);

                            /* 
                             * rename target -> tmp2
                             */
                
                            Verbose(TEXT("Renaming %s into %s\n"),fname,tmp2);
                            if (FILE_Rename(fname, tmp2)) {

                                /* 
                                 * rename tmp1 -> target
                                 */
                
                                Verbose(TEXT("Renaming %s into %s\n"),tmp1,fname);
                                if (FILE_Rename(tmp1, fname)) {
        
                                    /*
                                     * finally, remove tmp2 (old target)
                                     * perhaps, we should ignore the error
                                     * returned by FILE_Delete?
                                     */
                                    Verbose(TEXT("Deleting %s\n"),tmp2);
                                    success = FILE_Delete(tmp2);
                                }
                            }

                            MEM_Free(tmp2);
                        }

                    /*
                     * there's no target, just rename tmp1 -> target
                     */
                    } else {
                        Verbose(TEXT("Renaming %s into %s\n"),tmp1,fname);
                        success = FILE_Rename(tmp1, fname);
                    }

                } else {

                    /*
                     * just quietly delete the temporary file. An error
                     * message, if any, should have been provided by the
                     * callback.
                     */
                    FILE_Delete(tmp1);
                }
            }
        }
        MEM_Free(tmp1);
    }
    return success;
}

/**
 * Initializer for the platform independent part of the directory iterator.
 * Assumes that the memory has been zeroed.
 */
void DIR_ItrInit(DirIterator * di, const Itr * type)
{
    ITR_Init(&di->itr, type);
    STRBUF_Init(&di->dirName);
    STRBUF_Init(&di->fileName);
}

Bool DIR_ItrHasNext(Iterator * itr)
{
    DirIterator * di = CAST(itr,DirIterator,itr);
    return di->hasNext;
}

Bool DIR_ItrRemove(Iterator * itr)
{
    Bool ok = False;
    DirIterator * di = CAST(itr,DirIterator,itr);
    DirType type = di->entry.type;
    size_t dirlen = STRBUF_Length(&di->dirName);
    if (STRBUF_Alloc(&di->dirName, dirlen+STRBUF_Length(&di->fileName)+1)) {
        Str fullName;
        Bool isDir;
        STRBUF_AppendChar(&di->dirName, FILE_SEPARATOR_CHAR);
        STRBUF_AppendBuf(&di->dirName, &di->fileName);
        fullName = STRBUF_Text(&di->dirName);
        if (type == DTypeUnknown) {
            isDir = FILE_IsDir(fullName);
        } else {
            isDir = BoolValue(type == DTypeDir);
        }
        if (isDir) {
            ok = FILE_RmDir(fullName, True);
        } else {
            ok = FILE_Delete(fullName);
        }
        STRBUF_SetLength(&di->dirName, dirlen);
        di->entry.dir = STRBUF_Text(&di->dirName);
    }
    return ok;
}

void DIR_ItrDestroy(DirIterator * di)
{
    STRBUF_Destroy(&di->dirName);
    STRBUF_Destroy(&di->fileName);
}

#endif /* __KERNEL__ */

/**
 * The following functions write a 16/32/64-bit number to the stream
 * performing conversion it to the specified byte order if necessary.
 */
Bool FILE_WriteI16(File * f, I16u data, int endianess)
{
    data = DATA_Conv16(data, BYTE_ORDER, endianess);
    return FILE_WriteAll(f, &data, sizeof(data));
}

Bool FILE_WriteI32(File * f, I32u data, int endianess)
{
    data = DATA_Conv32(data, BYTE_ORDER, endianess);
    return FILE_WriteAll(f, &data, sizeof(data));
}

Bool FILE_WriteI64(File * f, I64u data, int endianess)
{
    data = DATA_Conv64(data, BYTE_ORDER, endianess);
    return FILE_WriteAll(f, &data, sizeof(data));
}

/**
 * The following functions read a 16/32/64-bit number to the stream
 * performing conversion from the specified byte order to the host
 * byte order if necessary.
 */
Bool FILE_ReadU16(File * f, I16u * data, int endianess)
{
    I16u tmp;
    ASSERT(data);
    if (FILE_ReadAll(f, &tmp, sizeof(*data))) {
        *data = DATA_Conv16(tmp, endianess, BYTE_ORDER);
        return True;
    }
    return False;
}

Bool FILE_ReadU32(File * f, I32u * data, int endianess)
{
    I32u tmp;
    ASSERT(data);
    if (FILE_ReadAll(f, &tmp, sizeof(*data))) {
        *data = DATA_Conv32(tmp, endianess, BYTE_ORDER);
        return True;
    }
    return False;
}

Bool FILE_ReadU64(File * f, I64u * data, int endianess)
{
    I64u tmp;
    ASSERT(data);
    if (FILE_ReadAll(f, &tmp, sizeof(*data))) {
        *data = DATA_Conv64(tmp, endianess, BYTE_ORDER);
        return True;
    }
    return False;
}

/**
 * The following functions write a 16/32/64-bit number to the stream
 * converting it to the network (big endian, MSB first) byte order
 * if necessary. It's functionally equivalent to FILE_WriteIxx(f,
 * data, BIG_ENDIAN) but these functions are a bit faster on big
 * endian systems because the byte order decision is made at compile
 * time.
 */
Bool FILE_WriteI16B(File * f, I16u data)
{
#if BYTE_ORDER != BIG_ENDIAN
    data = DATA_Conv16(data, BYTE_ORDER, BIG_ENDIAN);
#endif /* BYTE_ORDER != BIG_ENDIAN */
    return FILE_WriteAll(f, &data, sizeof(data));
}

Bool FILE_WriteI32B(File * f, I32u data)
{
#if BYTE_ORDER != BIG_ENDIAN
    data = DATA_Conv32(data, BYTE_ORDER, BIG_ENDIAN);
#endif /* BYTE_ORDER != BIG_ENDIAN */
    return FILE_WriteAll(f, &data, sizeof(data));
}

Bool FILE_WriteI64B(File * f, I64u data)
{
#if BYTE_ORDER != BIG_ENDIAN
    data = DATA_Conv64(data, BYTE_ORDER, BIG_ENDIAN);
#endif /* BYTE_ORDER != BIG_ENDIAN */
    return FILE_WriteAll(f, &data, sizeof(data));
}

/* The same, only for reading 16/32/64-bit numbers from big endian stream */
Bool FILE_ReadU16B(File * f, I16u * data)
{
#if BYTE_ORDER != BIG_ENDIAN
    I16u tmp;
    if (FILE_ReadAll(f, &tmp, sizeof(*data))) {
        *data = DATA_Conv16(tmp, BIG_ENDIAN, BYTE_ORDER);
        return True;
    }
    return False;
#else  /* BYTE_ORDER == BIG_ENDIAN */
    return FILE_ReadAll(f, data, sizeof(*data));
#endif /* BYTE_ORDER == BIG_ENDIAN */
}

Bool FILE_ReadU32B(File * f, I32u * data)
{
#if BYTE_ORDER != BIG_ENDIAN
    I32u tmp;
    if (FILE_ReadAll(f, &tmp, sizeof(*data))) {
        *data = DATA_Conv32(tmp, BIG_ENDIAN, BYTE_ORDER);
        return True;
    }
    return False;
#else  /* BYTE_ORDER == BIG_ENDIAN */
    return FILE_ReadAll(f, data, sizeof(*data));
#endif /* BYTE_ORDER == BIG_ENDIAN */
}

Bool FILE_ReadU64B(File * f, I64u * data)
{
#if BYTE_ORDER != BIG_ENDIAN
    I64u tmp;
    if (FILE_ReadAll(f, &tmp, sizeof(*data))) {
        *data = DATA_Conv64(tmp, BIG_ENDIAN, BYTE_ORDER);
        return True;
    }
    return False;
#else  /* BYTE_ORDER == BIG_ENDIAN */
    return FILE_ReadAll(f, data, sizeof(*data));
#endif /* BYTE_ORDER == BIG_ENDIAN */
}

/**
 * The following functions write a 16/32/64-bit number to the stream
 * converting it to little endian (LSB first) byte order if necessary.
 * It's functionally equivalent to FILE_WriteIxx(f, data, LITTLE_ENDIAN)
 * but it's a bit faster on little endian systems (such as Intel) because
 * the byte order decision is made at compile time.
 */
Bool FILE_WriteI16L(File * f, I16u data)
{
#if BYTE_ORDER != LITTLE_ENDIAN
    data = DATA_Conv16(data, BYTE_ORDER, LITTLE_ENDIAN);
#endif /* BYTE_ORDER == BIG_ENDIAN */
    return FILE_WriteAll(f, &data, 2);
}

Bool FILE_WriteI32L(File * f, I32u data)
{
#if BYTE_ORDER != LITTLE_ENDIAN
    data = DATA_Conv32(data, BYTE_ORDER, LITTLE_ENDIAN);
#endif /* BYTE_ORDER == BIG_ENDIAN */
    return FILE_WriteAll(f, &data, 4);
}

Bool FILE_WriteI64L(File * f, I64u data)
{
#if BYTE_ORDER != LITTLE_ENDIAN
    data = DATA_Conv64(data, BYTE_ORDER, LITTLE_ENDIAN);
#endif /* BYTE_ORDER == BIG_ENDIAN */
    return FILE_WriteAll(f, &data, 2);
}

/* The same, only for reading numbers from little endian stream */
Bool FILE_ReadU16L(File * f, I16u * data)
{
#if BYTE_ORDER != LITTLE_ENDIAN
    I16u tmp;
    if (FILE_ReadAll(f, &tmp, sizeof(*data))) {
        *data = DATA_Conv16(tmp, LITTLE_ENDIAN, BYTE_ORDER);
        return True;
    }
    return False;
#else  /* BYTE_ORDER == LITTLE_ENDIAN */
    return FILE_ReadAll(f, data, sizeof(*data));
#endif /* BYTE_ORDER == LITTLE_ENDIAN */
}

Bool FILE_ReadU32L(File * f, I32u * data)
{
#if BYTE_ORDER != LITTLE_ENDIAN
    I32u tmp;
    if (FILE_ReadAll(f, &tmp, sizeof(*data))) {
        *data = DATA_Conv32(tmp, LITTLE_ENDIAN, BYTE_ORDER);
        return True;
    }
    return False;
#else  /* BYTE_ORDER == LITTLE_ENDIAN */
    return FILE_ReadAll(f, data, sizeof(*data));
#endif /* BYTE_ORDER == LITTLE_ENDIAN */
}

Bool FILE_ReadU64L(File * f, I64u * data)
{
#if BYTE_ORDER != LITTLE_ENDIAN
    I64u tmp;
    if (FILE_ReadAll(f, &tmp, sizeof(*data))) {
        *data = DATA_Conv64(tmp, LITTLE_ENDIAN, BYTE_ORDER);
        return True;
    }
    return False;
#else  /* BYTE_ORDER == LITTLE_ENDIAN */
    return FILE_ReadAll(f, data, sizeof(*data));
#endif /* BYTE_ORDER == LITTLE_ENDIAN */
}

#if DEBUG
/* these functions only exist for better type safety in debug build */
Bool FILE_ReadI16(File * f, I16s * data, int endianess)
{
    return FILE_ReadU16(f, (I16u*)data, endianess);
}

Bool FILE_ReadI32(File * f, I32s * data, int endianess)
{
    return FILE_ReadU32(f, (I32u*)data, endianess);
}

Bool FILE_ReadI64(File * f, I64s * data, int endianess)
{
    return FILE_ReadU64(f, (I64u*)data, endianess);
}

Bool FILE_ReadI16B(File * f, I16s * data)
{
    return FILE_ReadU16B(f, (I16u*)data);
}

Bool FILE_ReadI32B(File * f, I32s * data)
{
    return FILE_ReadU32B(f, (I32u*)data);
}

Bool FILE_ReadI64B(File * f, I64s * data)
{
    return FILE_ReadU64B(f, (I64u*)data);
}

Bool FILE_ReadI16L(File * f, I16s * data)
{
    return FILE_ReadU16L(f, (I16u*)data);
}

Bool FILE_ReadI32L(File * f, I32s * data)
{
    return FILE_ReadU32L(f, (I32u*)data);
}

Bool FILE_ReadI64L(File * f, I64s * data)
{
    return FILE_ReadU64L(f, (I64u*)data);
}
#endif /* DEBUG */

#ifndef __KERNEL__
/* Assumptions we make about the size of floating point data types */
COMPILE_ASSERT(sizeof(float) == sizeof(I32u));
COMPILE_ASSERT(sizeof(double) == sizeof(I64u));

/**
 * The following functions write a 32/64-bit (single/double precision)
 * floating-point number to the stream converting it to the specified
 * byte order if necessary.
 */
Bool FILE_WriteF32(File * f, float data, int endianess)
{
    I32u tmp = DATA_Conv32(*((I32u*)(void*)&data), BYTE_ORDER, endianess);
    return FILE_WriteAll(f, &tmp, 4);
}

Bool FILE_WriteF64(File * f, double data, int endianess)
{
    I64u tmp = DATA_Conv64(*((I64u*)(void*)&data), BYTE_ORDER, endianess);
    return FILE_WriteAll(f, &tmp, 8);
}

/* Reading numbers from a stream */
Bool FILE_ReadF32(File * f, float * data, int endianess)
{
    I32u tmp;
    ASSERT(data);
    if (FILE_ReadAll(f, &tmp, sizeof(tmp))) {
        *((I32u*)data) = DATA_Conv32(tmp, endianess, BYTE_ORDER);
        return True;
    }
    return False;
}

Bool FILE_ReadF64(File * f, double * data, int endianess)
{
    I64u tmp;
    ASSERT(data);
    if (FILE_ReadAll(f, &tmp, sizeof(tmp))) {
        *((I64u*)data) = DATA_Conv64(tmp, endianess, BYTE_ORDER);
        return True;
    }
    return False;
}

/**
 * The following functions write a 32/64-bit (single/double precision)
 * floating-point number to the stream converting it to the network
 * (big endian, MSB first) byte order if necessary. It's functionally
 * equivalent to FILE_WriteFxx(f,data,BIG_ENDIAN) but it's a bit faster
 * on big endian systems because the byte order decision is made at compile
 * time.
 */
Bool FILE_WriteF32B(File * f, float data)
{
    I32u tmp = (*((I32u*)(void*)&data));
#if BYTE_ORDER != BIG_ENDIAN
    tmp = DATA_Conv32(tmp, BYTE_ORDER, BIG_ENDIAN);
#endif /* BYTE_ORDER == BIG_ENDIAN */
    return FILE_WriteAll(f, &tmp, 4);
}

Bool FILE_WriteF64B(File * f, double data)
{
    I64u tmp = (*((I64u*)(void*)&data));
#if BYTE_ORDER != BIG_ENDIAN
    tmp = DATA_Conv64(tmp, BYTE_ORDER, BIG_ENDIAN);
#endif /* BYTE_ORDER == BIG_ENDIAN */
    return FILE_WriteAll(f, &tmp, 8);
}

/* The same, only for reading numbers from big endian stream */
Bool FILE_ReadF32B(File * f, float * data)
{
#if BYTE_ORDER != BIG_ENDIAN
    I32u tmp;
    ASSERT(data);
    if (FILE_ReadAll(f, &tmp, sizeof(tmp))) {
        *((I32u*)data) = DATA_Conv32(tmp, BIG_ENDIAN, BYTE_ORDER);
        return True;
    }
    return False;
#else  /* BYTE_ORDER == BIG_ENDIAN */
    return FILE_ReadAll(f, data, sizeof(*data));
#endif /* BYTE_ORDER == BIG_ENDIAN */
}

Bool FILE_ReadF64B(File * f, double * data)
{
#if BYTE_ORDER != BIG_ENDIAN
    I64u tmp;
    ASSERT(data);
    if (FILE_ReadAll(f, &tmp, sizeof(tmp))) {
        *((I64u*)data) = DATA_Conv64(tmp, BIG_ENDIAN, BYTE_ORDER);
        return True;
    }
    return False;
#else  /* BYTE_ORDER == BIG_ENDIAN */
    return FILE_ReadAll(f, data, sizeof(*data));
#endif /* BYTE_ORDER == BIG_ENDIAN */
}

/**
 * The following functions write a 32/64-bit (single/double precision)
 * floating-point number to the stream converting it to little endian
 * (LMSB first) byte order if necessary. It's functionally equivalent
 * to FILE_WriteFxx(f,data,LITTLE_ENDIAN) but it's a bit faster on little
 * endian systems because the byte order decision is made at compile time.
 */
Bool FILE_WriteF32L(File * f, float data)
{
    I32u tmp = (*((I32u*)(void*)&data));
#if BYTE_ORDER != LITTLE_ENDIAN
    tmp = DATA_Conv32(tmp, BYTE_ORDER, LITTLE_ENDIAN);
#endif /* BYTE_ORDER == BIG_ENDIAN */
    return FILE_WriteAll(f, &tmp, 4);
}

Bool FILE_WriteF64L(File * f, double data)
{
    I64u tmp = (*((I64u*)(void*)&data));
#if BYTE_ORDER != LITTLE_ENDIAN
    tmp = DATA_Conv64(tmp, BYTE_ORDER, LITTLE_ENDIAN);
#endif /* BYTE_ORDER == BIG_ENDIAN */
    return FILE_WriteAll(f, &tmp, 8);
}

/* The same, only for reading numbers from little endian stream */
Bool FILE_ReadF32L(File * f, float * data)
{
#if BYTE_ORDER != LITTLE_ENDIAN
    I32u tmp;
    ASSERT(data);
    if (FILE_ReadAll(f, &tmp, sizeof(tmp))) {
        *((I32u*)data) = DATA_Conv32(tmp, LITTLE_ENDIAN, BYTE_ORDER);
        return True;
    }
    return False;
#else  /* BYTE_ORDER == LITTLE_ENDIAN */
    return FILE_ReadAll(f, data, sizeof(*data));
#endif /* BYTE_ORDER == LITTLE_ENDIAN */
}

Bool FILE_ReadF64L(File * f, double * data)
{
#if BYTE_ORDER != LITTLE_ENDIAN
    I64u tmp;
    ASSERT(data);
    if (FILE_ReadAll(f, &tmp, sizeof(tmp))) {
        *((I64u*)data) = DATA_Conv64(tmp, LITTLE_ENDIAN, BYTE_ORDER);
        return True;
    }
    return False;
#else  /* BYTE_ORDER == LITTLE_ENDIAN */
    return FILE_ReadAll(f, data, sizeof(*data));
#endif /* BYTE_ORDER == LITTLE_ENDIAN */
}

#endif /* __KERNEL__ */

/*
 * Support for multi-byte integers in the format described in section 5.1
 * of WBXML specification:
 *
 *   A multi-byte integer consists of a series of octets, where the most
 *   significant bit is the continuation flag and the remaining seven bits
 *   are a scalar value. The continuation flag indicates that an octet is
 *   not the end of the multi-byte sequence. A single integer value is
 *   encoded into a sequence of N octets. The first N-1 octets have the
 *   continuation flag set to a value of one (1). The final octet in the
 *   series has a continuation flag value of zero (0).
 *
 *   The remaining seven bits in each octet are encoded in a big-endian
 *   order, e.g., most significant bit first. The octets are arranged
 *   in a big-endian order, e.g., the most significant seven bits are
 *   transmitted first. In the situation where the initial octet has less
 *   than seven bits of value, all unused bits must be set to zero (0).
 *
 *   For example, the integer value 0xA0 would be encoded with the two-byte
 *   sequence 0x81 0x20. The integer value 0x60 would be encoded with the
 *   one-byte sequence 0x60
 */

#define MAX_MULTI_INT_SIZE_32 5
#define MAX_MULTI_INT_SIZE_64 10

/**
 * Returns the size of the multi-byte representation of the specified
 * 32-bit integer.
 */
int FILE_MultiByteSize32(I32u value)
{
    int n = 1;
    for (value >>= 7; value; value >>= 7) n++;
    return n;
}

/**
 * The same thing as above but for 64-bit integers. Copy/pasted for
 * efficiency reasons.
 */
int FILE_MultiByteSize64(I64u value)
{
    int n = 1;
    for (value >>= 7; value; value >>= 7) n++;
    return n;
}

/**
 * Writes the specified 32-bit integer value to the output stream
 * in multi-byte format. Returns number of bytes written to the stream,
 * or zero if an I/O error occurs.
  */
int FILE_WriteMultiByte32(File * out, I32u value)
{
    I8u buf[MAX_MULTI_INT_SIZE_32];
    int n = 1;
    buf[COUNT(buf)-1] = (I8u)(value & 0x7f);
    value >>= 7;
    while (value) {
        buf[COUNT(buf)-(++n)] = (I8u)(value | 0x80);
        value >>= 7;
    }
    if (FILE_WriteAll(out, buf + COUNT(buf) - n, n)) {
        return n;
    } else {
        return 0;
    }
}

/**
 * The same thing as above but for 64-bit integers. Copy/pasted for
 * efficiency reasons.
 */
int FILE_WriteMultiByte64(File * out, I64u value)
{
    I8u buf[MAX_MULTI_INT_SIZE_64];
    int n = 1;
    buf[COUNT(buf)-1] = (I8u)(value & 0x7f);
    value >>= 7;
    while (value) {
        buf[COUNT(buf)-(++n)] = (I8u)(value | 0x80);
        value >>= 7;
    }
    if (FILE_WriteAll(out, buf + COUNT(buf) - n, n)) {
        return n;
    } else {
        return 0;
    }
}

/**
 * Reads a 32-bit integer value in multi-byte format from the input stream.
 * Returns False on I/O error or stream format violation.
 */
Bool FILE_ReadMultiByte32(File * in, I32u * result)
{
    I32u value = 0;
    int i;
    for (i=0; i<MAX_MULTI_INT_SIZE_32; i++) {
        int c = FILE_Getc(in);
        if (c < 0) {
            return False;
        } else {
            value = (value << 7) | (c & 0x7f);
            if (!(c & 0x80)) {
                if (result) *result = value;
                return True;
            }
        }
    }
    /* stream corrupted - too many continuation bytes */
    return False;
}

/**
 * The same thing as above but for 64-bit integers. Copy/pasted for
 * efficiency reasons.
 */
Bool FILE_ReadMultiByte64(File * in, I64u * result)
{
    I64u value = 0;
    int i;
    for (i=0; i<MAX_MULTI_INT_SIZE_64; i++) {
        int c = FILE_Getc(in);
        if (c < 0) {
            return False;
        } else {
            value = (value << 7) | (c & 0x7f);
            if (!(c & 0x80)) {
                if (result) *result = value;
                return True;
            }
        }
    }
    /* stream corrupted - too many continuation bytes */
    return False;
}

#if DEBUG
/* These are here only to force compile-time control of the pointer type.
 * In the release build they are replaced with macros */
COMPILE_ASSERT(sizeof(int) == 4)
Bool FILE_ReadMultiByte32s(File * in, I32s * result)
{
    return FILE_ReadMultiByte32(in, (I32u*)result);
}
Bool FILE_ReadMultiByte64s(File * in, I64s * result)
{
    return FILE_ReadMultiByte64(in, (I64u*)result);
}
Bool FILE_ReadMultiByteInt(File * in, int * result)
{
    return FILE_ReadMultiByte32s(in, (I32s*)result);
}
Bool FILE_ReadMultiByteLong(File * in, long * result)
{
    return _FILE_ReadMultiByteLong(in, result);
}
Bool FILE_ReadMultiByteUInt(File * in, unsigned int * result)
{
    return FILE_ReadMultiByte32(in, (I32u*)result);
}
Bool FILE_ReadMultiByteULong(File * in, unsigned long * result)
{
    return _FILE_ReadMultiByteULong(in, result);
}
Bool FILE_ReadMultiByteSizeT(File * in, size_t * result)
{
    return _FILE_ReadMultiByteSizeT(in, result);
}
#endif /* DEBUG */

/*
 * HISTORY:
 *
 * $Log: s_futil.c,v $
 * Revision 1.25  2009/04/09 21:59:20  slava
 * o fixed compilation warnings (signed vs unsigned)
 *
 * Revision 1.24  2009/04/09 21:54:57  slava
 * o use size_t instead of int where it's appropriate
 *
 * Revision 1.23  2008/11/12 17:13:35  slava
 * o fixed compilation warning in FILE_ReadU64L
 *
 * Revision 1.22  2008/11/12 17:08:06  slava
 * o added signed variants of FILE_ReadI16/32/64/etc functions to avoid gcc
 *   warnings about signedness of the pointer's target
 *
 * Revision 1.21  2007/12/01 15:27:51  slava
 * o compile part of s_futil.c for kernel mode
 *
 * Revision 1.20  2007/11/30 21:29:34  slava
 * o fixed gcc 4.1.2 warning on Linux ("dereferencing type-punned pointer will
 *   break strict-aliasing rules")
 *
 * Revision 1.19  2006/11/03 17:06:49  slava
 * o special handling of size_t data type. Linux defines it as
 *   "unsigned long int" which gcc compiler considers different
 *   enough from "unsigned long" to generate tons of useless warnings.
 *
 * Revision 1.18  2006/10/21 21:33:05  slava
 * o fixed gcc complains about incompatible pointer types
 *
 * Revision 1.17  2006/10/20 22:55:59  slava
 * o fixed broken Unicode build
 *
 * Revision 1.16  2006/10/20 06:01:00  slava
 * o optimized FILE_WriteMultiByte32/64. There's no need to write data to
 *   the buffer in wrong byte order and then reverse it. Everything can be
 *   done in one step.
 *
 * Revision 1.15  2006/10/20 05:26:50  slava
 * o added set of utilities for serializing/deserializing multi-byte integers
 *
 * Revision 1.14  2006/10/20 04:40:49  slava
 * o moved FILE_ReadI32L, FILE_ReadI32B etc. from s_file.c to s_futil.c
 *
 * Revision 1.13  2006/04/13 12:15:39  slava
 * o fixed Unicode build
 *
 * Revision 1.12  2006/03/30 15:35:50  slava
 * o added FILE_Save2 function which does the same thing as FILE_Save, only
 *   allows the caller to specify the file mode (text vs binary)
 *
 * Revision 1.11  2005/08/23 23:06:51  slava
 * o modified FILE_Save behavior to NOT print any error messages in case
 *   if callback returned False, because it could be a perfectly normal
 *   situation. If an error message is appropriate, it should be issued
 *   by the callback. And, it could be more specific than what FILE_Save
 *   can produce. Also, FILE_Save needs to delete the temporary file in
 *   case if callback returns False. It didn't do that.
 *
 * Revision 1.10  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.9  2005/02/19 03:36:13  slava
 * o fixed compilation warnings produced by C++ compiler
 *
 * Revision 1.8  2005/02/19 01:12:37  slava
 * o cleanup
 *
 * Revision 1.7  2005/02/19 00:37:40  slava
 * o separated platform specific code from platform independent code
 *
 * Revision 1.6  2005/01/02 04:11:19  slava
 * o fixed a problem in FILE_FilePart which might produce unexpected results
 *   under Windows if the file name contains mixed slashes and backslashes
 *
 * Revision 1.5  2004/12/31 16:00:13  slava
 * o ported FILE_ListDir to Solaris
 * o the st_mode flags in struct stat were being checked incorrectly. Posix
 *   macros S_ISDIR, S_ISREG, etc. should be used instead of checking the
 *   individual flags, because these macros do more than just OR'ing mask
 *   with the corresponding flag.
 *
 * Revision 1.4  2004/12/31 01:27:47  slava
 * o added FILE_ListDir function
 *
 * Revision 1.3  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.2  2004/04/08 13:05:05  slava
 * o portability issues
 *
 * Revision 1.1  2003/11/30 02:49:58  slava
 * o port to Linux kernel mode environment
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
