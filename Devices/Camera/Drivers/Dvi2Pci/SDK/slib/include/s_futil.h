/*
 * $Id: s_futil.h,v 1.9 2010/10/01 16:22:49 slava Exp $
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

#ifndef _SLAVA_FUTIL_H_
#define _SLAVA_FUTIL_H_

#include "s_itr.h"
#include "s_file.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* file manupulations */
#define TEMP_FILE_NAME_LEN  8

typedef Bool (*FileSaveCB)P_((File* out, Str fname, void * ctx));
typedef Bool (*FileListCB)P_((Str dir, Str fname, void * ctx));

/*
 * Directory entry type enumeration. On Windows, you may expect DTypeFile
 * or DTypeDir, on other systems - anything else, including DTypeUnknown.
 * A pointer to the DirEntry structure is returned by the ITR_Next function
 * of the iterator created by FILE_ListDir.
 */
typedef enum _DirType {
    DTypeUnknown,       /* unknown entry type */
    DTypeFile,          /* a regular file */
    DTypeDir,           /* a directory */
    DTypePipe,          /* a named pipe */
    DTypeLink,          /* a symbolic link */
    DTypeSocket,        /* a socket */
    DTypeChar,          /* a character device */
    DTypeBlock          /* a block device */
} DirType;
typedef struct _DirEntry {
    Str dir;            /* the name of the directory containing the entry */
    Str name;           /* the name of the entry within the directory */
    DirType type;       /* the entry type, e.g. DTypeFile or DTypeDir */
} DirEntry;

extern Char * FILE_DirName P_((Str path, int extra));
extern Char * FILE_TempName P_((Char * buf, size_t bufsize));
extern Bool FILE_IsFileSeparator P_((Char c));
extern Bool FILE_Save P_((Str fname, FileSaveCB cb, void * ctx, IODesc io));
extern Bool FILE_Save2 P_((Str fn,FileSaveCB cb,void* ctx,Bool txt,IODesc io));
extern Str  FILE_FilePart P_((Str path));
extern int  FILE_FindSeparator P_((Str s));
extern Bool FILE_CanOpen P_((Str fname, const char * flags));
extern I64s FILE_Size P_((Str fname));
extern Bool FILE_Exist P_((Str fname));
extern Bool FILE_NonExist P_((Str fname));
extern Bool FILE_IsFile P_((Str fname));
extern Bool FILE_IsDir P_((Str fname));
extern Bool FILE_IsAbs P_((Str fname));
extern void FILE_MakeUnique P_((Char* name, size_t fixPart, size_t randPart));
extern Bool FILE_MkDir P_((Str dir));
extern Bool FILE_RmDir P_((Str dir, Bool recurse));
extern Bool FILE_Delete P_((Str fname));
extern Bool FILE_Rename P_((Str oldn, Str newn));
extern int  FILE_List P_((Str dir, FileListCB cb, void * ctx));
extern Iterator * FILE_ListDir P_((Str dir));

#define FILE_CanRead(_fname)  FILE_CanOpen(_fname, "r")
#define FILE_CanWrite(_fname) FILE_CanOpen(_fname, "w")

/* utilities for reading binary data with byte order convertion */
extern Bool   FILE_ReadU16  P_((File * f, I16u * data, int endianess));
extern Bool   FILE_ReadU32  P_((File * f, I32u * data, int endianess));
extern Bool   FILE_ReadU64  P_((File * f, I64u * data, int endianess));
extern Bool   FILE_ReadU16B P_((File * f, I16u * data)); /* big endian */
extern Bool   FILE_ReadU32B P_((File * f, I32u * data)); /* big endian */
extern Bool   FILE_ReadU64B P_((File * f, I64u * data)); /* big endian */
extern Bool   FILE_ReadU16L P_((File * f, I16u * data)); /* little endian */
extern Bool   FILE_ReadU32L P_((File * f, I32u * data)); /* little endian */
extern Bool   FILE_ReadU64L P_((File * f, I64u * data)); /* little endian */
extern Bool   FILE_ReadF32  P_((File * f, float * data, int endianess));
extern Bool   FILE_ReadF64  P_((File * f, double * data, int endianess));
extern Bool   FILE_ReadF32B P_((File * f, float * data));  /* big endian */
extern Bool   FILE_ReadF64B P_((File * f, double * data)); /* big endian */
extern Bool   FILE_ReadF32L P_((File * f, float * data));  /* little endian */
extern Bool   FILE_ReadF64L P_((File * f, double * data)); /* little endian */

#if DEBUG
/* this is for better type safety in debug build */
extern Bool   FILE_ReadI16  P_((File * f, I16s * data, int endianess));
extern Bool   FILE_ReadI32  P_((File * f, I32s * data, int endianess));
extern Bool   FILE_ReadI64  P_((File * f, I64s * data, int endianess));
extern Bool   FILE_ReadI16B P_((File * f, I16s * data)); /* big endian */
extern Bool   FILE_ReadI32B P_((File * f, I32s * data)); /* big endian */
extern Bool   FILE_ReadI64B P_((File * f, I64s * data)); /* big endian */
extern Bool   FILE_ReadI16L P_((File * f, I16s * data)); /* little endian */
extern Bool   FILE_ReadI32L P_((File * f, I32s * data)); /* little endian */
extern Bool   FILE_ReadI64L P_((File * f, I64s * data)); /* little endian */
#else /* !DEBUG */
#  define FILE_ReadI16(f,i,e)           FILE_ReadU16(f,(I16u*)(i),e)
#  define FILE_ReadI32(f,i,e)           FILE_ReadU32(f,(I32u*)(i),e)
#  define FILE_ReadI64(f,i,e)           FILE_ReadU64(f,(I64u*)(i),e)
#  define FILE_ReadI16B(f,i)            FILE_ReadU16B(f,(I16u*)(i))
#  define FILE_ReadI32B(f,i)            FILE_ReadU32B(f,(I32u*)(i))
#  define FILE_ReadI64B(f,i)            FILE_ReadU64B(f,(I64u*)(i))
#  define FILE_ReadI16L(f,i)            FILE_ReadU16L(f,(I16u*)(i))
#  define FILE_ReadI32L(f,i)            FILE_ReadU32L(f,(I32u*)(i))
#  define FILE_ReadI64L(f,i)            FILE_ReadU64L(f,(I64u*)(i))
#endif /* !DEBUG */

/* utilities for writing binary data with byte order convertion */
extern Bool   FILE_WriteI16  P_((File * f, I16u data, int endianess));
extern Bool   FILE_WriteI32  P_((File * f, I32u data, int endianess));
extern Bool   FILE_WriteI64  P_((File * f, I64u data, int endianess));
extern Bool   FILE_WriteI16B P_((File * f, I16u data)); /* big endian */
extern Bool   FILE_WriteI32B P_((File * f, I32u data)); /* big endian */
extern Bool   FILE_WriteI64B P_((File * f, I64u data)); /* big endian */
extern Bool   FILE_WriteI16L P_((File * f, I16u data)); /* little endian */
extern Bool   FILE_WriteI32L P_((File * f, I32u data)); /* little endian */
extern Bool   FILE_WriteI64L P_((File * f, I64u data)); /* little endian */
extern Bool   FILE_WriteF32  P_((File * f, float data, int endianess));
extern Bool   FILE_WriteF64  P_((File * f, double data, int endianess));
extern Bool   FILE_WriteF32B P_((File * f, float data));  /* big endian */
extern Bool   FILE_WriteF64B P_((File * f, double data)); /* big endian */
extern Bool   FILE_WriteF32L P_((File * f, float data));  /* little endian */
extern Bool   FILE_WriteF64L P_((File * f, double data)); /* little endian */

/*
 * support for multi-byte integers in the format described in section 5.1
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
extern int FILE_MultiByteSize32 P_((I32u value));
extern int FILE_MultiByteSize64 P_((I64u value));
extern int FILE_WriteMultiByte32 P_((File * out, I32u value));
extern int FILE_WriteMultiByte64 P_((File * out, I64u value));
extern Bool FILE_ReadMultiByte32 P_((File * in, I32u * result));
extern Bool FILE_ReadMultiByte64 P_((File * in, I64u * result));

#if DEBUG
/* this is for better type safety in debug build */
extern Bool FILE_ReadMultiByte32s P_((File * in, I32s * result));
extern Bool FILE_ReadMultiByte64s P_((File * in, I64s * result));
extern Bool FILE_ReadMultiByteInt P_((File * in, int * result));
extern Bool FILE_ReadMultiByteLong P_((File * in, long * result));
extern Bool FILE_ReadMultiByteUInt P_((File * in, unsigned int * result));
extern Bool FILE_ReadMultiByteULong P_((File * in, unsigned long * result));
extern Bool FILE_ReadMultiByteSizeT P_((File * in, size_t * result));
#else /* !DEBUG */
#  define FILE_ReadMultiByte32s(f,i)      FILE_ReadMultiByte32(f,(I32u*)(i))
#  define FILE_ReadMultiByte64s(f,i)      FILE_ReadMultiByte64(f,(I64u*)(i))
#  define FILE_ReadMultiByteInt(f,i)      FILE_ReadMultiByte32s(f,i)
#  define FILE_ReadMultiByteUInt(f,i)     FILE_ReadMultiByte32(f,i)
#  define FILE_ReadMultiByteLong(f,l)    _FILE_ReadMultiByteLong(f,l)
#  define FILE_ReadMultiByteULong(f,l)   _FILE_ReadMultiByteULong(f,l)
#  define FILE_ReadMultiByteSizeT(f,l)   _FILE_ReadMultiByteULong(f,l)
#endif /* !DEBUG */

#define FILE_MultiByteSizeInt(i)          FILE_MultiByteSize32(i)
#define FILE_WriteMultiByteInt(f,i)       FILE_WriteMultiByte32(f,i)
#ifdef __LONG_64__
#  define FILE_MultiByteSizeLong(l)       FILE_MultiByteSize64(l)
#  define FILE_MultiByteSizeSizeT(l)      FILE_MultiByteSize64(l)
#  define FILE_WriteMultiByteLong(f,l)    FILE_WriteMultiByte64(f,l)
#  define FILE_WriteMultiByteSizeT(f,l)   FILE_WriteMultiByte64(f,l)
#  define _FILE_ReadMultiByteLong(f,l)    FILE_ReadMultiByte64(f,(I64u*)(l))
#  define _FILE_ReadMultiByteULong(f,l)   FILE_ReadMultiByte64(f,(I64u*)(l))
#  define _FILE_ReadMultiByteSizeT(f,l)   FILE_ReadMultiByte64(f,(I64u*)(l))
#else  /* !__LONG_64__ */
#  define FILE_MultiByteSizeLong(l)       FILE_MultiByteSize32(l)
#  define FILE_MultiByteSizeSizeT(l)      FILE_MultiByteSize32(l)
#  define FILE_WriteMultiByteLong(f,l)    FILE_WriteMultiByte32(f,l)
#  define FILE_WriteMultiByteSizeT(f,l)   FILE_WriteMultiByte32(f,l)
#  define _FILE_ReadMultiByteLong(f,l)    FILE_ReadMultiByte32(f,(I32u*)(l))
#  define _FILE_ReadMultiByteULong(f,l)   FILE_ReadMultiByte32(f,(I32u*)(l))
#  define _FILE_ReadMultiByteSizeT(f,l)   FILE_ReadMultiByte32(f,(I32u*)(l))
#endif /* !__LONG_64__ */

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_FUTIL_H_ */

/*
 * HISTORY:
 *
 * $Log: s_futil.h,v $
 * Revision 1.9  2010/10/01 16:22:49  slava
 * o added FILE_Size function
 *
 * Revision 1.8  2009/04/09 21:54:56  slava
 * o use size_t instead of int where it's appropriate
 *
 * Revision 1.7  2008/11/12 17:29:13  slava
 * o fixed release build
 *
 * Revision 1.6  2008/11/12 17:08:06  slava
 * o added signed variants of FILE_ReadI16/32/64/etc functions to avoid gcc
 *   warnings about signedness of the pointer's target
 *
 * Revision 1.5  2006/11/03 17:06:49  slava
 * o special handling of size_t data type. Linux defines it as
 *   "unsigned long int" which gcc compiler considers different
 *   enough from "unsigned long" to generate tons of useless warnings.
 *
 * Revision 1.4  2006/10/21 21:33:05  slava
 * o fixed gcc complains about incompatible pointer types
 *
 * Revision 1.3  2006/10/20 07:07:11  slava
 * o added a few multi-byte related macros, like FILE_MultiByteSizeInt
 *
 * Revision 1.2  2006/10/20 05:26:50  slava
 * o added set of utilities for serializing/deserializing multi-byte integers
 *
 * Revision 1.1  2006/10/20 04:56:44  slava
 * o cleanup. moved file related utilities (most if not all of them implemented
 *   in s_futil.c) into a separate header file, s_futil.h. This may break
 *   compilation of the sources that include individual slib header files
 *   instead of including s_lib.h
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
