/*
 * $Id: s_file.h,v 1.49 2010/10/23 15:55:25 slava Exp $
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

#ifndef _SLAVA_FILE_H_
#define _SLAVA_FILE_H_

#include "s_def.h"
#include "s_buf.h"
#include "s_strbuf.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/*
 * File flags
 */
#define FILE_CAN_READ       0x0001  /* caller can read from the file */
#define FILE_CAN_WRITE      0x0002  /* caller can write to the file */
#define FILE_CAN_BLOCK      0x0004  /* read can block before end of stream */
#define FILE_PUBLIC_FLAGS   0x00ff  /* flags caller is allowed to set/read */

/*
 * Flags for FILE_Zip
 */
#define FILE_ZIP_IN         0x0001  /* compress input stream */
#define FILE_ZIP_OUT        0x0002  /* compress output stream */
#define FILE_ZIP_ZHDR       0x0004  /* read and write zlib header */
#define FILE_ZIP_GZIP       0x0008  /* read and write gzip header */

#define FILE_ZIP_NONE       0       /* no compression */
#define FILE_ZIP_ALL        (FILE_ZIP_IN | FILE_ZIP_OUT)

/*
 * File is a context associted with an open file.
 * FileIO defines the set of functions that actualy do the I/O
 */
typedef struct _File File;
typedef struct _FileIO FileIO;
typedef const FileIO* IODesc;

/*
 * Set of file functions for plain file I/O
 */
extern const FileIO PlainFileIO;
#define PlainFile (&PlainFileIO)

/*
 * Set of file functions for socket I/O
 */
extern const FileIO SocketIO;
#define SocketFile (&SocketIO)

/*
 * A set of handlers for doing gzipped file I/O. Must have zlib in order 
 * to have this functionality
 */
#define GZIP_EXTENSION ".gz"
extern const FileIO GZipFileIO;
#define GZipFile (&GZipFileIO)

/* 
 * Function prototypes. The type of file I/O (i.e. gzipped or not) is 
 * defined by the set of handlers passed into FILE_Open. If FileIO 
 * parameter is NULL, the default will be used which is plain (non
 * compressed) file I/O. FILE_Connect creates a File that uses TCP
 * connection as an I/O channel.
 *
 * NOTE: these functions are not thread safe. They could be made thread 
 * safe buf that would have negative impact on performance, so I decided 
 * against it. Writing the same file by multiple threads is a relatively 
 * rare operation, and it would likely to require some synchronization 
 * anyway, which can be used to synchronize access to the file functions
 * as well.
 *
 * Another note:  FILE_Connect() takes IP address and port in host byte order.
 */
extern File * FILE_Connect  P_((IPaddr addr, Port port));
extern File * FILE_Open     P_((Str path, const char * mode, IODesc io));
extern Bool   FILE_Reopen   P_((File * f, Str path, const char * mode));
extern Bool   FILE_SetParam P_((File * f, Str name, void * value));
extern Str    FILE_Name     P_((File * f));
extern Str    FILE_GetName  P_((File * f));
extern Bool   FILE_SetName  P_((File * f, Str name));
extern int    FILE_Flags    P_((File * f));
extern void   FILE_SetFlag  P_((File * f, int flag)); /* flags |= flag */
extern void   FILE_ClrFlag  P_((File * f, int flag)); /* flags &= ~flag */
extern int    FILE_Read     P_((File * f, void * buf, int len));
extern Bool   FILE_ReadAll  P_((File * f, void * buf, int len));
extern int    FILE_Write    P_((File * f, const void * buf, int len));
extern Bool   FILE_WriteAll P_((File * f, const void * buf, int len));
extern size_t FILE_Skip     P_((File * f, size_t skip));
extern Bool   FILE_SkipAll  P_((File * f, size_t skip));
extern int    FILE_PushBack P_((File * f, const void * buf, int len));
extern int    FILE_Printf   P_((File * f, Str format, ...) PRINTF_ATTR(2,3));
extern int    FILE_VaPrintf P_((File * f, Str format, va_list va));
extern Bool   FILE_Puts     P_((File * f, Str s));
extern Bool   FILE_Gets     P_((File * f, Char * buf, size_t len));
extern Bool   FILE_Putc     P_((File * f, int c));
extern int    FILE_Getc     P_((File * f));
extern Bool   FILE_PutByte  P_((File * f, int b));
extern int    FILE_GetByte  P_((File * f));
extern Bool   FILE_Ungetc   P_((File * f, Char c));
extern Bool   FILE_Flush    P_((File * f));
extern Bool   FILE_Eof      P_((File * f));
extern int    FILE_Fd       P_((File * f));
extern int    FILE_TargetFd P_((File * f));
extern File * FILE_Target   P_((File * f));
extern void   FILE_Detach   P_((File * f));
extern void   FILE_Finish   P_((File * f));
extern void   FILE_Close    P_((File * f));
extern Bool   FILE_IsFileIO P_((File * f));
extern Bool   FILE_CanBlock P_((File * f));
extern size_t FILE_BytesRead P_((File * f));
extern size_t FILE_BytesWritten P_((File * f));

/* "substream" reads no more than maxread bytes from the underlying file */
extern File * FILE_SubStream P_((File * f, size_t maxread));
extern Bool   FSUB_Reset P_((File * sub, size_t maxread));
extern Bool   FSUB_SkipRest P_((File * sub));

/* compress/decompress the stream. */
extern File * FILE_Zip P_((File * f, int flags));
extern File * FILE_Zip2 P_((File * f, int flags, int level));
extern Bool   FILE_ZipFinish P_((File * f));

/* backward compatibility */
#define FILE_Compress(_f,_fl) FILE_Zip(_f, (_fl) | FILE_ZIP_GZIP)
#define FILE_COMPRESS_IN    FILE_ZIP_IN
#define FILE_COMPRESS_OUT   FILE_ZIP_OUT
#define FILE_COMPRESS_NONE  0
#define FILE_COMPRESS_ALL  (FILE_ZIP_IN | FILE_ZIP_OUT)

/* attach to existing file/socket */
extern File * FILE_AttachToSocket P_((Socket s));
#ifndef __KERNEL__
extern File * FILE_AttachToFile   P_((FILE * file, Str name));
#endif /* __KERNEL__ */

/* open a URL, typically http: */
extern File * FILE_OpenURL P_((Str url));
extern File * FILE_AuthURL P_((Str url, Str username, Str password));

/* dummy file - consumes all output, provides no input */
extern File * FILE_Null     P_((void));
extern Bool   FILE_IsNull   P_((const File * f));

/* in-memory I/O */
extern File * FILE_Mem      P_((void));
extern File * FILE_MemBuf   P_((Buffer * buf, Bool freeBuf));
extern File * FILE_MemIn    P_((const void * data, size_t size));
extern File * FILE_MemOut   P_((int maxsize));
extern File * FILE_MemOut2  P_((void * data, size_t size));
extern Bool   FILE_IsMem    P_((const File * f));
extern size_t FILE_MemSize  P_((const File * f));
extern void * FILE_MemData  P_((File * f));
extern void   FILE_MemClear P_((File * f));

/* splitting a stream */
extern File * FILE_Split    P_((Str name, File * f[], int n));
extern File * FILE_Split2   P_((Str name, File * f1, File * f2));

/* wrapping a text stream */
extern File * FILE_Wrap      P_((File * f, int step, int margin));
extern Bool   FILE_IsWrap    P_((const File * f));
extern Bool   WRAP_IsEnabled P_((const File * wrap));
extern Bool   WRAP_Enable    P_((File * wrap, Bool enable));
extern Bool   WRAP_Indent    P_((File * wrap, int change));
extern Bool   WRAP_SetIndent P_((File * wrap, int level));
extern int    WRAP_GetIndent P_((File * wrap));

/* utilities */
extern Str    FILE_ReadLine P_((File * in, StrBuf * sb));
extern int    FILE_ReadData P_((File * in, Buffer * out, int max));
extern int    FILE_Copy     P_((File * in, File * out));
extern int    FILE_CopyN    P_((File * in, File * out, int max));
extern void   FILE_Dump     P_((File* out, const void* buf, size_t off,
                                size_t len, size_t max));
/* handy macros */
#define FILE_AllowsReads(_f) (((FILE_Flags(_f)) & FILE_CAN_READ) != 0)
#define FILE_AllowsWrites(_f) (((FILE_Flags(_f)) & FILE_CAN_WRITE) != 0)

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_FILE_H_ */

/*
 * HISTORY:
 *
 * $Log: s_file.h,v $
 * Revision 1.49  2010/10/23 15:55:25  slava
 * o added FILE_Dump function
 *
 * Revision 1.48  2010/09/25 09:55:06  slava
 * o added PRINTF_ATTR macro which assigns printf-like characteristics to the
 *   declared function and enables gcc to check the format string against the
 *   parameters.
 *
 * Revision 1.47  2009/12/26 13:57:23  slava
 * o updated FILE_Skip to take size_t as the number of bytes to skip
 *
 * Revision 1.46  2009/12/26 12:22:40  slava
 * o working on 64-bit issues; some APIs have changed.
 *
 * Revision 1.45  2009/10/08 14:32:10  slava
 * o added FILE_Fd() and FILE_TargetFd() functions
 *
 * Revision 1.44  2009/04/09 21:54:56  slava
 * o use size_t instead of int where it's appropriate
 *
 * Revision 1.43  2009/02/09 23:02:16  slava
 * o added FILE_AuthURL function
 *
 * Revision 1.42  2008/11/10 22:26:56  slava
 * o added new function FILE_Zip2 which allows the caller to specify desired
 *   compression level of the compressor stream
 *
 * Revision 1.41  2008/09/03 12:22:21  slava
 * o added FILE_PutByte function
 *
 * Revision 1.40  2008/09/03 09:37:02  slava
 * o removed static FIO_BLOCKING_READ flag and added FILE_CAN_BLOCK flag that
 *   can be set on individual streams. This is a lot more flexible. Added
 *   FILE_CanBlock function.
 *
 * Revision 1.39  2008/09/01 09:40:16  slava
 * o added FSUB_Reset and FSUB_SkipRest functions that extend the functionality
 *   of "substreams""
 *
 * Revision 1.38  2007/03/05 02:01:57  slava
 * o added FILE_SetName() function. This function only affects the output
 *   of FILE_Name() and FILE_GetName(). It has no effect on the actual I/O.
 *
 * Revision 1.37  2007/02/07 03:58:11  slava
 * o added FILE_OpenURL function.
 *
 * Revision 1.36  2007/01/26 16:45:19  slava
 * o added FILE_GetByte function. Unlike FILE_Getc it does the same thing in
 *   both Unicode and non-Unicode builds.
 *
 * Revision 1.35  2006/10/20 04:56:44  slava
 * o cleanup. moved file related utilities (most if not all of them implemented
 *   in s_futil.c) into a separate header file, s_futil.h. This may break
 *   compilation of the sources that include individual slib header files
 *   instead of including s_lib.h
 *
 * Revision 1.34  2006/10/13 23:35:37  slava
 * o added "sub-stream" I/O which reads no more than specified number of bytes
 *   from another stream. It can be created by FILE_SubStream() function.
 *
 * Revision 1.33  2006/10/13 23:32:25  slava
 * o added FILE_GetName() function which returns NULL if doesn't find non-empty
 *   file name in the the chain of streams, unlike FILE_Name() which returns an
 *   empty string in such case.
 *
 * Revision 1.32  2006/10/13 16:14:22  slava
 * o count number of bytes written to and read from the stream
 *
 * Revision 1.31  2006/03/26 05:50:50  slava
 * o added a bunch of utilities for reading and writing binary data with byte
 *   order conversion
 *
 * Revision 1.30  2006/03/25 07:23:27  slava
 * o added FILE_ReadAll, FILE_WriteAll, FILE_SkipAll and FILE_CopyN functions
 *
 * Revision 1.29  2005/08/23 23:10:52  slava
 * o added NULL I/O object
 *
 * Revision 1.28  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.27  2003/11/30 02:49:57  slava
 * o port to Linux kernel mode environment
 *
 * Revision 1.26  2003/09/10 00:32:13  slava
 * o added FILE_MemClear and FILE_IsMem functions
 * o changed FILE_MemSize, FILE_IsWrap and WRAP_IsEnabled to take const
 *   pointer as a parameter
 *
 * Revision 1.25  2003/06/05 02:42:25  slava
 * o added FILE_PushBack and FILE_Ungetc functions
 *
 * Revision 1.24  2003/05/27 05:21:53  slava
 * o renamed FILE_Compress into FILE_Zip, added FILE_ZipFinish
 *
 * Revision 1.23  2003/05/26 18:29:33  slava
 * o fixed FILE_MemData and FILE_MemSize prototypes
 *
 * Revision 1.22  2003/01/08 17:15:43  slava
 * o removed ZIP_GetTarget and WRAP_GetTarget functions, replaced them
 *   with generic FILE_Target
 *
 * Revision 1.21  2002/12/30 21:44:10  slava
 * o added FILE_IsFileIO function
 *
 * Revision 1.20  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.19  2002/08/27 11:45:10  slava
 * o added FILE_Copy
 *
 * Revision 1.18  2002/08/01 03:45:15  slava
 * o added another parameter for FILE_ReadData - maximum number of bytes
 *   to read. If this parameter is negative, the function behaves the
 *   same way as before - reads the file until either end-of-file or
 *   out-of-memory condition.
 *
 * Revision 1.17  2002/05/22 04:22:07  slava
 * o in-memory I/O
 *
 * Revision 1.16  2002/01/25 03:55:33  slava
 * o added FILE_COMPRESS_NONE and FILE_COMPRESS_ALL defines
 *
 * Revision 1.15  2002/01/25 03:08:31  slava
 * o added FILE_Compress function
 * o no more 'compress' parameter for FILE_AttachToSocket and FILE_Connect
 *
 * Revision 1.14  2001/12/24 17:18:31  slava
 * o added FILE_Skip
 *
 * Revision 1.13  2001/12/23 21:20:00  slava
 * o _HAVE_ZLIB macro is not being used any more. If you don't have zlib
 *   or have a good reason not to use it, just don't compile s_fzip.c
 *
 * Revision 1.12  2001/12/20 10:44:31  slava
 * o port to Windows CE
 *
 * Revision 1.11  2001/10/10 16:00:31  slava
 * o added output stream "splitter"
 *
 * Revision 1.10  2001/10/09 21:09:47  slava
 * o fixed WRAP_SetIndent declaration (return type was wrong)
 *
 * Revision 1.9  2001/10/09 15:29:52  slava
 * o changed semantics of WRAP_Indent(). First, it now manipulates
 *   something that I call "indentation level". The actual number of
 *   positions is indentation level multiplied by the indentation step,
 *   the latter is defined when wrapper is created. Second, WRAP_Indent()
 *   now *changes* the current indentation level, typically by +1 or -1,
 *   rather then *sets* it. The indentation level can be set by calling
 *   WRAP_SetIndent()
 *
 * Revision 1.8  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.7  2001/04/12 07:47:45  slava
 * o added prototypes for the new "text wrapper" stream
 *
 * Revision 1.6  2001/03/11 03:22:13  slava
 * o replaced FILE_SetFlags() with FILE_SetFlag() and FILE_ClrFlag()
 *
 * Revision 1.5  2001/03/11 02:47:43  slava
 * o added user-settable flags
 *
 * Revision 1.4  2001/01/13 16:05:41  slava
 * o added FILE_VaPrintf() function
 *
 * Revision 1.3  2001/01/12 00:30:53  slava
 * o renamed FILE_Socket() into FILE_AttachToSocket()
 * o added FILE_Detach() and FILE_AttachToFile()
 *
 * Revision 1.2  2001/01/06 05:03:37  slava
 * o added support for network socket I/O via generic interface
 *
 * Revision 1.1  2001/01/03 09:18:23  slava
 * o new file I/O support for transparent access to plain or compressed
 *   files, sockets, etc.
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
