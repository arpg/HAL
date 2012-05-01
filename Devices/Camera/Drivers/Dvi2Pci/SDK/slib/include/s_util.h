/*
 * $Id: s_util.h,v 1.74 2010/12/19 17:59:16 slava Exp $
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

#ifndef _SLAVA_UTIL_H_
#define _SLAVA_UTIL_H_

#include "s_itr.h"
#include "s_buf.h"
#include "s_vector.h"
#include "s_file.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* output */
#define PRINT_ERROR     0x0001
#define PRINT_WARNING   0x0002
#define PRINT_NORMAL    0x0004
#define PRINT_VERBOSE   0x0008
#define PRINT_DEBUG     0x0010

/* Default print mask depends on whether debug trace is on */
#if DEBUG_TRACE

#  define PRINT_ALL  (PRINT_ERROR   |\
                      PRINT_WARNING |\
                      PRINT_NORMAL  |\
                      PRINT_VERBOSE |\
                      PRINT_DEBUG)

#  define DEFAULT_PRINT_MASK  PRINT_ALL

#else  /* DEBUG_TRACE */

#  define PRINT_ALL  (PRINT_ERROR   |\
                      PRINT_WARNING |\
                      PRINT_NORMAL  |\
                      PRINT_VERBOSE)

#  define DEFAULT_PRINT_MASK  (PRINT_ERROR | PRINT_WARNING | PRINT_NORMAL)

#endif /* DEBUG_TRACE */

/* Backward compatibility macros */
#define DEFAULT_TRACE_MASK DEFAULT_PRINT_MASK
#define TRACE_ALL          PRINT_ALL
#define TRACE_ERROR        PRINT_ERROR
#define TRACE_WARNING      PRINT_WARNING
#define TRACE_NORMAL       PRINT_NORMAL
#define TRACE_VERBOSE      PRINT_VERBOSE
#define TRACE_DEBUG        PRINT_DEBUG
#define TRACE_Error        PRINT_Error
#define TRACE_ErrorVa      PRINT_ErrorVa
#define TRACE_Warning      PRINT_Warning
#define TRACE_WarningVa    PRINT_WarningVa
#define TRACE_Output       PRINT_Output
#define TRACE_OutputVa     PRINT_OutputVa
#define TRACE_Verbose      PRINT_Verbose
#define TRACE_VerboseVa    PRINT_VerboseVa
#define TRACE_SetMask      PRINT_SetMask
#define TRACE_GetMask      PRINT_GetMask
#define TRACE_UseConsole   PRINT_UseConsole
#define TRACE_Dump         PRINT_Dump
#ifndef TRACE_MACROS_DEFINED
#  define TRACE_MACROS_DEFINED 1
#  define Error              PRINT_Error
#  define Warning            PRINT_Warning
#  define Output             PRINT_Output
#  define Verbose            PRINT_Verbose
#  define Dump               PRINT_Dump
#endif /* TRACE_MACROS_DEFINED */

extern void PRINT_SetMask P_((int mask));
extern int  PRINT_GetMask P_((void));

typedef int (*PrintProc) P_((Str format, ...) PRINTF_ATTR(1,2));
typedef int (*PrintVaProc) P_((Str format, va_list va));

extern int PRINT_Error P_((Str format, ... ) PRINTF_ATTR(1,2));
extern int PRINT_ErrorVa P_((Str format, va_list va));
extern int PRINT_Warning P_((Str format, ... ) PRINTF_ATTR(1,2));
extern int PRINT_WarningVa P_((Str format, va_list va));
extern int PRINT_Output P_((Str format, ... ) PRINTF_ATTR(1,2));
extern int PRINT_OutputVa P_((Str format, va_list va));
extern int PRINT_Verbose P_((Str format, ... ) PRINTF_ATTR(1,2));
extern int PRINT_VerboseVa P_((Str format, va_list va));

/* system information */
extern int  SYSTEM_CountCPU P_((void));
extern Bool IO_TermSize P_((int * rows, int * cols));

/* date/time utilities */
extern Time   TIME_Now P_((void));
extern time_t TIME_ToUnix P_((Time t));
extern Str    TIME_ToString P_((Time t));

/* socket manupulations (port and IP address are in host byte order) */
extern Bool SOCKET_New P_((int type, Socket * s));
extern Bool SOCKET_Bind P_((Socket s, IPaddr addr, Port p));
extern Bool SOCKET_Connect P_((Socket s, IPaddr addr, Port p));
extern Bool SOCKET_Block P_((Socket s, Bool block));
extern Bool SOCKET_Close P_((Socket s));
extern int  SOCKET_GetLastError P_((void));
extern int  SOCKET_Wait P_((Socket s, int mask, Time timeout));

/* Internet related utilities */
extern Bool INET_ResolveAddr P_((Str host, IPaddr * addr));

/* useful wrapper for SOCKET_New + SOCKET_Bind */
extern Bool SOCKET_Create P_((int type, IPaddr addr, Port p, Socket * s));

/* useful wrappers for SOCKET_Create() */
#define SOCKET_GetTcp(_port,_sock) SOCKET_Create(SOCK_STREAM,0,_port,_sock)
#define SOCKET_GetUdp(_port,_sock) SOCKET_Create(SOCK_DGRAM,0,_port,_sock)

/* constants used by SOCKET_Wait */
#define SOCK_WAIT_READ 0x001
#define SOCK_WAIT_WRITE 0x002
#define SOCK_WAIT_EXCEPT 0x004
#define SOCK_WAIT_ALL (SOCK_WAIT_READ | SOCK_WAIT_WRITE | SOCK_WAIT_EXCEPT)

/* 
 * useful wrappers for SOCKET_Wait()
 * Note that SOCKET_WaitRead() and SOCKET_WaitWrite() set SOCK_WAIT_EXCEPT 
 * flag, might be a little bit counter-inituitive...
 */
#define SOCKET_WaitRead(s,t)  SOCKET_Wait(s,SOCK_WAIT_EXCEPT|SOCK_WAIT_READ,t)
#define SOCKET_WaitWrite(s,t) SOCKET_Wait(s,SOCK_WAIT_EXCEPT|SOCK_WAIT_WRITE,t)
#define SOCKET_WaitAll(s,t)   SOCKET_Wait(s,SOCK_WAIT_ALL,t)

/* URL encoding */
extern Str URL_Decode P_((StrBuf * dest, Str src));
extern Str URL_Encode P_((StrBuf * dest, Str src));
extern Str URL_EncodeChars P_((StrBuf * dest, Str src, Str esc));

/* string manipulations */
extern char * STRING_Dup8 P_((const char * s));
extern wchar_t * STRING_DupU P_((const wchar_t * ws));
extern Bool STRING_StartsWith8 P_((const char * s1, const char * s2));
extern Bool STRING_StartsWithU P_((const wchar_t * s1, const wchar_t * s2));
extern Bool STRING_StartsWithNoCase8 P_((const char * s1, const char * s2));
extern Bool STRING_StartsWithNoCaseU P_((const wchar_t*s1,const wchar_t*s2));
extern Bool STRING_EndsWith8 P_((const char * s1, const char * s2));
extern Bool STRING_EndsWithU P_((const wchar_t * s1, const wchar_t * s2));
extern Bool STRING_EndsWithNoCase8 P_((const char * s1, const char * s2));
extern Bool STRING_EndsWithNoCaseU P_((const wchar_t * s1, const wchar_t * s2));
extern int STRING_IndexOf P_((Str s, Char c));
extern int STRING_LastIndexOf P_((Str s, Char c));
extern int STRING_HashCode P_((Str s));
extern int STRING_HashCodeNoCase P_((Str s));
extern char * STRING_ToMultiByte P_((const wchar_t * ws));
extern char * STRING_ToMultiByteN P_((const wchar_t * ws, size_t count));
extern wchar_t * STRING_ToUnicode P_((const char * mbs));
extern Str STRING_FormatDouble P_((StrBuf * sb, double d));
extern Str STRING_FormatFloat P_((StrBuf * sb, float d));
extern int STRING_Split P_((Str s, Vector * v, Str delim, Bool emptyOK));
extern Char * STRING_FormatVa P_((Char * buf, int size, Str fmt, va_list va));
extern Char * STRING_Format P_((Char * buf, int bufsize, Str format, ...)
                               PRINTF_ATTR(3,4));

#ifdef UNICODE
#  define STRING_Dup(s)                     STRING_DupU(s)
#  define STRING_StartsWith(s1,s2)          STRING_StartsWithU(s1,s2)
#  define STRING_StartsWithNoCase(s1,s2)    STRING_StartsWithNoCaseU(s1,s2)
#  define STRING_EndsWith(s1,s2)            STRING_EndsWithU(s1,s2)
#  define STRING_EndsWithNoCase(s1,s2)      STRING_EndsWithNoCaseU(s1,s2)
#else  /* !UNICODE */
#  define STRING_Dup(s)                     STRING_Dup8(s)
#  define STRING_StartsWith(s1,s2)          STRING_StartsWith8(s1,s2)
#  define STRING_StartsWithNoCase(s1,s2)    STRING_StartsWithNoCase8(s1,s2)
#  define STRING_EndsWith(s1,s2)            STRING_EndsWith8(s1,s2)
#  define STRING_EndsWithNoCase(s1,s2)      STRING_EndsWithNoCase8(s1,s2)
#endif /* !UNICODE */

/* UTF-8 utilities */
extern size_t UTF8_Size P_((const wchar_t * ws)); 
extern size_t UTF8_EncodeChar P_((char * utf8, size_t bufsize, wchar_t wc));
extern size_t UTF8_Encode2 P_((char* utf8, size_t bufsiz, const wchar_t* ws));
extern Bool UTF8_EncodeBuf P_((Buffer* buf, const wchar_t * ws));
extern int UTF8_CharSize P_((const char* utf8, size_t maxlen)); 
extern int UTF8_DecodeChar P_((const char* utf8, size_t* siz, wchar_t * wc));
extern size_t UTF8_Length P_((const char * utf8));
extern char * UTF8_Encode P_((const wchar_t * ws));
extern wchar_t * UTF8_Decode P_((const char * utf8));

#define BOM_CHAR 0xFEFF /* byte order mark (BOM) */
#define UTF8_MAX_SIZE 6 /* max size of a UTF-8 character */
#define UTF8_ERROR ((size_t)-1)
#define STRING_ToUTF8(_ws) UTF8_Encode(_ws)

/* dump data into the stream */
extern void PRINT_Dump P_((PrintProc p,const void* buf,size_t cb,size_t off));
extern void PRINT_Dump2 P_((PrintProc p,const void* buf,size_t cb,size_t off,
                           size_t max)); /* dump no more than max bytes */

/* parsing */
extern Bool PARSE_Bool P_((Str s, Bool * b));
extern Bool PARSE_Byte P_((Str s, char * n, int base));
extern Bool PARSE_UByte P_((Str s, unsigned char * n, int base));
extern Bool PARSE_Short P_((Str s, short * n, int base));
extern Bool PARSE_UShort P_((Str s, unsigned short * n, int base));
extern Bool PARSE_Int P_((Str s, int * n, int base));
extern Bool PARSE_UInt P_((Str s, unsigned int * n, int base));
extern Bool PARSE_Long P_((Str s, long * n, int base));
extern Bool PARSE_ULong P_((Str s, unsigned long * n, int base));
extern Bool PARSE_Double P_((Str s, double * d));
extern Bool PARSE_Float P_((Str s, float * f));
extern Bool PARSE_Long64 P_((Str s, int64_t * n, int base));
extern Bool PARSE_ULong64 P_((Str s, uint64_t * n, int base));

#define PARSE_I8(_x,_y,_z)  PARSE_Byte(_x,_y,_z)
#define PARSE_U8(_x,_y,_z)  PARSE_UByte(_x,_y,_z)
#define PARSE_I16(_x,_y,_z) PARSE_Short(_x,_y,_z)
#define PARSE_U16(_x,_y,_z) PARSE_UShort(_x,_y,_z)
#define PARSE_I32(_x,_y,_z) PARSE_Int(_x,_y,_z)
#define PARSE_U32(_x,_y,_z) PARSE_UInt(_x,_y,_z)
#define PARSE_I64(_x,_y,_z) PARSE_Long64(_x,_y,_z)
#define PARSE_U64(_x,_y,_z) PARSE_ULong64(_x,_y,_z)

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_UTIL_H_ */

/*
 * HISTORY:
 *
 * $Log: s_util.h,v $
 * Revision 1.74  2010/12/19 17:59:16  slava
 * o added UTF8_EncodeChar and UTF8_EncodeBuf functions
 *
 * Revision 1.73  2010/11/19 09:24:59  slava
 * o added STRING_FormatVa
 *
 * Revision 1.72  2010/11/19 08:50:51  slava
 * o removed yyerror from slib
 *
 * Revision 1.71  2010/09/28 19:26:21  slava
 * o 64-bit Unix build issues
 *
 * Revision 1.70  2010/09/25 09:55:06  slava
 * o added PRINTF_ATTR macro which assigns printf-like characteristics to the
 *   declared function and enables gcc to check the format string against the
 *   parameters.
 *
 * Revision 1.69  2010/07/04 13:44:25  slava
 * o UNICODE build issues
 *
 * Revision 1.68  2010/07/03 12:13:27  slava
 * o more UTF-8 stuff
 *
 * Revision 1.67  2010/07/03 10:08:53  slava
 * o fixed STRING_ToUTF8 macro
 *
 * Revision 1.66  2010/07/03 09:42:42  slava
 * o added UTF-8 utilities
 *
 * Revision 1.65  2009/05/26 06:10:21  slava
 * o define DEFAULT_PRINT_MASK based on DEBUG_TRACE rather than DEBUG value
 *
 * Revision 1.64  2008/11/20 11:52:13  slava
 * o allow use of non-UNICODE console output functions in UNICODE build
 *
 * Revision 1.63  2008/10/26 09:23:56  slava
 * [pzeldin@epiphan.com] Split SOCKET_Create into SOCKET_New and SOCKET_Bind.
 * Lets user manupulate socket in-between socket() and bind(), e.g. setsockopt()
 *
 * Revision 1.62  2007/01/27 02:42:31  slava
 * o added STRING_ToUTF8 function
 *
 * Revision 1.61  2006/11/20 18:44:18  slava
 * o added STRING_Split function
 *
 * Revision 1.60  2006/11/03 16:35:39  slava
 * o added PRINT_Dump2 which can truncate the data and provide visual
 *   indication that the data have been truncated (which is slightly
 *   different from simply using MIN(count,maxCount) as number of bytes
 *   to dump).
 *
 * Revision 1.59  2006/10/20 04:56:44  slava
 * o cleanup. moved file related utilities (most if not all of them implemented
 *   in s_futil.c) into a separate header file, s_futil.h. This may break
 *   compilation of the sources that include individual slib header files
 *   instead of including s_lib.h
 *
 * Revision 1.58  2006/10/05 17:11:00  slava
 * o added STRING_FormatFloat function
 *
 * Revision 1.57  2006/09/25 15:18:35  slava
 * o added PARSE_Float function
 *
 * Revision 1.56  2006/03/30 15:35:50  slava
 * o added FILE_Save2 function which does the same thing as FILE_Save, only
 *   allows the caller to specify the file mode (text vs binary)
 *
 * Revision 1.55  2006/03/30 06:52:17  slava
 * o added FILE_IsAbs function
 *
 * Revision 1.54  2006/03/30 02:21:44  slava
 * o added PrintVaProc typedef
 *
 * Revision 1.53  2006/03/27 21:52:33  slava
 * o STRING_DupU is now a function present in both Unicode and single-byte
 *   builds, and STRING_Dup is a macro pointing to either STRING_Dup8 or
 *   STRING_DupU depending on whether or not UNICODE macro is defined.
 *   This change is source code compatible with the older builds (but
 *   not binary compatible)
 *
 * Revision 1.52  2006/03/12 08:01:04  slava
 * o moved PRINT_Stdout and PRINT_Stderr declarations from s_def.h to s_util.h
 *
 * Revision 1.51  2005/05/31 07:02:56  slava
 * o fixed a problem with TRACE_Stdout and TRACE_Stderr macros not being
 *   defined in release build
 *
 * Revision 1.50  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.49  2005/02/19 01:18:57  slava
 * o added TRACE_Dump macro for backward compatibility
 *
 * Revision 1.48  2005/02/19 00:28:58  slava
 * o added FILE_FindSeparator, FILE_Delete and FILE_Rename functions
 *
 * Revision 1.47  2005/02/16 06:01:52  slava
 * o renamed TRACE_xxx functions and macros to PRINT_xxx. Macros are provided
 *   for backward compatibility
 *
 * Revision 1.46  2005/01/24 18:24:19  slava
 * o added STRING_ToMultiByteN function
 *
 * Revision 1.45  2005/01/23 14:01:56  slava
 * o added URL_EncodeChars function
 *
 * Revision 1.44  2005/01/02 00:09:01  slava
 * o added STRING_StartsWithNoCase and STRING_EndsWithNoCase functions which
 *   are case-insensitive versions of STRING_StartsWith and STRING_EndsWith,
 *   respectively.
 *
 * Revision 1.43  2004/12/31 01:27:47  slava
 * o added FILE_ListDir function
 *
 * Revision 1.42  2004/07/19 22:55:11  slava
 * o moved BASE64 encoding functions from s_util to s_base64 module
 *
 * Revision 1.41  2004/03/15 18:52:45  slava
 * o added STRING_FormatDouble which formats a double value into the shortest
 *   possible %f style string without losing precision.
 *
 * Revision 1.40  2003/10/13 18:18:29  slava
 * o added TRACE_OutputVa, TRACE_VerboseVa, TRACE_WarningVa and TRACE_ErrorVa
 *
 * Revision 1.39  2003/07/11 15:04:13  slava
 * o renamed Error, Warning, Output, Verbose and Dump functions into
 *   TRACE_Error, TRACE_Warning, TRACE_Output, TRACE_Verbose and TRACE_Dump,
 *   respecively. The old names were too likely to get into conflict with
 *   non-slib symbols if slib is being integrated into a large project
 *   (which is exactly what has just happened). For backward compatibility,
 *   the old names (Error, Warning, Output, Verbose and Dump) are still
 *   defined as macros which can be disabled by defining TRACE_MACROS_DEFINED
 *   prior to including slib headers.
 *
 * Revision 1.38  2003/03/12 05:42:25  slava
 * o renamed SOCKET_SetBlocking to SOCKET_Block
 *
 * Revision 1.37  2003/02/06 20:12:31  slava
 * o added SOCKET_SetBlocking
 *
 * Revision 1.36  2003/02/05 00:47:30  slava
 * o added STRING_IndexOf and STRING_LastIndexOf
 *
 * Revision 1.35  2003/01/02 02:44:29  slava
 * o added IO_TermSize function
 *
 * Revision 1.34  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.33  2002/10/21 05:20:35  slava
 * o added FILE_IsFile function
 *
 * Revision 1.32  2002/08/12 05:13:27  slava
 * o use (void) rather than () to declare functions that have
 *   no arguments. The problem with () is that compilers interpret
 *   it as an *unspecified* argument list, but (void) clearly
 *   indicates that function takes no arguments. Improves compile
 *   time error checking
 *
 * Revision 1.31  2002/07/08 07:12:37  slava
 * o added URL_Encode and URL_Decode
 *
 * Revision 1.30  2002/07/07 09:13:47  slava
 * o added BASE64_Encode and BASE64_Decode
 *
 * Revision 1.29  2002/07/01 04:43:10  slava
 * o renamed TIME_Convert to TIME_ToUnix
 *
 * Revision 1.28  2002/05/29 05:14:14  slava
 * o exported TRACE_Stdout and TRACE_Stderr functions
 *
 * Revision 1.27  2001/12/23 18:37:39  slava
 * o added STRING_ToMultiByte and STRING_ToUnicode
 *
 * Revision 1.26  2001/12/20 10:44:32  slava
 * o port to Windows CE
 *
 * Revision 1.25  2001/12/20 02:47:33  slava
 * o added STRING_StartsWith and STRING_EndsWith
 *
 * Revision 1.24  2001/10/18 00:01:26  slava
 * o fixed PARSE_Byte prototype
 *
 * Revision 1.23  2001/10/17 05:13:27  slava
 * o added FILE_IsFileSeparator
 *
 * Revision 1.22  2001/10/08 05:17:52  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.21  2001/06/27 01:56:34  slava
 * o added STRING_HashCode() and STRING_HashCodeNoCase() functions
 *
 * Revision 1.20  2001/06/12 08:53:08  slava
 * o added TRACE_UseConsole() - most useful on Windows CE where console
 *   output is disabled by default.
 *
 * Revision 1.19  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.18  2001/05/18 22:29:54  slava
 * o slib has been (partially) ported to Windows CE
 *
 * Revision 1.17  2001/03/17 07:42:42  slava
 * o added SOCKET_Connect() function
 *
 * Revision 1.16  2001/03/13 06:04:37  slava
 * o added PARSE_Byte and PARSE_UByte() functions
 *
 * Revision 1.15  2001/01/31 02:00:41  slava
 * o portability fixes. Now compiles on Solaris
 *
 * Revision 1.14  2001/01/23 13:26:17  slava
 * o define TRACE_ALL differently in debug and release builds to reflect
 *   the fact that TRACE_DEBUG is unavailable in release build
 *
 * Revision 1.13  2001/01/13 16:05:00  slava
 * o added FILE_FilePart() function
 * o FileSaveCB callback now gets the destination file name rather than
 *   the temporary one. The temporary file name can be queried from the
 *   File object, if needed.
 *
 * Revision 1.12  2001/01/06 04:57:05  slava
 * o added INET_ResolveAddr() function
 *
 * Revision 1.11  2001/01/03 09:18:23  slava
 * o new file I/O support for transparent access to plain or compressed
 *   files, sockets, etc.
 *
 * Revision 1.10  2000/12/17 16:09:47  slava
 * o added extern "C" {} so that slib could be used from a C++ program
 *
 * Revision 1.9  2000/11/17 05:28:49  slava
 * o moved TEMP_FILE_NAME_LEN to header file
 * o made FILE_MakeUnique() public
 * o a bunch of new file manipulation functions: FILE_Exist, FILE_NonExist,
 *   FILE_IsDir, FILE_MkDir, FILE_RmDir and FILE_List
 *
 * Revision 1.8  2000/11/05 07:07:49  slava
 * o reorganization of output functions (Verbose(), Output(), etc.)
 *
 * Revision 1.7  2000/09/08 04:12:37  slava
 * o added trace mask so that trace level can be configured at run time
 *
 * Revision 1.6  2000/09/03 00:41:03  slava
 * o some network-related changes - IPaddr is now a 32-bit integer, added
 *   byte order definitions for Win32
 *
 * Revision 1.5  2000/09/01 11:41:32  slava
 * o added SOCKET_Wait()
 *
 * Revision 1.4  2000/08/31 05:26:34  slava
 * o added SOCKET_GetLastError
 *
 * Revision 1.3  2000/08/26 12:11:45  slava
 * o added SOCKET_Create() and SOCKET_Close() functions
 *
 * Revision 1.2  2000/08/25 12:01:23  slava
 * o added FILE_CanOpen(), FILE_CanRead() and FILE_CanWrite() definitions
 *
 * Revision 1.1  2000/08/19 04:48:59  slava
 * o initial checkin
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
