/*
 * $Id: s_win32.h,v 1.47 2010/07/04 16:14:30 slava Exp $
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

#ifndef _SLAVA_WIN32_H_
#define _SLAVA_WIN32_H_ 1

#ifndef _WIN32
#  error "Do not include s_win32.h directly. Include s_os.h"
#endif

#define _TLS_SUPPORT 1

/* make sure that UNICODE and _UNICODE are both defined, or both undefined */
#ifdef _UNICODE
#  ifndef UNICODE
#    define UNICODE     /* _UNICODE was defined but UNICODE was not */
#  endif /* UNICODE */
#else
#  ifdef UNICODE
#    define _UNICODE    /* UNICODE was defined but _UNICODE was not */
#  endif /* UNICODE */
#endif /* _UNICODE */

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <ctype.h>

/* these includes are present on all known platforms except Windiws CE */
#ifndef _WIN32_WCE
#include <stddef.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#endif /* _WIN32_WCE */

/* this is necessary to eliminate some conflicts */
#define Random Win32Random
#define __RPCASYNC_H__

/* ensure that windows.h is sucked in */
#ifndef WIN32_LEAN_AND_MEAN
#  define WIN32_LEAN_AND_MEAN 1
#endif /* WIN32_LEAN_AND_MEAN */

#ifndef NOCRYPT
#  define NOCRYPT
#endif /* NOCRYPT */

#include <windows.h>
#include <winsock.h>

/* undo the damage done to slib namespace */
#undef ASSERT
#undef VERIFY
#undef Random

/* value returned by GetFileAttributes() that indicates failure */
#define FILE_ATTRIBUTE_FAILURE 0xffffffff

/* _stat() definitions */
#ifndef _WIN32_WCE
#  include <sys/types.h>
#  include <sys/stat.h>
#endif /* _WIN32_WCE */

/* _mkdir() is defined there */
#ifndef _WIN32_WCE
#  include <direct.h>
#endif /* _WIN32_WCE */

/* TLS_OUT_OF_INDEXES may not be defined  */
#ifndef TLS_OUT_OF_INDEXES
#  define TLS_OUT_OF_INDEXES (DWORD)0xFFFFFFFF
#endif /* TLS_OUT_OF_INDEXES */

/*
 * Disable warning C4996 (use of deprecated functions). Recent versions of
 * Microsoft Visual C++ consider a bunch of standard C Runtime functions,
 * such as strcat and strcpy, deprecated. Disable that.
 */
#pragma warning(disable: 4996)

/* 
 * __CW32__ in defined when the code is being build with CodeWarrior.
 * CodeWarrior for Symbian comes with its own set of Win32 header
 * files which are slightly different from those distributed by 
 * Microsoft. Also, the Metrowerks compiler does not support some
 * of the Microsoft's keywords.
 */
#ifdef __CW32__
#  define _WCTYPE_T_DEFINED
#  define _I64_MAX      __INT64_C(9223372036854775807)
#  define _I64_MIN      (_I64_MAX - 1)
#  define _UI64_MAX     __UINT64_C(0xffffffffffffffff)
#  ifndef _CRTIMP
#    define _CRTIMP
#  endif /* _CRTIMP */
_CRTIMP wchar_t * __cdecl _wctime(const time_t *);
_CRTIMP FILE * __cdecl _wfopen(const wchar_t *, const wchar_t *);
_CRTIMP FILE * __cdecl _wfreopen(const wchar_t *, const wchar_t *, FILE *);
extern int _cw_swprintf(wchar_t *, const wchar_t *, ...);
#endif /* __CW32__ */

#include <tchar.h>

#undef  far
#undef  near
#undef  FAR
#undef  NEAR
#define far
#define near
#define FAR
#define NEAR

/* character type */
#ifdef _UNICODE
typedef WCHAR Char;
#  define U_(u,a) u
#  define UNICODE_ONLY_(x) x
#  define ANSI_ONLY_(x)
#else  /* !_UNICODE */
typedef char Char;
#  define U_(u,a) a
#  define UNICODE_ONLY_(x)
#  define ANSI_ONLY_(x)  x
#endif /* !_UNICODE */

#define TEXT_(quote) TEXT(quote)

/* string manipulation functions */
#define StrLen      _tcslen
#define StrCmp      _tcscmp
#define StrnCmp     _tcsncmp
#define StrCpy      _tcscpy
#define StrCat      _tcscat
#define StrChr      _tcschr
#define StrStr      _tcsstr
#define StrrChr     _tcsrchr
#define StrnCpy     _tcsncpy
#define StrpBrk     _tcspbrk
#define StrTod      _tcstod
#define StrTol      _tcstol
#define StrToul     _tcstoul
#define StrCaseCmp  _tcsicmp
#define StrnCaseCmp _tcsnicmp
#define Snprintf    _sntprintf
#define Vsnprintf   _vsntprintf
#define Vfprintf    _vftprintf

/* CodeWarrior defines swprintf incorrectly */
#if defined(__CW32__) && defined(UNICODE)
#define Sprintf     _cw_swprintf
#else  /* !(__CW32__ && UNICODE) */
#define Sprintf     _stprintf
#endif /* !(__CW32__ && UNICODE) */

/* character classification functions */
#define IsUpper     _istupper
#define IsLower     _istlower
#define IsPrint     _istprint
#define IsDigit     _istdigit
#define IsXdigit    _istxdigit
#define IsAlpha     _istalpha
#define IsSpace     _istspace
#define ToUpper     _totupper
#define ToLower     _totlower

/* other functions */
#define Ctime       _tctime
#define Fopen       _tfopen
#define Freopen     _tfreopen

/* redefine Winsock-style shutdown() constants in Unix style */
#define SHUT_RD   SD_RECEIVE
#define SHUT_WR   SD_SEND
#define SHUT_RDWR SD_BOTH  

#ifndef SD_BOTH
#  define SD_RECEIVE 0x00
#  define SD_SEND    0x01
#  define SD_BOTH    0x02
#endif /* SD_BOTH */

/* define these types the way they are defined on Unix platforms */
typedef __int8 int8_t;
typedef unsigned __int8 uint8_t;
typedef __int16 int16_t;
typedef unsigned __int16 uint16_t;
typedef __int32 int32_t;
typedef unsigned __int32 uint32_t;
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
typedef int pid_t;
typedef int socklen_t;

/* 
 * and these are their Win32-style equivalents. Check for ADDRESS_TAG_BIT
 * is an attempt to detect recent Platform SDKs which define these missing
 * types.
 */
#ifndef _WIN32_WCE
#  ifndef ADDRESS_TAG_BIT
#    ifndef MISSING_WIN32_TYPES_DEFINED
#      define MISSING_WIN32_TYPES_DEFINED
typedef int8_t       INT8;
typedef uint8_t      UINT8;
typedef int16_t      INT16;
typedef uint16_t     UINT16;
#    endif /* !MISSING_WIN32_TYPES_DEFINED */
#  endif /* !ADDRESS_TAG_BIT */
typedef const void * PCVOID;
#endif /* !_WIN32_WCE */

#define INT8_MIN   _I8_MIN
#define INT8_MAX   _I8_MAX
#define UINT8_MAX  _UI8_MAX

#define INT16_MIN   _I16_MIN
#define INT16_MAX   _I16_MAX
#define UINT16_MAX  _UI16_MAX

#define INT32_MIN   _I32_MIN
#define INT32_MAX   _I32_MAX
#define UINT32_MAX  _UI32_MAX

#ifdef __CW32__
#  define __INT64_C(c)  ((I64s)(c))
#  define __UINT64_C(c) ((I64u)(c))
#else  /* !__CW32__ */
#  define __INT64_C(c)  c ## i64 
#  define __UINT64_C(c) c ## ui64
#endif /* !__CW32__ */

#define INT64_MIN  _I64_MIN
#define INT64_MAX  _I64_MAX
#define UINT64_MAX _UI64_MAX

#define LONG_HEX_FORMAT "%I64X"
#define LONG_LONG_FORMAT "%I64d"
#define LONG_ULONG_FORMAT "%I64u"

#define I64X_FORMAT LONG_HEX_FORMAT
#define I64S_FORMAT LONG_LONG_FORMAT
#define I64U_FORMAT LONG_ULONG_FORMAT

#define stat              _stat
#define fstat             _fstat
#define getcwd            _getcwd
#define chdir             _chdir
#define rmdir             _rmdir
#define unlink            _unlink
#define mkdir(_dir,_perm) _mkdir(_dir)
#define tzset             _tzset
#define tzname            _tzname
#define sleep(_n)         Sleep(1000*(_n))
#define strdup            _strdup
#define strupr            _strupr
#define strrev            _strrev
#define stricmp           _stricmp
#define strnicmp          _strnicmp
#define snprintf          _snprintf
#define vsnprintf         _vsnprintf
#define strcasecmp        _stricmp
#define strncasecmp       _strnicmp
#define getpid()          ((pid_t)GetCurrentProcessId())

#ifndef _POSIX_
#  define fileno          _fileno
#  define fdopen          _fdopen
#endif /*  _POSIX_ */

#ifndef __CW32__
#  define S_IFMT   _S_IFMT
#  define S_IFDIR  _S_IFDIR
#  define S_IFCHR  _S_IFCHR
#  define S_IFREG  _S_IFREG
#  define S_IREAD  _S_IREAD
#  define S_IWRITE _S_IWRITE
#  define S_IEXEC  _S_IEXEC
#endif /* !__CW32__ */

#define READ_TEXT_MODE "rt"
#define WRITE_TEXT_MODE "wt"
#define APPEND_TEXT_MODE "a+t"
#define READ_BINARY_MODE "rb"
#define WRITE_BINARY_MODE "wb"
#define APPEND_BINARY_MODE "a+b"

#define FILE_SEPARATOR "\\"
#define FILE_SEPARATOR_CHAR '\\'
#define PATH_SEPARATOR ";"
#define PATH_SEPARATOR_CHAR ';'

typedef SOCKET Socket;

/* 
 * Definitions for byte order, according to significance of bytes, from low
 * addresses to high addresses.  The value is what you get by putting '4'
 * in the most significant byte, '3' in the second most significant byte,
 * '2' in the second least significant byte, and '1' in the least
 * significant byte.  
 */

#define __LITTLE_ENDIAN 1234
#define __BIG_ENDIAN    4321
#define __PDP_ENDIAN    3412

#define LITTLE_ENDIAN   __LITTLE_ENDIAN
#define BIG_ENDIAN      __BIG_ENDIAN
#define PDP_ENDIAN      __PDP_ENDIAN
#define BYTE_ORDER      __BYTE_ORDER

/* if byte order is not defined, default to little endian, like x86 */
#if defined(_X86_) || !defined (__BYTE_ORDER)
#  define __BYTE_ORDER __LITTLE_ENDIAN
#endif 

/* the word that has same size as a pointer. */
#ifdef _WIN64
typedef uint64_t PtrWord;
#else  /* ! _WIN64 */
typedef uint32_t PtrWord;
#endif /* ! _WIN64 */

/* memory allocation primitives provided by the system */
#define OS_MemAlloc(_size) malloc(_size)
#define OS_MemRealloc(_ptr,_size) realloc((_ptr),(_size))
#define OS_MemFree(_ptr) free(_ptr)

#endif /* _SLAVA_WIN32_H_ */

/*
 * HISTORY:
 *
 * $Log: s_win32.h,v $
 * Revision 1.47  2010/07/04 16:14:30  slava
 * o moved T_ macro from s_win32.h to s_def.h
 *
 * Revision 1.46  2010/07/03 11:48:26  slava
 * o added U_, UNICODE_ONLY_ and ANSI_ONLY_ macros
 *
 * Revision 1.45  2010/07/03 09:51:41  slava
 * o added T_ macro (shorter version of TEXT_)
 *
 * Revision 1.44  2009/12/02 07:27:36  slava
 * o removed definition of UINT32. It produces warning C4142 (benign
 *   redefinition of type) even on quite ancient SDKs such as the one
 *   that comes with VC6.
 *
 * Revision 1.43  2009/10/17 10:45:42  slava
 * o define far and near macros
 *
 * Revision 1.42  2009/10/08 14:30:52  slava
 * o defined fileno and fdopen macros
 *
 * Revision 1.41  2009/05/27 21:47:50  slava
 * o use builtin types __int8, __int16 and __int32 to define Unix-like types
 *   int8_t, int16_t, int32_t etc.
 *
 * Revision 1.40  2008/11/12 17:36:10  slava
 * o defined socklen_t
 *
 * Revision 1.39  2007/12/01 15:30:04  slava
 * o attempt to improve compatibility with newer Platform SDKs
 *
 * Revision 1.38  2006/11/23 00:22:10  slava
 * o added Snprintf macro
 *
 * Revision 1.37  2006/10/21 21:43:16  slava
 * o PtrWord should be unsigned
 *
 * Revision 1.36  2006/08/26 05:18:12  slava
 * o Windows: disable warning C4996 (use of deprecated functions)
 *
 * Revision 1.35  2006/02/23 01:35:03  slava
 * o CodeWarrior 3.1 compilation issues
 *
 * Revision 1.34  2005/02/25 02:53:39  slava
 * o TLS code moved to platform specific area
 *
 * Revision 1.33  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.32  2005/01/11 23:30:20  slava
 * o better definition for the Char type. Turns some link errors into
 *   compilation errors.
 *
 * Revision 1.31  2005/01/01 23:06:09  slava
 * o added StrnCmp macro
 *
 * Revision 1.30  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.29  2003/08/28 17:16:23  slava
 * o replaced tabs with spaces
 *
 * Revision 1.28  2003/07/11 05:28:20  slava
 * o including <windows.h> followed by <winsock.h> seems to solve some
 *   strange compilation problems. I'm not sure why...
 *
 * Revision 1.27  2003/03/18 14:47:59  slava
 * o make sure that UNICODE and _UNICODE are either both defined, or both
 *   undefined. Other combinations don't work
 *
 * Revision 1.26  2003/02/24 21:31:26  slava
 * o include <winsock.h> because including <winsock2.h> makes it virtually
 *   impossible to use slib in an MFC program (yuck!)
 *
 * Revision 1.25  2003/02/03 21:56:14  slava
 * o added StrpBrk macro
 *
 * Revision 1.24  2002/09/06 04:53:20  slava
 * o added pid_t typedef and getpid() macro
 *
 * Revision 1.23  2002/07/09 08:32:11  slava
 * o added PCVOID definition for non-WinCE platforms
 *
 * Revision 1.22  2002/01/22 05:00:42  slava
 * o defined tzset and tzname
 *
 * Revision 1.21  2002/01/21 07:38:45  slava
 * o added strdup, strupr, strrev, stricmp and strnicmp macros which point
 *   to _strdup, _strupr, _strrev and so on...
 *
 * Revision 1.20  2002/01/02 17:18:36  slava
 * o added StrStr macro
 *
 * Revision 1.19  2001/12/23 13:28:00  slava
 * o added READ_BINARY_MODE, WRITE_BINARY_MODE and APPEND_BINARY_MODE defines
 *
 * Revision 1.18  2001/12/20 10:44:32  slava
 * o port to Windows CE
 *
 * Revision 1.17  2001/12/06 04:31:12  slava
 * o added LOCK_UnlockMany() function
 * o changed LockAccess enum type to avoid conflict with LOCK_WRITE constant
 *   defined in one of Windows header files
 *
 * Revision 1.16  2001/12/01 05:18:05  slava
 * o added PATH_SEPARATOR and PATH_SEPARATOR_CHAR
 *
 * Revision 1.15  2001/11/26 07:54:48  slava
 * o added __UINT64_C macro
 *
 * Revision 1.14  2001/11/24 21:03:10  slava
 * o redefined sleep() via Sleep (rather than _sleep)
 *
 * Revision 1.13  2001/11/06 12:39:54  slava
 * o moved __STDC__ definition to s_os.h
 * o conditionally define some typs (INT8, UINT8, INT16 and such) which
 *   are defined in Windows CE headers and not anywhere else. This improves
 *   portability between Windows CE and other Win32 platforms.
 *
 * Revision 1.12  2001/10/22 00:10:37  slava
 * o recrafted memory management.
 *
 * Revision 1.11  2001/09/07 05:42:52  slava
 * o define SD_RECEIVE, SD_SEND and SD_BOTH if the are not defined
 *
 * Revision 1.10  2001/07/01 19:35:02  slava
 * o assume little endian byte order if nothing is defined
 *
 * Revision 1.9  2001/06/22 09:14:25  slava
 * o added StrCat macro
 *
 * Revision 1.8  2001/06/19 15:23:23  slava
 * o added Sprintf macro
 *
 * Revision 1.7  2001/06/12 08:54:33  slava
 * o cleanup
 *
 * Revision 1.6  2001/06/12 08:50:15  slava
 * o define TLS_OUT_OF_INDEXES if it's not defined
 * o define _sleep() as Sleep() on CE
 *
 * Revision 1.5  2001/06/08 08:17:50  slava
 * o added wrappers for character classification functions
 *
 * Revision 1.4  2001/06/03 19:32:57  slava
 * o removed references to INT_PTR
 *
 * Revision 1.3  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.2  2001/05/27 10:45:21  slava
 * o moved SysMem* macros to platform specific header files
 *
 * Revision 1.1  2001/05/26 19:19:29  slava
 * o moved most platform specific definitions from s_os.h to new s_win32.h
 *   and s_unix.h header files.
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
