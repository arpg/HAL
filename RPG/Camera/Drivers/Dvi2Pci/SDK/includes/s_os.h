/*
 * $Id: s_os.h,v 1.52 2010/09/25 09:31:32 slava Exp $
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

#ifndef _SLAVA_OS_H_
#define _SLAVA_OS_H_

#if defined(_WIN32) && !defined(__STDC__)
#  define __STDC__ 1
#endif /* _WIN32 && !__STDC__ */

/* compile-time assertion. If the expression evaluates to false at compile
 * time, the compiler will complain about negative subscript or something
 * to that effect */
#define COMPILE_ASSERT(x) extern int _compile_time_assert[2*(x)-1];

/* anything still does not support arguments in function prototypes? */
#define P_(prot) prot

#if defined(_NT_KERNEL)
#  include "s_ntk.h"
#elif defined(_WIN32)
#  include "s_win32.h"
#else /* !_WIN32 */
#  include "s_unix.h"
#endif /* !WIN32 */

/*
 * ALLPERMS - 777 mode
 */
#ifndef ALLPERMS
#  define ALLPERMS (S_IRWXU | S_IRWXG | S_IRWXO)
#endif /* ALLPERMS */

/* IP address and port */
typedef uint32_t IPaddr;
typedef unsigned short Port; 

/*
 * define INADDR_NONE
 * some platforms don't define it
 */
#ifndef INADDR_NONE
#  define INADDR_NONE ((IPaddr)-1)
#endif /* INADDR_NONE */

/*
 * Network byte order
 */
#define NET_BYTE_ORDER  BIG_ENDIAN

/* 
 * representing numbers in network byte order.
 * b0 is the least significant byte.
 */
#if BYTE_ORDER == LITTLE_ENDIAN
#  define NET_INT32(b3,b2,b1,b0) (\
    ((((uint32_t)(b3)))       & ((uint32_t)0x000000ff)) | \
    ((((uint32_t)(b2)) << 8)  & ((uint32_t)0x0000ff00)) | \
    ((((uint32_t)(b1)) << 16) & ((uint32_t)0x00ff0000)) | \
    ((((uint32_t)(b0)) << 24) & ((uint32_t)0xff000000)))
#  define NET_INT16(b1,b0) (\
    ((((uint16_t)(b1)))       & ((uint16_t)0x00ff)) | \
    ((((uint16_t)(b0)) << 8)  & ((uint16_t)0xff00))
#elif BYTE_ORDER == BIG_ENDIAN
#  define NET_INT32(b3,b2,b1,b0) (\
    ((((uint32_t)(b0)))       & ((uint32_t)0x000000ff)) | \
    ((((uint32_t)(b1)) << 8)  & ((uint32_t)0x0000ff00)) | \
    ((((uint32_t)(b2)) << 16) & ((uint32_t)0x00ff0000)) | \
    ((((uint32_t)(b3)) << 24) & ((uint32_t)0xff000000)))
#  define NET_INT16(b1,b0) (\
    ((((uint16_t)(b0)))       & ((uint16_t)0x00ff)) | \
    ((((uint16_t)(b1)) << 8)  & ((uint16_t)0xff00))
#else
#  error "Please fix BYTE_ORDER"
#endif /* BYTE_ORDER */

/*
 * the same, but using hex numbers. For example, 
 * NET_HEX32(63,82,53,63) gets translated into 
 * NET_INT32(0x63,0x82,0x53,0x63)
 */
#define NET_HEX32(b3,b2,b1,b0) NET_INT32(0x##b3,0x##b2,0x##b1,0x##b0)
#define NET_HEX16(b1,b0)       NET_INT16(0x##b1,0x##b0)

/* 
 * useful macros for IPv4 address formatting.
 */
#define BYTE_0(_i32) ((uint8_t)((_i32)  & 0x000000ff))
#define BYTE_1(_i32) ((uint8_t)(((_i32) & 0x0000ff00) >> 8))
#define BYTE_2(_i32) ((uint8_t)(((_i32) & 0x00ff0000) >> 16))
#define BYTE_3(_i32) ((uint8_t)(((_i32) & 0xff000000) >> 24))

/* 
 * IPv4 address in host byte order.
 */
#define HOST_IPADDR(b3,b2,b1,b0) (\
    ((((IPaddr)(b0)))       & 0x000000ff) | \
    ((((IPaddr)(b1)) << 8)  & 0x0000ff00) | \
    ((((IPaddr)(b2)) << 16) & 0x00ff0000) | \
    ((((IPaddr)(b3)) << 24) & 0xff000000))

/*
 * IPv4 address formatting
 */ 
#define IPADDR_FORMAT "%u.%u.%u.%u"

/* IPv4 address in the host byte order */
#define HOST_IPADDR_FORMAT_ARG(_i32) \
    (unsigned int)BYTE_3(_i32), \
    (unsigned int)BYTE_2(_i32), \
    (unsigned int)BYTE_1(_i32), \
    (unsigned int)BYTE_0(_i32)

/* IPv4 address in network byte order */
#define NET_IPADDR(b3,b2,b1,b0) NET_INT32(b3,b2,b1,b0)
#if BYTE_ORDER == LITTLE_ENDIAN
#  define NET_IPADDR_FORMAT_ARG(_i32) \
    (unsigned int)BYTE_0(_i32), \
    (unsigned int)BYTE_1(_i32), \
    (unsigned int)BYTE_2(_i32), \
    (unsigned int)BYTE_3(_i32)
#elif BYTE_ORDER == BIG_ENDIAN
#  define NET_IPADDR_FORMAT_ARG(_i32) HOST_IPADDR_FORMAT_ARG(_i32)
#endif /* BYTE_ORDER */

/* alias to NET_IPADDR_FORMAT_ARG for backward compatibility */
#define IPADDR_FORMAT_ARG(_i32) NET_IPADDR_FORMAT_ARG(_i32)

/* compile-time assertion that PtrWord actually has the size of a pointer */
COMPILE_ASSERT(sizeof(PtrWord) == sizeof(void*))
COMPILE_ASSERT(sizeof(PtrWord) == sizeof(size_t))

/* a few compile-time assertion regarding sizes of basic types */
COMPILE_ASSERT(sizeof(int) == 4)
#ifdef __LONG_64__
COMPILE_ASSERT(sizeof(long) == 8)
#else  /* !__LONG_64__ */
COMPILE_ASSERT(sizeof(long) == 4)
#endif /* !__LONG_64__ */

#endif /* _SLAVA_OS_H_ */

/*
 * HISTORY:
 *
 * $Log: s_os.h,v $
 * Revision 1.52  2010/09/25 09:31:32  slava
 * o made it possible to compile some slib modules in Mac OS X kernel build
 *   environment
 *
 * Revision 1.51  2010/09/25 07:49:59  slava
 * o removed references to Symbian which is no longer supported
 *   (and never really was)
 *
 * Revision 1.50  2006/11/03 17:07:47  slava
 * o changed default compile command from gmake to make
 *
 * Revision 1.49  2006/11/03 17:03:46  slava
 * o added another compile time assert that size_t and (void*) have the
 *   same size. Sounds pretty obvious, but it's always better be safe
 *   than sorry.
 *
 * Revision 1.48  2006/10/13 16:08:57  slava
 * o added a few compile-time assertion regarding sizes of basic types
 *
 * Revision 1.47  2006/07/17 21:08:23  slava
 * o moved NT kernel specific definition of _CRTIMP from s_os.h to s_ntk.h,
 *   plus a few more tweaks for better compatibility with various versions
 *   of the DDK
 *
 * Revision 1.46  2006/03/07 16:42:57  slava
 * o support for Symbian build environment
 *
 * Revision 1.45  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.44  2005/01/18 18:31:38  slava
 * o improved support for CodeWarrior for Symbian. The Symbian development
 *   environment uses gcc to generate dependencies, and it runs it without
 *   _WIN32 defined. This change should fix this problem.
 *
 * Revision 1.43  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.42  2003/12/04 04:48:55  slava
 * o replaced _LINUX_KERNEL_ with _LINUX_KERNEL for consistency with other
 *   platform defines such as _UNIX, _WIN32, _NT_KERNEL etc.
 *
 * Revision 1.41  2003/11/30 02:49:57  slava
 * o port to Linux kernel mode environment
 *
 * Revision 1.40  2003/10/13 19:15:57  slava
 * o fixed INADDR_NONE definition
 *
 * Revision 1.39  2003/05/21 19:11:31  slava
 * o fixed NET_IPADDR_FORMAT_ARG macro (didn't work on big endian systems)
 *
 * Revision 1.38  2003/05/21 00:11:03  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.37  2003/03/11 01:44:40  slava
 * o FreeBSD port
 *
 * Revision 1.36  2002/06/08 17:01:25  slava
 * o added NET_HEX32, NET_HEX16 and NET_BYTE_ORDER macros
 *
 * Revision 1.35  2002/01/22 05:01:50  slava
 * o do not define _POSIX_ under Windose
 *
 * Revision 1.34  2001/11/28 09:58:28  slava
 * o do not include <math.h> from here. The problem is that it defines some
 *   functions with very simple names, like j0, j1, y0, y1, etc. These
 *   identifiers are often being used as local variables, which produces
 *   warnings when being compiled by gcc with -Wshadow option. I have decided
 *   that <math.h> is not being used very often, it can be included when needed
 *   from individual source files, while all other source files can use j0, j1,
 *   jn, y0 and y1 as local variables.
 *
 * Revision 1.33  2001/11/06 12:37:20  slava
 * o small Win32 specific change - define __STDC__ and _POSIX_ before
 *   including any system headers
 *
 * Revision 1.32  2001/09/14 07:23:31  slava
 * o cleaned up IPADDR_FORMAT_ARG and other related macros. Added more
 *   macros for converting IP address to/from a human readable form
 *
 * Revision 1.31  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.30  2001/05/27 10:45:21  slava
 * o moved SysMem* macros to platform specific header files
 *
 * Revision 1.29  2001/05/26 19:19:29  slava
 * o moved most platform specific definitions from s_os.h to new s_win32.h
 *   and s_unix.h header files.
 *
 * Revision 1.28  2001/05/26 18:53:46  slava
 * o define PtrWord on Windows CE as 'long'. Some Windows CE SDKs don't
 *   provide INT_PTR definition.
 *
 * Revision 1.27  2001/05/26 18:44:00  slava
 * o don't define WIN32_LEAN_AND_MEAN if it's already defined
 *
 * Revision 1.26  2001/05/18 23:25:35  slava
 * o resolved macro conflict with dbgapi.h on Windows CE
 *
 * Revision 1.25  2001/05/18 22:29:54  slava
 * o slib has been (partially) ported to Windows CE
 *
 * Revision 1.24  2001/03/04 05:33:18  slava
 * o Win32 specific tweaking
 *
 * Revision 1.23  2001/03/04 02:34:58  slava
 * o defined INT8_MIN, INT8_MAX, UINT8_MIN, INT16_MIN, etc. when
 *   compiling on Windoze. On Unix they are defined in system header
 *   files
 *
 * Revision 1.22  2001/02/22 04:26:29  slava
 * o cleanup comments
 *
 * Revision 1.21  2001/01/31 02:00:41  slava
 * o portability fixes. Now compiles on Solaris
 *
 * Revision 1.20  2001/01/08 12:30:50  slava
 * o added I64X_FORMAT, I64S_FORMAT and I64U_FORMAT which have do be used
 *   when we know that the number to be formatted is precisely 64 bit on
 *   any system (i.e. is one of I64u, I64s, etc). Using LONG_LONG_FORMAT
 *   and such won't suffice because the meaning of "long long" is OS and
 *   hardware dependent.
 *
 * Revision 1.19  2001/01/06 05:12:38  slava
 * o include <arpa/inet.h> and <netdb.h> when compiling on Unix
 *
 * Revision 1.18  2000/12/23 04:13:13  slava
 * o minor changes related to PtrWord and __BYTE_ORDER definitions that
 *   may be useful in unlikely event if this code is being compiled on
 *   a non-Intel or 64-bit Windows machine
 *
 * Revision 1.17  2000/11/17 05:26:44  slava
 * o added APPEND_TEXT_MODE define
 * o added stat() definitions for Windows
 *
 * Revision 1.16  2000/11/01 03:57:53  slava
 * o made Bool a enum
 *
 * Revision 1.15  2000/09/18 07:01:58  slava
 * o included <string.h>
 *
 * Revision 1.14  2000/09/03 01:00:34  slava
 * o fixed compilation error on Linux
 *
 * Revision 1.13  2000/09/03 00:41:03  slava
 * o some network-related changes - IPaddr is now a 32-bit integer, added
 *   byte order definitions for Win32
 *
 * Revision 1.12  2000/08/28 02:19:05  slava
 * o redefine Winsock-style shutdown() constants (SD_RECEIVE, SD_SEND and
 *   SD_BOTH) in Unix style (that is, SHUT_RD, SHUT_WR and SHUT_RDWR)
 *
 * Revision 1.11  2000/08/28 02:07:28  slava
 * o Windoze related changes:
 *   1. include <winsock2.h> rather than <winsock.h>
 *   2. define FAR and NEAR as empty strings
 *
 * Revision 1.10  2000/08/26 12:10:34  slava
 * o added some socket-related includes and definitions
 *
 * Revision 1.9  2000/08/25 11:24:26  slava
 * o on Windoze we need to include <direct.h> in order to use mkdir().
 *   Also, had to fix mkdir() macro because Windoze implementation of
 *   mkdir() takes different number of arguments (no permission mask)
 *
 * Revision 1.8  2000/08/25 01:24:23  slava
 * o under Windoze, include <winsock/h> rather than <windows.h>
 *
 * Revision 1.7  2000/08/25 01:15:37  slava
 * o added definition of type Socket
 *
 * Revision 1.6  2000/08/24 15:10:12  slava
 * o undefine 'near' and 'far' macros under Win32
 *
 * Revision 1.5  2000/08/21 03:38:14  slava
 * o added sleep(n) macro for Win32 platform
 *
 * Revision 1.4  2000/08/21 03:07:13  slava
 * o include <signal.h>
 *
 * Revision 1.3  2000/08/19 11:50:32  slava
 * o fixed typo in comment
 *
 * Revision 1.2  2000/08/19 11:44:39  slava
 * o included <math.h> - why not?
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
