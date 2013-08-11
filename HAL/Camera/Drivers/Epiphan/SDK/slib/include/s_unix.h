/*
 * $Id: s_unix.h,v 1.51 2010/12/23 09:42:14 slava Exp $
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

#ifndef _SLAVA_UNIX_H_
#define _SLAVA_UNIX_H_ 1

#ifdef _WIN32
#  error "Do not include s_unix.h directly. Include s_os.h"
#endif

#ifdef __KERNEL__
#  if defined(__linux__)
#    define _LINUX_KERNEL 1
#  elif defined(__APPLE__)
#    define _MACOSX_KERNEL 1
#  else
#    error "Unsupported kernel environment"
#  endif
#endif /* __KERNEL__ */

/* define __LONG_64__ if long is 64 bits */ 
#ifdef _LP64
#  define __LONG_64__
COMPILE_ASSERT(sizeof(long) == 8)
#endif /* _LP64 */

/* Limited support for kernel environment */
#ifdef _LINUX_KERNEL

#  ifndef KERNELRELEASE
#    define KERNELRELEASE 2.4
#  endif /* KERNELRELEASE */

#  include <linux/kernel.h>
#  include <linux/vmalloc.h>
#  include <linux/string.h>
#  include <linux/ctype.h>
#  include <linux/time.h>
#  include <linux/slab.h>
#  include <linux/nls.h>
#  include <asm/string.h>

   /* signals that should never be blocked by kernel */
#  define SHUTDOWN_SIGS  (sigmask(SIGKILL)|sigmask(SIGINT)|sigmask(SIGQUIT))

   /* converts milliseconds to kernel jiffies */
#  define MILLIS_TO_JIFFIES(_millis)  (((_millis) * HZ)/1000)

   /* memory allocation primitives */
#  define OS_MemAlloc(_size) vmalloc(_size)
#  define OS_MemFree(_ptr) vfree(_ptr)
#  define NO_REALLOC

extern int strcasecmp(const char *, const char *);

#elif defined(_MACOSX_KERNEL)

#  include <string.h>
#  include <stdarg.h>
#  include <sys/types.h>
#  include <sys/errno.h>
#  include <sys/time.h>
#  include <sys/stat.h>
#  include <sys/systm.h>

typedef __PTRDIFF_TYPE__ ptrdiff_t;
typedef __WCHAR_TYPE__ wchar_t;

static inline int isspace(int c) {
    switch (c) {
    case '\t':case '\n':case '\v':case '\f':case '\r':case ' ': return 1;
    default: return 0;
    }
}

/* these are called by MEM_Alloc, MEM_Free and such */
extern void * OS_MemAlloc P_((size_t size));
extern void   OS_MemFree P_((void * ptr));

#else /* !_LINUX_KERNEL && !_MACOSX_KERNEL*/

#  ifndef __STDC_LIMIT_MACROS
#    define __STDC_LIMIT_MACROS
#  endif
#  define _TLS_SUPPORT 1
#  define _USE_PTHREADS 1

#  include <stddef.h>
#  include <stdarg.h>
#  include <stdlib.h>
#  include <stdio.h>
#  include <string.h>
#  include <limits.h>
#  include <signal.h>
#  include <errno.h>
#  include <ctype.h>
#  include <time.h>
#  include <unistd.h>
#  include <pthread.h>
#  include <sys/time.h>
#  include <sys/stat.h>
#  include <sys/types.h>
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <netdb.h>

#  include <wchar.h>   /* requires MacOS X 10.3 and later */

#  ifndef __CYGWIN__

#    include <inttypes.h>
#    include <wctype.h> /* requires MacOS X 10.3 and later */

#  else /* __CYGWIN__ */

typedef u_int8_t uint8_t;
typedef u_int16_t uint16_t;
typedef u_int32_t uint32_t;
typedef u_int64_t uint64_t;

#    define INT32_MIN  INT_MIN
#    define INT32_MAX  INT_MAX
#    define UINT32_MAX UINT_MAX
#    define INT64_MIN  LONG_LONG_MIN
#    define INT64_MAX  LONG_LONG_MAX
#    define UINT64_MAX ULONG_LONG_MAX

#  endif /* __CYGWIN__ */

   /* memory allocation primitives */
#  define OS_MemAlloc(_size) malloc(_size)
#  define OS_MemRealloc(_ptr,_size) realloc((_ptr),(_size))
#  define OS_MemFree(_ptr) free(_ptr)

#endif /* !_LINUX_KERNEL */

#ifndef _UNIX
#  define _UNIX
#endif /* _UNIX */

#ifndef __hpux__
#  define HAVE_STRTOLL
#  define HAVE_STRTOULL
#  ifdef _LINUX_KERNEL
#    define strtol(_cp,_endp,_base) simple_strtol(_cp,_endp,_base) 
#    define strtoul(_cp,_endp,_base) simple_strtoul(_cp,_endp,_base) 
#    define strtoll(_cp,_endp,_base) simple_strtoll(_cp,_endp,_base) 
#    define strtoull(_cp,_endp,_base) simple_strtoull(_cp,_endp,_base) 
#  endif /* _LINUX_KERNEL */
#endif /* __hpux__ */

/* make sure BYTE_ORDER is properly defined */
#ifndef BYTE_ORDER
#  if defined(BSD) && (BSD >= 199103)
#    include <machine/endian.h>
#  else
#    if defined(__linux__) && !defined(__KERNEL__)
#      include <endian.h>
#    else
#      define LITTLE_ENDIAN 1234 /* least-significant byte first (vax, pc) */
#      define BIG_ENDIAN    4321 /* most-significant byte first (IBM, net) */
#      define PDP_ENDIAN    3412 /* LSB first in word, MSW first in long */
       /* these platforms are known to be little endian */
#      if defined(vax) || defined(ns32000) || defined(sun386) || \
          defined(i386) || defined(__ia64) || defined(MIPSEL) || \
          defined(_MIPSEL) || defined(BIT_ZERO_ON_RIGHT) || \
          defined(__alpha__) || defined(__alpha) || defined(__x86_64) || \
          defined(__arm__)
#        define BYTE_ORDER LITTLE_ENDIAN
#      endif
       /* these platforms are known to be big endian */
#      if defined(sel) || defined(pyr) || defined(mc68000) || \
          defined(sparc) || defined(is68k) || defined(tahoe) || \
          defined(ibm032) || defined(ibm370) || defined(MIPSEB) || \
          defined(_MIPSEB) || defined(_IBMR2) || defined(DGUX) || \
          defined(apollo) || defined(__convex__) || defined(_CRAY) || \
          defined(__hppa) || defined(__hp9000) || defined(__hp9000s300) || \
          defined(__hp9000s700) || defined(BIT_ZERO_ON_LEFT) || \
          defined(m68k) || defined(_BIG_ENDIAN) || defined(__BIG_ENDIAN__)
#        define BYTE_ORDER BIG_ENDIAN
#      endif
#    endif /* linux */
#  endif /* BSD */
#endif /* BYTE_ORDER */

/* final sanity check */
#if !defined(BYTE_ORDER) || \
     (BYTE_ORDER != BIG_ENDIAN && \
      BYTE_ORDER != LITTLE_ENDIAN && \
      BYTE_ORDER != PDP_ENDIAN)

  /*
   * you must determine what the correct byte order is for
   * your compiler - the next line is an intentional error
   * which will force your compiles to bomb until you fix
   * the above macros.
   */
#  error "Undefined or invalid BYTE_ORDER";
#endif

/* Linux defines this but other platforms may not */
#ifndef __INT64_C 
#  define __INT64_C(c) c##LL 
#endif /* __INT64_C  */

/* 
 * __pthread_initialize() used to be in some Linux header files, but now 
 * it's gone, at least from RedHat's headers.
 */
#ifndef HAVE_PTHREAD_INITIALIZE
#  define __pthread_initialize() NOTHING
#endif /* HAVE_PTHREAD_INITIALIZE */

/* 64-bit formats */
#define LONG_HEX_FORMAT "%llX"
#define LONG_LONG_FORMAT "%lld"
#define LONG_ULONG_FORMAT "%llu"

#ifdef __LONG_64__
#  ifndef __INT64_C
#    define __INT64_C(c) c ## L
#  endif /* __INT64_C */
#  ifndef __UINT64_C
#    define __UINT64_C(c) c ## UL
#  endif /* __UINT64_C */
#  ifdef __linux__
#    define I64X_FORMAT "%lX"
#    define I64S_FORMAT "%ld"
#    define I64U_FORMAT "%lu"
#  endif /* __linux__ */
#else /* !__LONG_64__ */
#  ifndef __INT64_C
#    define __INT64_C(c) c ## LL
#  endif /* __INT64_C */
#  ifndef __UINT64_C
#    define __UINT64_C(c) c ## ULL
#  endif /* __UINT64_C */
#endif /* !__LONG_64__ */

#ifndef I64X_FORMAT
#  define I64X_FORMAT LONG_HEX_FORMAT
#  define I64S_FORMAT LONG_LONG_FORMAT
#  define I64U_FORMAT LONG_ULONG_FORMAT
#endif

#ifdef _LINUX_KERNEL
#  define SCHAR_MIN  (-128)
#  define SCHAR_MAX  127
#  define UCHAR_MAX  255
#  ifdef __CHAR_UNSIGNED__
#    define CHAR_MIN 0
#    define CHAR_MAX UCHAR_MAX
#  else
#    define CHAR_MIN SCHAR_MIN
#    define CHAR_MAX SCHAR_MAX
#  endif
#  ifndef SHRT_MIN
#    define SHRT_MIN   (-32768)
#    define SHRT_MAX   32767
#    define USHRT_MAX  65535
#  endif
#  define UINT32_MAX UINT_MAX
#  define INT32_MAX  INT_MAX
#  define INT32_MIN  INT_MIN
#  define UINT64_MAX (~(__UINT64_C(0)))
#  define INT64_MAX  ((int64_t)(~(UINT64_MAX)>>1))
#  define INT64_MIN  (-INT64_MAX - 1)
#endif /* _LINUX_KERNEL */

#define READ_TEXT_MODE "r"
#define WRITE_TEXT_MODE "w"
#define APPEND_TEXT_MODE "a+"
#define READ_BINARY_MODE "r"
#define WRITE_BINARY_MODE "w"
#define APPEND_BINARY_MODE "a+"

#define FILE_SEPARATOR "/"
#define FILE_SEPARATOR_CHAR '/'
#define PATH_SEPARATOR ":"
#define PATH_SEPARATOR_CHAR ':'

typedef int Socket;
#define INVALID_SOCKET (-1)
#define closesocket(_s) close(_s) 
#define ioctlsocket(_s,_cmd,_argp)  ioctl(_s,_cmd,(char*)(_argp))

/* the word that has same size as a pointer */
typedef unsigned long PtrWord;

/* character type */
typedef char Char;

/* string manipulation routines */
#define StrLen      strlen
#define StrCmp      strcmp
#define StrnCmp     strncmp
#define StrCpy      strcpy
#define StrCat      strcat
#define StrChr      strchr
#define StrStr      strstr
#define StrrChr     strrchr
#define StrnCpy     strncpy
#define StrpBrk     strpbrk
#define StrTod      strtod
#define StrTol      strtol
#define StrToul     strtoul
#define StrCaseCmp  strcasecmp
#define StrnCaseCmp strncasecmp
#define Snprintf    snprintf
#define Vsnprintf   vsnprintf
#define Vfprintf    vfprintf
#define Sprintf     sprintf

/* character classification functions */
#define IsUpper(x)  isupper((int)(x))
#define IsLower(x)  islower((int)(x))
#define IsPrint(x)  isprint((int)(x))
#define IsDigit(x)  isdigit((int)(x))
#define IsXdigit(x) isxdigit((int)(x))
#define IsAlpha(x)  isalpha((int)(x))
#define IsSpace(x)  isspace((int)(x))
#define ToUpper(x)  toupper((int)(x))
#define ToLower(x)  tolower((int)(x))

/* other functions */
#define Ctime       ctime
#define Fopen       fopen
#define Freopen     freopen
#define Rmdir       rmdir

/* don't need UNICODE on UNIX... at least not yet */
#define TEXT(x) x
#define TEXT_(x) TEXT(x)
#define U_(u,a) a
#define UNICODE_ONLY_(x)
#define ANSI_ONLY_(x) x

#endif /* _SLAVA_UNIX_H_ */

/*
 * HISTORY:
 *
 * $Log: s_unix.h,v $
 * Revision 1.51  2010/12/23 09:42:14  slava
 * o Mac OS X doesn't understand %Ld
 *
 * Revision 1.50  2010/11/12 12:25:34  slava
 * o Linux kernel compilation issue (SHRT_MIN, SHRT_MAX and USHRT_MAX redef)
 *
 * Revision 1.49  2010/09/28 20:08:19  slava
 * o added compile-time assert that sizeof(long) == 8) if _LP64 is defined
 * o tweaking 64-bit formatting
 *
 * Revision 1.48  2010/09/28 19:26:21  slava
 * o 64-bit Unix build issues
 *
 * Revision 1.47  2010/09/25 09:31:32  slava
 * o made it possible to compile some slib modules in Mac OS X kernel build
 *   environment
 *
 * Revision 1.46  2010/09/25 07:46:29  slava
 * o use vmalloc/vfree in Linux kernel rather than kmalloc/kfree
 *
 * Revision 1.45  2010/07/04 16:25:42  slava
 * o define UNICODE_ONLY_ and ANSI_ONLY_ macros
 *
 * Revision 1.44  2010/07/04 16:22:43  slava
 * o define U_ macro for Unix
 *
 * Revision 1.43  2010/07/04 14:06:21  slava
 * o drop support for Mac OS X 10.2 and earlier (assume that <wchar.h> and
 *   <wctype.h> exist)
 *
 * Revision 1.42  2010/06/29 22:15:18  slava
 * o make use of gcc _BIG_ENDIAN when detecting endianess
 * o defined arm as little endian
 *
 * Revision 1.41  2010/06/21 16:50:30  slava
 * o a few changes to compile parts of slib in Linux kernel environment
 *
 * Revision 1.40  2009/05/21 14:15:30  slava
 * o attempt to fix x86_64 Linux build
 *
 * Revision 1.39  2006/11/23 00:22:10  slava
 * o added Snprintf macro
 *
 * Revision 1.38  2006/04/13 12:50:14  slava
 * o fixed typo in a comment
 *
 * Revision 1.37  2006/03/19 22:05:24  slava
 * o MacOS X compilation issues
 *
 * Revision 1.36  2005/02/25 02:53:39  slava
 * o TLS code moved to platform specific area
 *
 * Revision 1.35  2005/02/21 02:16:40  slava
 * o build errors
 *
 * Revision 1.34  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.33  2005/01/01 23:06:09  slava
 * o added StrnCmp macro
 *
 * Revision 1.32  2003/12/04 04:48:55  slava
 * o replaced _LINUX_KERNEL_ with _LINUX_KERNEL for consistency with other
 *   platform defines such as _UNIX, _WIN32, _NT_KERNEL etc.
 *
 * Revision 1.31  2003/12/04 04:30:54  slava
 * o event support for Linux kernel
 *
 * Revision 1.30  2003/12/03 04:30:32  slava
 * o I changed my mind, let's use kmalloc/kfree
 *
 * Revision 1.29  2003/12/02 05:14:42  slava
 * o use vmalloc/vfree in Linux kernel rather than kmalloc/kfree because
 *   we generally don't need physically contiguous memory. Also, kmalloc
 *   has 128k size limitation, and vmalloc doesn't.
 *
 * Revision 1.28  2003/12/01 13:18:21  slava
 * o provided primitive strcasecmp substitution for Linux kernel environment
 *
 * Revision 1.27  2003/11/30 18:10:22  slava
 * o resolved conflict with the 'current' macro defined in Linux kernel
 *   include files. Those guys never worried about namespace pollution...
 *
 * Revision 1.26  2003/11/30 02:49:57  slava
 * o port to Linux kernel mode environment
 *
 * Revision 1.25  2003/06/21 18:25:11  slava
 * o port to HP-UX
 *
 * Revision 1.24  2003/05/16 02:19:07  slava
 * o port to Mac OS X
 *
 * Revision 1.23  2003/05/16 01:50:38  slava
 * o slib compiles and appears to work under cygwin
 *
 * Revision 1.22  2003/03/18 16:07:58  slava
 * o added Ctime macro
 *
 * Revision 1.21  2003/03/11 01:44:40  slava
 * o FreeBSD port
 *
 * Revision 1.20  2003/02/06 20:10:57  slava
 * o added ioctlsocket macro
 *
 * Revision 1.19  2003/02/03 21:56:13  slava
 * o added StrpBrk macro
 *
 * Revision 1.18  2003/01/10 06:42:41  slava
 * o use __sun rather than sun as a test for Solaris/SunOS platform
 *
 * Revision 1.17  2002/06/25 02:55:21  slava
 * o do not define _UNIX if it's already defined
 *
 * Revision 1.16  2002/06/11 04:14:55  slava
 * o fixed a problem with BYTE_ORDER macro not being defined on some Unix
 *   platforms (e.g. Solaris)
 *
 * Revision 1.15  2002/02/11 05:35:00  slava
 * o define _USE_PTHREADS in Unix build
 *
 * Revision 1.14  2002/02/06 12:27:19  slava
 * o include wchar.h and wctype.h
 *
 * Revision 1.13  2002/01/02 17:18:36  slava
 * o added StrStr macro
 *
 * Revision 1.12  2001/12/23 13:28:00  slava
 * o added READ_BINARY_MODE, WRITE_BINARY_MODE and APPEND_BINARY_MODE defines
 *
 * Revision 1.11  2001/12/20 10:51:36  slava
 * o defined Freopen
 *
 * Revision 1.10  2001/12/07 03:31:41  slava
 * o small fix that makes slib compile on Solaris
 *
 * Revision 1.9  2001/12/01 05:18:05  slava
 * o added PATH_SEPARATOR and PATH_SEPARATOR_CHAR
 *
 * Revision 1.8  2001/11/24 19:39:19  slava
 * o cleaned up thread support for Unix platforms
 *
 * Revision 1.7  2001/10/22 00:10:37  slava
 * o recrafted memory management.
 *
 * Revision 1.6  2001/06/22 09:14:25  slava
 * o added StrCat macro
 *
 * Revision 1.5  2001/06/19 15:23:23  slava
 * o added Sprintf macro
 *
 * Revision 1.4  2001/06/08 08:17:50  slava
 * o added wrappers for character classification functions
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
 * compile-command: "make -C .."
 * End:
 */
