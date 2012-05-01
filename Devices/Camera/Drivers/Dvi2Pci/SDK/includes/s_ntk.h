/*
 * $Id: s_ntk.h,v 1.33 2010/07/04 16:27:08 slava Exp $
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

#ifndef _SLAVA_NTK_H_
#define _SLAVA_NTK_H_ 1

#if !defined(_WIN32) || !defined(_NT_KERNEL) || !defined(_SLAVA_OS_H_)
#  error "Do not include s_ntk.h directly. Include s_os.h"
#endif

#define __KERNEL__ 1
#define _TLS_SUPPORT 0 /* No TLS support in kernel mode */

#ifndef _CRTIMP
#  define _CRTIMP
#endif /* _CRTIMP */

#ifdef DBG
#  undef DEBUG
#  define DEBUG DBG
#  if DEBUG
#    ifndef _DEBUG
#      define _DEBUG
#    endif 
#    undef NDEBUG
#  else
#    ifndef NDEBUG
#      define NDEBUG
#    endif 
#    undef _DEBUG
#  endif 
#else
#   define DBG DEBUG
#endif /* DBG */

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
#include <stddef.h>
#include <signal.h>
#include <limits.h>
#include <errno.h>
#include <ctype.h>
#include <time.h>

#ifdef _USE_WDM_H_
#  include <wdm.h>
#elif defined(_USE_NTDDK_H_)
#  include <ntddk.h>
#else  /* _USE_NTDDK_H_ */
#  include <ntifs.h>
#  define _NTDDK_ /* prevent ntddk.h from being included */
#endif /* _USE_NTDDK_H_ */

#  ifndef OBJ_KERNEL_HANDLE
     /* this flag did't exist until Win2K */
#    define OBJ_KERNEL_HANDLE 0x00000200L
#  endif /* OBJ_KERNEL_HANDLE */

/* character type */
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
#ifdef UNICODE
#  define StrLen      wcslen
#  define StrCmp      wcscmp
#  define StrCpy      wcscpy
#  define StrnCmp     wcsncmp
#  define StrCat      wcscat
#  define StrChr      wcschr
#  define StrStr      wcsstr
#  define StrrChr     wcsrchr
#  define StrnCpy     wcsncpy
#  define Sprintf     swprintf
#  define Vsnprintf   _vsnwprintf
#  define StrCaseCmp  _wcsicmp
#  define StrnCaseCmp _wcsnicmp
#else  /* UNICODE */
#  define StrLen      strlen
#  define StrCmp      strcmp
#  define StrCpy      strcpy
#  define StrnCmp     strncmp
#  define StrCat      strcat
#  define StrChr      strchr
#  define StrStr      strstr
#  define StrrChr     strrchr
#  define StrnCpy     strncpy
//#define StrTod      strtod
//#define StrTol      strtol
//#define StrToul     strtoul
#  define Sprintf     sprintf
#  define Vsnprintf   _vsnprintf
#  define StrCaseCmp  _stricmp
#  define StrnCaseCmp _strnicmp
#endif /* UNICODE */

/* character classification functions */
#define IsUpper     isupper
#define IsLower     islower
#define IsPrint     isprint
#define IsDigit     isdigit
#define IsXdigit    isxdigit
#define IsSpace     isspace
#define ToUpper     toupper
#define ToLower     tolower

#undef isupper
#undef islower
#undef isdigit
#undef isxdigit
#undef isspace
#undef isprint

_CRTIMP int __cdecl isupper(int);
_CRTIMP int __cdecl islower(int);
_CRTIMP int __cdecl isdigit(int);
_CRTIMP int __cdecl isxdigit(int);
_CRTIMP int __cdecl isspace(int);
_CRTIMP int __cdecl isprint(int);

#define TEXT_(x) TEXT(x)
#define IsAlpha(_c) (\
   ((_c) >= 'a' && (_c) <= 'z') || \
   ((_c) >= 'A' && (_c) <= 'Z'))

/* define these types the way they are defined on Unix platforms */
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;

/* and their windows equivalents... */
typedef int8_t   INT8,   * PINT8;
typedef uint8_t  UINT8,  * PUINT8;
typedef int16_t  INT16,  * PINT16;
typedef uint16_t UINT16, * PUINT16;
typedef int32_t  INT32,  * PINT32;
typedef uint32_t UINT32, * PUINT32;
typedef int64_t  INT64,  * PINT64;
typedef uint64_t UINT64, * PUINT64;

typedef unsigned long  DWORD, * PDWORD;
typedef unsigned char  BYTE,  * PBYTE;
typedef unsigned short WORD,  * PWORD;

#define INT8_MIN   _I8_MIN
#define INT8_MAX   _I8_MAX
#define UINT8_MAX  _UI8_MAX

#define INT16_MIN   _I16_MIN
#define INT16_MAX   _I16_MAX
#define UINT16_MAX  _UI16_MAX

#define INT32_MIN   _I32_MIN
#define INT32_MAX   _I32_MAX
#define UINT32_MAX  _UI32_MAX

#define __INT64_C(c)  c ## i64 
#define __UINT64_C(c) c ## ui64
#define INT64_MIN  _I64_MIN
#define INT64_MAX  _I64_MAX
#define UINT64_MAX _UI64_MAX

#define LONG_HEX_FORMAT "%I64X"
#define LONG_LONG_FORMAT "%I64d"
#define LONG_ULONG_FORMAT "%I64u"

#define I64X_FORMAT LONG_HEX_FORMAT
#define I64S_FORMAT LONG_LONG_FORMAT
#define I64U_FORMAT LONG_ULONG_FORMAT

#define snprintf          _snprintf
#define vsnprintf         _vsnprintf
#define strcasecmp        _stricmp
#define strncasecmp       _strnicmp

/* 
 * no sockets in kernel mode, define it only to avoid too many conditional 
 * statements in the other include files 
 */
typedef int Socket;

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
#else
typedef uint32_t PtrWord;
#endif

/* (non-paged) memory allocation primitives provided by the system */
#define OS_MemAlloc(_size) ExAllocatePoolWithTag(NonPagedPool, _size, 'bilS')
#define OS_MemFree(_ptr) ((_ptr) ? ExFreePool(_ptr) : ((void)0))

#define NO_REALLOC

/*==========================================================================*
 *              6 4 - B I T
 *==========================================================================*/

/*
 * The following functions are inline for amd64
 *
 * KeGetCurrentThread
 * KeGetCurrentIrql
 * KeQuerySystemTime
 * KeInitializeSpinLock
 */
#if defined(_AMD64_) && !defined(__drv_maxIRQL)

ULONG64 __readgsqword (IN ULONG Offset);
#pragma intrinsic(__readgsqword)
#define KeGetCurrentThread() ((struct _KTHREAD *)__readgsqword(0x188))

ULONG64 __readcr8(VOID);
#pragma intrinsic(__readcr8)
#define KeGetCurrentIrql() ((KIRQL)__readcr8())

#define KI_USER_SHARED_DATA 0xFFFFF78000000000UI64
#define SharedSystemTime (KI_USER_SHARED_DATA + 0x14)
#define KeQuerySystemTime(CurrentCount) \
    (*((PULONG64)(CurrentCount)) = *((volatile ULONG64 *)(SharedSystemTime)))

#define KeInitializeSpinLock(Lock) (*(Lock) = 0)

#endif

/*==========================================================================*
 *              A D D I T I O N A L     A P I
 *==========================================================================*/

/* NtQuerySystemInformation */
typedef enum _SYSTEM_INFORMATION_CLASS {
    SystemBasicInformation,
    /* there are more, of course ... */
} SYSTEM_INFORMATION_CLASS;

typedef struct _SYSTEM_BASIC_INFORMATION {
    ULONG Reserved;
    ULONG TimerResolution;
    ULONG PageSize;
    ULONG NumberOfPhysicalPages;
    ULONG LowestPhysicalPageNumber;
    ULONG HighestPhysicalPageNumber;
    ULONG AllocationGranularity;
    ULONG MinimumUserModeAddress;
    ULONG MaximumUserModeAddress;
    KAFFINITY ActiveProcessorsAffinityMask;
    CCHAR NumberOfProcessors;
} SYSTEM_BASIC_INFORMATION, *PSYSTEM_BASIC_INFORMATION;

NTSYSAPI
NTSTATUS
NTAPI
NtQuerySystemInformation (
    IN SYSTEM_INFORMATION_CLASS SystemInformationClass,
    OUT PVOID SystemInformation,
    IN ULONG SystemInformationLength,
    OUT PULONG ReturnLength OPTIONAL
    );

/* some things are defined only in ntifs.h */
#if defined(_USE_NTDDK_H_) || defined(_USE_WDM_H_)
extern POBJECT_TYPE *PsThreadType;
NTKERNELAPI
NTSTATUS
ObQueryNameString (
    IN PVOID Object,
    OUT POBJECT_NAME_INFORMATION ObjectNameInfo,
    IN ULONG Length,
    OUT PULONG ReturnLength
    );
NTSYSAPI
NTSTATUS
NTAPI
ZwWaitForSingleObject(
    IN HANDLE Handle,
    IN BOOLEAN Alertable,
    IN PLARGE_INTEGER Timeout OPTIONAL
    );
#endif /* _USE_NTDDK_H_ */

/* Missing from pre-XP versions of the DDK */
NTKERNELAPI
LONG
KeReadStateEvent (
    IN PRKEVENT Event
    );

/*==========================================================================*
 *              U T I L I T I E S
 *==========================================================================*/

/* Unicode strings */
PUNICODE_STRING 
NT_AllocUnicodeString( 
    IN USHORT MaxLength 
    );

PUNICODE_STRING 
NT_DupUnicodeString( 
    IN PUNICODE_STRING String, 
    IN USHORT MaxLength
    );

PUNICODE_STRING 
NT_ConcatUnicodeStrings( 
    IN PWSTR s1,
    IN PWSTR s2,
    IN USHORT MaxLength 
    );

PUNICODE_STRING 
NT_AllocDeviceName( 
    IN PWSTR Name 
    ); 

PUNICODE_STRING 
NT_AllocSymLinkName( 
    IN PWSTR Name 
    ); 

VOID
NT_FreeUnicodeString( 
    IN PUNICODE_STRING s
    );

/* Object name */
OBJECT_NAME_INFORMATION * 
NT_QueryObjectName(
    PVOID obj
    );

/* Device object manipulations */
BOOLEAN 
NT_CreateDeviceObject( 
    IN  PDRIVER_OBJECT   DriverObject, 
    IN  PWSTR            Name, 
    IN  ULONG            DeviceExtensionSize,
    IN  BOOLEAN          Exclusive,
    IN  DEVICE_TYPE      DeviceType,
    OUT PDEVICE_OBJECT * DeviceObject,
    OUT NTSTATUS       * Status
    );

VOID 
NT_DeleteDeviceObject( 
    IN PDEVICE_OBJECT DeviceObject
    );

#endif /* _SLAVA_NTK_H_ */

/*
 * HISTORY:
 *
 * $Log: s_ntk.h,v $
 * Revision 1.33  2010/07/04 16:27:08  slava
 * o updated copyright statement
 *
 * Revision 1.32  2010/07/04 16:26:02  slava
 * o Unicode build issues
 *
 * Revision 1.31  2009/12/29 22:52:00  slava
 * o support for AMD64 build
 *
 * Revision 1.30  2008/11/20 08:26:24  slava
 * o switched NT kernel build to UNICODE
 *
 * Revision 1.29  2006/07/17 21:08:23  slava
 * o moved NT kernel specific definition of _CRTIMP from s_os.h to s_ntk.h,
 *   plus a few more tweaks for better compatibility with various versions
 *   of the DDK
 *
 * Revision 1.28  2005/02/25 02:53:39  slava
 * o TLS code moved to platform specific area
 *
 * Revision 1.27  2005/02/21 02:17:18  slava
 * o fixed NT kernel build erro
 *
 * Revision 1.26  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.25  2005/01/01 23:06:09  slava
 * o added StrnCmp macro
 *
 * Revision 1.24  2004/11/27 17:28:21  slava
 * o ZwWaitForSingleObject and PsThreadType are defined only in ntifs.h
 *   and not in ntddk.h or wdm.h
 *
 * Revision 1.23  2004/11/27 06:58:54  slava
 * o added support for NT kernel threads
 *
 * Revision 1.22  2004/04/06 06:32:51  slava
 * o added NT_QueryObjectName function
 *
 * Revision 1.21  2004/03/25 03:20:51  slava
 * o support for WinXP DDK
 *
 * Revision 1.20  2003/12/14 16:56:48  slava
 * o fixed a problem with PARSE_ULong64 and PARSE_Long64 functions no being
 *   compiled in some configurations
 *
 * Revision 1.19  2003/11/30 07:10:02  slava
 * o moved ObQueryNameString prototype from s_ntk.c to s_ntk.h
 * o added NtQuerySystemInformation prototype
 *
 * Revision 1.18  2003/11/30 02:47:09  slava
 * o define __KERNEL__ in NT kernel build
 *
 * Revision 1.17  2003/11/02 17:54:25  slava
 * o added a few utilities specific to NT kernel mode environment
 *
 * Revision 1.16  2003/08/28 17:16:23  slava
 * o replaced tabs with spaces
 *
 * Revision 1.15  2002/05/22 03:59:49  slava
 * o added BYTE_ORDER definition
 *
 * Revision 1.14  2002/01/02 17:18:36  slava
 * o added StrStr macro
 *
 * Revision 1.13  2001/12/25 03:24:23  slava
 * o fixed build for NT kernel mode
 *
 * Revision 1.12  2001/10/22 00:10:36  slava
 * o recrafted memory management.
 *
 * Revision 1.11  2001/07/07 22:42:50  slava
 * o defined ToUpper and ToLower macros for Windows NT kernel mode build
 *
 * Revision 1.10  2001/06/22 09:14:25  slava
 * o added StrCat macro
 *
 * Revision 1.9  2001/06/19 15:23:23  slava
 * o added Sprintf macro
 *
 * Revision 1.8  2001/06/08 08:17:50  slava
 * o added wrappers for character classification functions
 *
 * Revision 1.7  2001/06/03 19:41:41  slava
 * o after including <ntifs.h>, define _NTDDK_ to prevent <ntddk.h> from
 *   being included
 *
 * Revision 1.6  2001/06/03 19:37:12  slava
 * o by default, include ntifs.h because it contains more stuff. However,
 *   ntddk.h is sufficient for slib to compile. If _USE_NTDDK_H_ is defined,
 *   we include ntddk.h
 *
 * Revision 1.5  2001/05/30 09:09:30  slava
 * o added Windows style typedefs missing from ntddk.h (INT32, INT16, etc)
 *
 * Revision 1.4  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.3  2001/05/28 03:54:37  slava
 * o cleaned up mess with DBG vs DEBUG macros
 *
 * Revision 1.2  2001/05/27 17:52:19  slava
 * o allocate tagged memory
 *
 * Revision 1.1  2001/05/27 11:45:51  slava
 * o port to NT kernel mode environment
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
