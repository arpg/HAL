/*
 * $Id: s_strbuf.h,v 1.32 2010/09/25 09:55:06 slava Exp $
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

#ifndef _SLAVA_STRBUF_H_
#define _SLAVA_STRBUF_H_

#include "s_def.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/*
 * string buffer of variable length.
 */
typedef struct _StrBuf {
    size_t len;                 /* current length (no need for strlen()) */
    size_t alloc;               /* available size */
    Char * s;                   /* current buffer */
    Char * ext;                 /* external buffer (typically, on stack) */
} StrBuf;

extern StrBuf * STRBUF_Create P_((void));
extern void STRBUF_Delete P_((StrBuf * sb));
extern void STRBUF_Init P_((StrBuf * sb));
extern void STRBUF_InitBuf P_((StrBuf * sb, Char* buf, size_t bufsize));
extern void STRBUF_InitBuf2 P_((StrBuf* sb, Char* buf, size_t len, size_t n));
extern void STRBUF_Destroy P_((StrBuf * sb));
extern size_t STRBUF_Length P_((const StrBuf * sb));
extern void STRBUF_SetLength P_((StrBuf * sb, size_t maxlen));
extern Bool STRBUF_Equals P_((const StrBuf * sb1, const StrBuf * sb2));
extern Bool STRBUF_EqualsNoCase P_((const StrBuf * sb1, const StrBuf * sb2));
extern Bool STRBUF_EqualsTo P_((const StrBuf * sb, Str s));
extern Bool STRBUF_EqualsToNoCase P_((const StrBuf * sb, Str s));
extern Char STRBUF_CharAt P_((const StrBuf * sb, size_t pos));
extern Char STRBUF_FirstChar P_((const StrBuf * sb));
extern Char STRBUF_LastChar P_((const StrBuf * sb));
extern int  STRBUF_IndexOf P_((const StrBuf * sb, Char ch));
extern int  STRBUF_LastIndexOf P_((const StrBuf * sb, Char ch));
extern int  STRBUF_Find P_((const StrBuf * sb, Str s));
extern int  STRBUF_FindNoCase P_((const StrBuf * sb, Str s));
extern int  STRBUF_FindLast P_((const StrBuf * sb, Str s));
extern int  STRBUF_FindLastNoCase P_((const StrBuf * sb, Str s));
extern int  STRBUF_FindFrom P_((const StrBuf * sb, int pos, Str s));
extern int  STRBUF_FindFromNoCase P_((const StrBuf * sb, int pos, Str s));
extern int  STRBUF_Replace P_((StrBuf * sb, Char c1, Char c2));
extern int  STRBUF_ReplaceStr P_((StrBuf * sb, Str from, Str to));
extern Str  STRBUF_GetString P_((const StrBuf * sb));
extern Bool STRBUF_StartsWith P_((const StrBuf * sb, Str s));
extern Bool STRBUF_EndsWith P_((const StrBuf * sb, Str s));
extern Bool STRBUF_StartsWithNoCase P_((const StrBuf * sb, Str s));
extern Bool STRBUF_EndsWithNoCase P_((const StrBuf * sb, Str s));
extern void STRBUF_ToUpperCase P_((StrBuf * sb));
extern void STRBUF_ToLowerCase P_((StrBuf * sb));
extern Bool STRBUF_Alloc P_((StrBuf * sb, size_t minlen));
extern void STRBUF_Clear P_((StrBuf * sb));
extern void STRBUF_Erase P_((StrBuf * sb, size_t from, size_t to));
extern Bool STRBUF_Trim P_((StrBuf * sb));
extern Bool STRBUF_Copy P_((StrBuf * sb, Str s));
extern Bool STRBUF_CopyN P_((StrBuf * sb, Str s, size_t size));
extern Bool STRBUF_Append P_((StrBuf * sb, Str s));
extern Bool STRBUF_AppendN P_((StrBuf * sb, Str s, size_t n));
extern Bool STRBUF_AppendBuf P_((StrBuf * sb1, const StrBuf * sb2));
extern Bool STRBUF_AppendInt P_((StrBuf * sb, int i));
extern Bool STRBUF_AppendChar P_((StrBuf * sb, Char c));
extern Bool STRBUF_AppendBool P_((StrBuf * sb, Bool b));
extern Bool STRBUF_AppendDouble P_((StrBuf * sb, double d));
extern Bool STRBUF_AppendFormat P_((StrBuf*sb,Str format,...) PRINTF_ATTR(2,3));
extern Bool STRBUF_AppendFormatVa P_((StrBuf * sb, Str format, va_list va));
extern Bool STRBUF_Inflate P_((StrBuf * sb, size_t len, Char fill));
extern Bool STRBUF_Insert P_((StrBuf * sb, Str s, size_t pos));
extern Bool STRBUF_InsertN P_((StrBuf * sb, size_t pos, Str s, size_t len));
extern Bool STRBUF_InsertChar P_((StrBuf * sb, Char c, size_t pos));
extern Bool STRBUF_Format P_((StrBuf * sb, Str format, ...) PRINTF_ATTR(2,3));
extern Bool STRBUF_FormatVa P_((StrBuf * sb, Str format, va_list va));
extern Char * STRBUF_Dup P_((const StrBuf * sb));

#ifndef _WIN32_WCE
extern Bool STRBUF_AppendTime P_((StrBuf * sb, Time t));
extern Bool STRBUF_FormatTime P_((StrBuf * sb, Time t));
#endif /* _WIN32_WCE */

/*
 * UTF-8 conversion functions is only available in UNICODE build
 */
#ifdef _UNICODE
extern Bool STRBUF_CopyUTF8 P_((StrBuf * sb, const char * utf8));
extern Bool STRBUF_AppendUTF8 P_((StrBuf * sb, const char * utf8));
#define STRBUF_ToUTF8(sb) UTF8_Encode(STRBUF_Text(sb))
#endif /* _UNICODE */

/* 
 * STRBUF_Text is somewhat like STRBUF_GetString but it's a macro in release 
 * build and therefore is more efficient. Another difference of STRBUF_Text 
 * macro from STRBUF_GetString function is that it does not check if StrBuf 
 * pointer is NULL. 
 */
#if DEBUG
#   define STRBUF_Text(_sb) STRBUF_GetString(_sb)
#else  /* DEBUG */
#   define STRBUF_Text(_sb) ((_sb)->len ? (_sb)->s : TEXT(""))
#endif /* DEBUG */

/*
 * these variants include a fixed size buffer as well
 */
typedef struct _StrBuf16 {
    Char buf[16];
    StrBuf sb;
} StrBuf16;

typedef struct _StrBuf32 {
    Char buf[32];
    StrBuf sb;
} StrBuf32;

typedef struct _StrBuf64 {
    Char buf[64];
    StrBuf sb;
} StrBuf64;

typedef struct _StrBuf128 {
    Char buf[128];
    StrBuf sb;
} StrBuf128;

typedef struct _StrBuf256 {
    Char buf[256];
    StrBuf sb;
} StrBuf256;

#define STRBUF_InitBufXXX(b) STRBUF_InitBuf(&(b)->sb,(b)->buf,COUNT((b)->buf))

/* Macro to initialize StrBuf structure to point to a string constant. */
#define STRBUF_INIT(_string) { \
    COUNT(_string)-1,   /* current length */    \
    COUNT(_string),     /* available size */    \
    _string,            /* current buffer */    \
    _string             /* external buffer */   \
}
 
#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_STRBUF_H_ */

/*
 * HISTORY:
 *
 * $Log: s_strbuf.h,v $
 * Revision 1.32  2010/09/25 09:55:06  slava
 * o added PRINTF_ATTR macro which assigns printf-like characteristics to the
 *   declared function and enables gcc to check the format string against the
 *   parameters.
 *
 * Revision 1.31  2010/07/03 12:13:27  slava
 * o more UTF-8 stuff
 *
 * Revision 1.30  2010/03/10 21:16:25  slava
 * o added STRBUF_EqualsNoCase and STRBUF_EqualsToNoCase functions
 *
 * Revision 1.29  2009/12/26 12:22:40  slava
 * o working on 64-bit issues; some APIs have changed.
 *
 * Revision 1.28  2005/10/19 22:35:16  slava
 * o added STRBUF_FindLastNoCase function
 *
 * Revision 1.27  2005/08/26 21:11:39  slava
 * o added STRBUF_StartsWithNoCase and STRBUF_EndsWithNoCase functions
 *
 * Revision 1.26  2005/08/24 01:19:20  slava
 * o added STRBUF_FindNoCase and STRBUF_FindFromNoCase functions
 *
 * Revision 1.25  2005/01/23 16:02:43  slava
 * o added STRBUF_Dup function
 *
 * Revision 1.24  2004/12/31 01:53:55  slava
 * o added STRBUF_AppendBuf function
 *
 * Revision 1.23  2004/11/08 06:56:03  slava
 * o new functions: STRBUF_InitBuf2, STRBUF_Find, STRBUF_FindFrom,
 *   STRBUF_FindLast, STRBUF_ReplaceStr and STRBUF_InsertN
 *
 * Revision 1.22  2003/07/28 05:04:50  slava
 * o added STRBUF_Create and STRBUF_Delete functions
 *
 * Revision 1.21  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.20  2002/05/31 06:56:07  slava
 * o added STRBUF_ToUpperCase and STRBUF_ToLowerCase
 *
 * Revision 1.19  2002/05/22 04:04:55  slava
 * o renamed s_buf files into s_strbuf, renamed STRBUF_Read into
 *   FILE_ReadLine and moved it to s_file module.
 *
 * Revision 1.18  2001/12/31 01:04:12  slava
 * o added STRBUF_Text macro
 *
 * Revision 1.17  2001/12/20 10:44:31  slava
 * o port to Windows CE
 *
 * Revision 1.16  2001/12/07 03:26:40  slava
 * o added STRBUF_Replace() function
 *
 * Revision 1.15  2001/11/10 11:42:50  slava
 * o added STRBUF_Trim()
 *
 * Revision 1.14  2001/10/08 05:17:51  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.13  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.12  2001/05/18 22:29:54  slava
 * o slib has been (partially) ported to Windows CE
 *
 * Revision 1.11  2001/03/06 12:59:20  slava
 * o minor correction in STRBUF_AppendN() declaration
 *
 * Revision 1.10  2001/01/03 09:18:23  slava
 * o new file I/O support for transparent access to plain or compressed
 *   files, sockets, etc.
 *
 * Revision 1.9  2000/12/25 04:39:27  slava
 * o added STRBUF_Equals() and STRBUF_EqualsTo() functions and
 *   STRBUF_INIT() macro
 *
 * Revision 1.8  2000/12/17 16:09:47  slava
 * o added extern "C" {} so that slib could be used from a C++ program
 *
 * Revision 1.7  2000/11/17 05:24:04  slava
 * o STRBUF_CopyFromN() has been renamed into STRBUF_CopyN()
 * o STRBUF_CopyFrom() and STRBUF_Copy() were doing the same thing -
 *   removed STRBUF_CopyFrom()
 *
 * Revision 1.6  2000/11/09 17:54:18  slava
 * o removed some macros that were not being used
 *
 * Revision 1.5  2000/09/18 07:04:09  slava
 * o new functions: STRBUF_AppendFormat(), STRBUF_AppendFormatVa() and
 *   STRBUF_FormatVa()
 *
 * Revision 1.4  2000/09/03 00:34:50  slava
 * o added STRBUF_AppendN() function
 *
 * Revision 1.3  2000/09/02 13:18:34  slava
 * o added STRBUF_IndexOf() and STRBUF_LastIndexOf()
 *
 * Revision 1.2  2000/08/19 05:07:58  slava
 * o fixed "signed/unsigned mismatch" warnings issued by Microsoft
 *   VC 6.0 compiler
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
