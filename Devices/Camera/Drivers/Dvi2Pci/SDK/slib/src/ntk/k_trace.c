/*
 * $Id: k_trace.c,v 1.4 2009/03/25 22:30:50 slava Exp $
 *
 * Copyright (C) 2008-2009 by Slava Monich
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

/**
 * Trace utilities for Windows NT kernel environment
 */

#ifndef _NT_KERNEL
#  error "_NT_KERNEL must be defined to compile this code"
#endif

#undef PRINT_Stdout
#undef PRINT_Stderr

#if DEBUG_TRACE

#  if !DBG
NTSYSAPI VOID NTAPI RtlAssert(PVOID ass, PVOID file, ULONG line, PCHAR msg);
#  endif /* !DBG */


/**
 * Default assertion handler for Windows NT kernel environment
 */
void DEBUG_AssertHandler(const char * msg, const char * file, long line)
{
    RtlAssert((PVOID)msg, (PVOID)file, line, NULL);
}
#endif /* DEBUG_TRACE */

/**
 * This call has no effect in Windows NT kernel environment 
 */
void PRINT_UseConsole(Bool enable)
{
    UNREF(enable);
}

/**
 * Common code for PRINT_Stdout and PRINT_Stderr.
 * Dumps formatted string to the debug console.
 */
STATIC int NT_DbgPrintVa(LPCSTR format, va_list va)
{
    char buf[512];
    int n = vsnprintf(buf,COUNT(buf),format,va);
    buf[COUNT(buf)-1] = 0;
    DbgPrint("%s",buf);
    return n;
}

/**
 * Dumps formatted string to the debug console
 */
int PRINT_Stdout(const char* format, va_list va)
{
    return NT_DbgPrintVa(format, va);
}

/**
 * Dumps formatted string to the debug console
 */
int PRINT_Stderr(const char* format, va_list va)
{
    return NT_DbgPrintVa(format, va);
}

/**
 * Common code for PRINT_StdoutU and PRINT_StderrU.
 * Dumps formatted string to the debug console.
 */
#ifdef _UNICODE
STATIC int NT_DbgPrintVaU(LPCWSTR format, va_list va)
{
    int n;
    StrBuf256 buf;
    Bool printUnicode = (KeGetCurrentIrql() == PASSIVE_LEVEL) ||
        !wcsstr(format, L"%C")  || !wcsstr(format, L"%S")  ||
        !wcsstr(format, L"%lc") || !wcsstr(format, L"%ls") ||
        !wcsstr(format, L"%wc") || !wcsstr(format, L"%ws") ||
        !wcsstr(format, L"%wZ");
    if (!printUnicode) {
        LPCWSTR ptr = format;
        while (*ptr && !(*ptr & (~((WCHAR)0xff)))) ptr++;
        if (*ptr) printUnicode = True;
    }
    STRBUF_InitBufXXX(&buf);
    if (STRBUF_AppendFormatVa(&buf.sb, format, va)) {
        if (printUnicode) {
            n = DbgPrint("%ws",STRBUF_Text(&buf.sb));
        } else {
            LPCWSTR ws = buf.sb.s;
            LPSTR s = (LPSTR)ws;
            while ((*s++ = (char)(*ws++)) != 0);
            n = DbgPrint("%s",buf.sb.s);
        }
    }
    STRBUF_Destroy(&buf.sb);
    return n;
}

/**
 * Dumps formatted string to the debug console
 */
int PRINT_StdoutU(Str format, va_list va)
{
    return NT_DbgPrintVaU(format, va);
}

/**
 * Dumps formatted string to the debug console
 */
int PRINT_StderrU(Str format, va_list va)
{
    return NT_DbgPrintVaU(format, va);
}

#endif /* _UNICODE */

/*
 * HISTORY:
 *
 * $Log: k_trace.c,v $
 * Revision 1.4  2009/03/25 22:30:50  slava
 * o made it easier to compile release version of slib with debug trace
 *
 * Revision 1.3  2008/11/20 10:36:26  slava
 * o fixed NT_DbgPrintVaU, now it really works
 *
 * Revision 1.2  2008/11/20 08:26:25  slava
 * o switched NT kernel build to UNICODE
 *
 * Revision 1.1  2008/11/05 12:15:08  slava
 * o made assert handler configurable, cleaned up the assertion handling
 *   code a bit, moved system specific code to system-specific directories
 *
 */
