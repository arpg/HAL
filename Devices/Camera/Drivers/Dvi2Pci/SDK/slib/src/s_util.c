/*
 * $Id: s_util.c,v 1.95 2010/09/25 09:55:06 slava Exp $
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

#include "s_util.h"

#if !defined(_WIN32_WCE) && !defined(__KERNEL__)
#  define HAVE_ERRNO 1
#endif /* !_WIN32_WCE && !__KERNEL__ */

#ifdef __linux__
#  define PROC_STAT_CPU_COUNT "cpus detected"
#  define PROC_STAT_CPU "processor"
#  ifndef STATFILE
#  define STATFILE "/proc/cpuinfo"
#  endif /* STATFILE */
#endif /* __linux__ */

#ifdef __sun
#  include <kstat.h>
#endif /* __sun */

#if defined (__OpenBSD__) || defined (__NetBSD__) || defined (__FreeBSD__) || defined(__APPLE__)
#  include <sys/sysctl.h>
#endif /* __OpenBSD__ || __NetBSD__ || __FreeBSD__ || __APPLE__ */

/*==========================================================================*
 *              S Y S T E M    I N F O R M A T I O N
 *==========================================================================*/

/**
 * Returns number of active CPUs.
 */
int SYSTEM_CountCPU() 
{
    static int CPU_count = 0;

    /* detect number of CPUs when we first time get here */
    if (!CPU_count) {

#if defined(_NT_KERNEL)

        NTSTATUS Status;
        SYSTEM_BASIC_INFORMATION BasicInfo;
        memset(&BasicInfo, 0, sizeof(BasicInfo));
        Status = NtQuerySystemInformation(SystemBasicInformation,&BasicInfo,
            sizeof(BasicInfo),NULL);
        ASSERT(NT_SUCCESS(Status));
        if (NT_SUCCESS(Status)) {
            CPU_count = BasicInfo.NumberOfProcessors;
        }

#elif defined(_WIN32)

        SYSTEM_INFO sysInfo;
        GetSystemInfo(&sysInfo);
        CPU_count = sysInfo.dwNumberOfProcessors;

#elif defined(__linux__) && !defined(__KERNEL__)

        File * f = FILE_Open(STATFILE, "r", PlainFile);
        if (f) {
            StrBuf64 buf;
            STRBUF_InitBufXXX(&buf);

            /* first try to find "cpus detected" field */
            while (FILE_ReadLine(f, &buf.sb)) {
                if (STRBUF_StartsWith(&buf.sb, PROC_STAT_CPU_COUNT)) {
                    int colon = STRBUF_IndexOf(&buf.sb, ':');
                    if (colon > 0) {
                        Str s = buf.sb.s + colon + 1;
                        while (*s && IsSpace(*s)) s++;
                        VERIFY(PARSE_Int(s, &CPU_count, 0));
                    }
                    break;
                }
            }
            FILE_Close(f);

            /* try to count the number of CPUs mentioned in the stat file */
            if (CPU_count <= 0) {
                f = FILE_Open(STATFILE, "r", PlainFile);
                if (f) {
                    CPU_count = 0;
                    while (FILE_ReadLine(f, &buf.sb)) {
                        if (STRBUF_StartsWith(&buf.sb, PROC_STAT_CPU)) {
                            CPU_count++;
                        }
                    }
                    FILE_Close(f);
                }
            }

            STRBUF_Destroy(&buf.sb);
        }

#elif defined(__sun)

        kstat_ctl_t * kc = kstat_open();
        if (kc) {
            static const char KS_MODULE [] = "unix";
            static const char KS_NAME [] = "system_misc";
            static const char KS_DATA [] = "ncpus";
            
            kstat_t * ks;
            
            /* copy strings to avoid compilation warnings... */
            char ks_module[COUNT(KS_MODULE)];
            char ks_name[COUNT(KS_NAME)];
            char ks_data[COUNT(KS_DATA)];
            StrCpy(ks_module, KS_MODULE);
            StrCpy(ks_name, KS_NAME);
            StrCpy(ks_data, KS_DATA);

            ks = kstat_lookup(kc, ks_module, 0, ks_name);
            if (ks) {
                if (kstat_read(kc, ks, 0) != -1) {
                    kstat_named_t * kn = kstat_data_lookup(ks, ks_data);
                    if (kn && kn->value.ui32 > 0) {
                        CPU_count = kn->value.ui32;
                    }
                }
            }
            kstat_close(kc);
        }
#elif defined (__OpenBSD__) || defined (__NetBSD__) || defined (__FreeBSD__) || defined(__APPLE__)

        size_t buflen = sizeof(CPU_count);
        int mib[2];

        mib[0] = CTL_HW;
        mib[1] = HW_NCPU;
        if (sysctl(mib, COUNT(mib), &CPU_count, &buflen, NULL, 0) != 0) {
            TRACE1("SYSTEM: sysctl error %d",errno);
            CPU_count = 1;
        }

#else

        /* autodetection NOT currently implemented for other platforms */ 
        TRACE("SYSTEM: auto-detection of number of CPUs not implemented!\n");
        CPU_count = 1;

#endif

        /* final sanity check */
        ASSERT(CPU_count > 0);
        if (CPU_count <= 0) CPU_count = 1;
        Verbose(TEXT("SYSTEM: %d CPU detected\n"),CPU_count);
    }
    return CPU_count;
}

/*==========================================================================*
 *              U R L    E N C O D I N G
 *==========================================================================*/

/**
 * URL encodes a string into a string buffer. On success, returns pointer 
 * to the string buffer data. The URL encoded string is guaranteed to be a
 * 7-bit ASCII string consisting entirely of valid URL characters. If buffer
 * is empty, memory allocation fails or source string contains characters 
 * that cannot be URL encoded, returns NULL. Does not destroy the original
 * contents of the string buffer (appends the result). 
 */
Str URL_Encode(StrBuf * dest, Str src)
{
    /* The list of characters that are not encoded has been
     * determined as follows:
     *
     * RFC 2396 states:
     * -----
     * Data characters that are allowed in a URI but do not have a
     * reserved purpose are called unreserved.  These include upper
     * and lower case letters, decimal digits, and a limited set of
     * punctuation marks and symbols. 
     *
     * unreserved  = alphanum | mark
     *
     * mark        = "-" | "_" | "." | "!" | "~" | "*" | "'" | "(" | ")"
     *
     * Unreserved characters can be escaped without changing the
     * semantics of the URI, but this should not be done unless the
     * URI is being used in a context that does not allow the
     * unescaped character to appear.
     * -----
     *
     * It appears that both Netscape and Internet Explorer escape
     * all special characters from this list with the exception
     * of "-", "_", ".", "*". While it is not clear why they are
     * escaping the other characters, perhaps it is safest to
     * assume that there might be contexts in which the others
     * are unsafe if not escaped. Therefore, we will use the same
     * list. It is also noteworthy that this is consistent with
     * O'Reilly's "HTML: The Definitive Guide" (page 164).
     *
     * As a last note, Intenet Explorer does not encode the "@"
     * character which is clearly not unreserved according to the
     * RFC. We are being consistent with the RFC in this matter,
     * as is Netscape.
     *
     */
    if (src) {
        Char c;
        size_t orig = STRBUF_Length(dest);
        while ((c = *src++) != 0) {
            switch (c) {

            /* valid URL characters */
            case 'a': case 'b': case 'c': case 'd': 
            case 'e': case 'f': case 'g': case 'h': 
            case 'i': case 'j': case 'k': case 'l': 
            case 'm': case 'n': case 'o': case 'p': 
            case 'q': case 'r': case 's': case 't': 
            case 'u': case 'v': case 'w': case 'x': 
            case 'y': case 'z': case 'A': case 'B': 
            case 'C': case 'D': case 'E': case 'F': 
            case 'G': case 'H': case 'I': case 'J': 
            case 'K': case 'L': case 'M': case 'N': 
            case 'O': case 'P': case 'Q': case 'R': 
            case 'S': case 'T': case 'U': case 'V': 
            case 'W': case 'X': case 'Y': case 'Z': 
            case '0': case '1': case '2': case '3': 
            case '4': case '5': case '6': case '7': 
            case '8': case '9':

            /*
             * (some) unreserved characters not being escaped (see the 
             * long comment above) 
             */
            case '-': case '_': case '.': case '*': 

                /* append this character */
                if (!STRBUF_AppendChar(dest,c)) {
                    STRBUF_SetLength(dest, orig);
                    return NULL;
                }
                break;

            default:
#ifdef UNICODE
                /* cannot encode wide characters */
                if ((c & (~((Char)0xff))) != 0) {
                    STRBUF_SetLength(dest, orig);
                    return NULL;
                }
                /* fall through */
#endif /* UNICODE */

            /*
             * (some) unreserved characters being escaped (see the 
             * long comment above) 
             */
            case '!':  case '~': case '(': case ')':
            case '\\': 

            /* reserved characters */
            case ';': case '/': case '?': case ':': 
            case '@': case '&': case '=': case '+': 
            case '$': case ',':
                if (!STRBUF_AppendFormat(dest,TEXT("%%%02X"),((int)c)&0xff)) {
                    STRBUF_SetLength(dest, orig);
                    return NULL;
                }
            }
        }
        return STRBUF_Text(dest);
    }
    return NULL;
}

/**
 * Encodes the specified characters in the data as escape triples.
 * If the input string is NULL, memory allocation fails or source
 * string contains characters that cannot be URL encoded, returns
 * NULL. Does not destroy the original contents of the string buffer 
 * (appends the result). 
 */
Str URL_EncodeChars(StrBuf * dest, Str src, Str esc)
{
    if (src) {
        Char c;
        size_t origLen = STRBUF_Length(dest);
        while ((c = *src++) != 0) {

#ifdef UNICODE
            /* cannot encode wide characters */
            if ((c & (~((Char)0xff))) != 0) {
                STRBUF_SetLength(dest, origLen);
                return NULL;
            }
#endif /* UNICODE */

            if (StrChr(esc, c)) {
                /* encode this character */
                if (!STRBUF_AppendFormat(dest,TEXT("%%%02X"),((int)c)&0xff)) {
                    STRBUF_SetLength(dest, origLen);
                    return NULL;
                }
            
            } else {
                /* append this character */
                if (!STRBUF_AppendChar(dest,c)) {
                    STRBUF_SetLength(dest, origLen);
                    return NULL;
                }
            }
        }
        return STRBUF_Text(dest);
    }
    return NULL;
}

/**
 * Decodes URL encoded string into a string buffer. On success, returns 
 * pointer to string buffer data, NULL if memory allocation fails. Does 
 * not destroy the original contents of the string buffer (appends the 
 * result). 
 */
Str URL_Decode(StrBuf * dest, Str src)
{
    if (src) {
        Char c;
        size_t orig = STRBUF_Length(dest);
        while ((c = *src++) != 0) {
            if (c == '%' && IsXdigit(src[0]) && IsXdigit(src[1])) {
                Char x1 = src[0];
                Char x2 = src[1];
                if (x1 >= '0' && x1 <= '9') {
                    x1 = (Char)(x1 - '0');
                } else if (x1 >= 'A' && x1 <= 'F') {
                    x1 = (Char)(x1 - 'A' + 10);
                } else if (x1 >= 'a' && x1 <= 'f') {
                    x1 = (Char)(x1 - 'a' + 10);
                } else {
                    ASSMSG("Not a hex digit!");
                }
                if (x2 >= '0' && x2 <= '9') {
                    x2 = (Char)(x2 - '0');
                } else if (x2 >= 'A' && x2 <= 'F') {
                    x2 = (Char)(x2 - 'A' + 10);
                } else if (x2 >= 'a' && x2 <= 'f') {
                    x2 = (Char)(x2 - 'a' + 10);
                } else {
                    ASSMSG("Not a hex digit!");
                }
                c = (Char)(x2 | (x1 << 4));
                src += 2;
            }
            if (!STRBUF_AppendChar(dest,c)) {
                STRBUF_SetLength(dest, orig);
                return NULL;
            }
        }
        return STRBUF_Text(dest);
    }
    return NULL;
}

/*
 * HISTORY:
 *
 * $Log: s_util.c,v $
 * Revision 1.95  2010/09/25 09:55:06  slava
 * o added PRINTF_ATTR macro which assigns printf-like characteristics to the
 *   declared function and enables gcc to check the format string against the
 *   parameters.
 *
 * Revision 1.94  2009/04/09 22:36:04  slava
 * o BSD way of detecting number of CPUs works for Mac OS X as well (why am I
 *   not surprised?)
 *
 * Revision 1.93  2009/04/09 21:54:57  slava
 * o use size_t instead of int where it's appropriate
 *
 * Revision 1.92  2006/10/12 18:30:39  slava
 * o splitting s_util.c into multiple files, because it's getting huge.
 *   Moved parsing utilities to s_parse.c
 *
 * Revision 1.91  2006/10/07 21:42:02  slava
 * o fixed yet another problem with PARSE_Float. It appears that expression
 *   (value == value) sometimes evaluates to false for quite legitimate
 *   floating point values. Replaced it with explicit check for the bit
 *   pattern (exponent of all 1s) common for all NaN and Inf values. Should
 *   be portable enough.
 *
 * Revision 1.90  2006/09/25 16:02:25  slava
 * o PARSE_Float didn't work on negative numbers. Now it does.
 *
 * Revision 1.89  2006/09/25 15:18:36  slava
 * o added PARSE_Float function
 *
 * Revision 1.88  2005/02/20 20:31:30  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.87  2005/01/23 16:38:43  slava
 * o oops... fixed a bug in URL_EncodeChars
 *
 * Revision 1.86  2005/01/23 14:01:56  slava
 * o added URL_EncodeChars function
 *
 * Revision 1.85  2004/12/26 18:22:20  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.84  2004/07/19 22:55:11  slava
 * o moved BASE64 encoding functions from s_util to s_base64 module
 *
 * Revision 1.83  2004/04/19 08:06:29  slava
 * o use BoolValue macro to convert compiler's internal boolean value into Bool
 *
 * Revision 1.82  2003/12/14 16:56:48  slava
 * o fixed a problem with PARSE_ULong64 and PARSE_Long64 functions no being
 *   compiled in some configurations
 *
 * Revision 1.81  2003/11/30 07:12:36  slava
 * o implemented detection of number of CPUs in NT kernel mode environment
 *
 * Revision 1.80  2003/11/30 02:49:58  slava
 * o port to Linux kernel mode environment
 *
 * Revision 1.79  2003/08/28 17:16:23  slava
 * o replaced tabs with spaces
 *
 * Revision 1.78  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.77  2003/03/18 16:52:44  slava
 * o added Unicode build configurations for Windoze
 *
 * Revision 1.76  2003/03/15 17:58:34  slava
 * o detection of number of cpus on FreeBSD
 *
 * Revision 1.75  2003/03/11 06:24:26  slava
 * o FreeBSD port
 *
 * Revision 1.74  2003/01/10 06:47:28  slava
 * o figured out how to detect number of CPUs on Solaris
 *
 * Revision 1.73  2003/01/02 02:44:29  slava
 * o added IO_TermSize function
 *
 * Revision 1.72  2002/11/24 03:24:02  slava
 * o fixed URL encoding code to NOT replace space with "+" character.
 *   Replacing space with a plus is apparently something specific to
 *   x-www-form-urlencoded MIME type only. RFC 2396 says nothing about
 *   that. Therefore, we encode space as "%20"
 *
 * Revision 1.71  2002/10/21 05:20:35  slava
 * o added FILE_IsFile function
 *
 * Revision 1.70  2002/08/27 11:52:03  slava
 * o some reformatting
 *
 * Revision 1.69  2002/08/16 04:09:55  slava
 * o fixed a relatively minor bug in BASE64_Decode (didn't take original
 *   buffer length into account; that could cause unnecessary memory
 *   reallocations or even truncate the data in unlikely event if memory
 *   allocation fails)
 *
 * Revision 1.68  2002/07/08 07:32:24  slava
 * o fixed compilation error on Unix
 *
 * Revision 1.67  2002/07/08 07:12:37  slava
 * o added URL_Encode and URL_Decode
 *
 * Revision 1.66  2002/07/07 09:13:47  slava
 * o added BASE64_Encode and BASE64_Decode
 *
 * Revision 1.65  2002/05/22 04:59:01  slava
 * o fixed compilation error in Unix-specific part of SYSTEM_CountCPU
 *
 * Revision 1.64  2002/05/22 04:19:10  slava
 * o fixed include statements after s_sbuf.h was renamed into s_strbuf.h
 *
 * Revision 1.63  2002/01/22 20:35:23  slava
 * o fixed a bug in FILE_Save
 *
 * Revision 1.62  2002/01/22 05:08:22  slava
 * o changed FILE_Save to create the directory where output file is located
 * o minor changes in FILE_MkDir (don't walk the directory hierarhy if the
 *   directory in question already exists)
 *
 * Revision 1.61  2001/12/29 03:00:58  slava
 * o fixed error message
 *
 * Revision 1.60  2001/12/23 20:12:40  slava
 * o fixed a bug affecting Unicode build (i.e. WinCE)
 *
 * Revision 1.59  2001/12/22 03:38:24  slava
 * o moved socket-related functions from s_util.c to s_net.c in attempt to
 *   prevent unnecessary import of winsock functions on Win32 platforms
 *
 * Revision 1.58  2001/12/20 10:44:32  slava
 * o port to Windows CE
 *
 * Revision 1.57  2001/11/28 10:03:56  slava
 * o <math.h> is no longer being include from s_os.h
 *
 * Revision 1.56  2001/11/25 21:20:53  slava
 * o compiled it for Windoze CE
 *
 * Revision 1.55  2001/11/24 20:49:50  slava
 * o parsing was broken on Unix (didn't detect overflow)
 *
 * Revision 1.54  2001/10/26 03:47:00  slava
 * o fixed bugs in FILE_CreateDir and FILE_MkDir
 *
 * Revision 1.53  2001/10/17 05:13:27  slava
 * o added FILE_IsFileSeparator
 *
 * Revision 1.52  2001/10/15 02:54:24  slava
 * o cleanup
 *
 * Revision 1.51  2001/10/08 05:17:52  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.50  2001/09/22 06:10:43  slava
 * fixed formatting of an IP address
 *
 * Revision 1.49  2001/09/17 00:24:15  slava
 * o under Win32 (or at least Windows CE) inet_addr() proudly returns zero
 *   when given an empty string (supposed to return INADDR_NONE). Changed
 *   INET_ResolveAddr() to implicitely check for empty string.
 *
 * Revision 1.48  2001/09/16 16:05:16  slava
 * o enabled INET_ResolveAddr in Windows CE build
 *
 * Revision 1.47  2001/06/26 01:41:11  slava
 * o fixed Unix version of FILE_CreateDir() - it was returning an error after
 *   successfully creating the directory
 *
 * Revision 1.46  2001/06/08 08:17:50  slava
 * o added wrappers for character classification functions
 *
 * Revision 1.45  2001/05/30 09:10:48  slava
 * o moved time related functions into separate file s_time.c
 *
 * Revision 1.44  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.43  2001/05/27 11:45:51  slava
 * o port to NT kernel mode environment
 *
 * Revision 1.42  2001/05/26 21:51:44  slava
 * o moved trace functions to s_trace.c
 *
 * Revision 1.41  2001/05/20 02:47:09  slava
 * o use printf on Windows CE only in debug build because it brings up a
 *   console window.
 *
 * Revision 1.40  2001/05/18 22:36:57  slava
 * o THIS_FILE variable is no longer being used
 *
 * Revision 1.39  2001/05/18 22:33:29  slava
 * o fixed compilation error on Unix
 *
 * Revision 1.38  2001/05/18 22:29:54  slava
 * o slib has been (partially) ported to Windows CE
 *
 * Revision 1.37  2001/05/03 01:48:31  slava
 * o improved precision of TIME_Now() under Windoze
 *
 * Revision 1.36  2001/04/27 07:43:03  slava
 * o fixed comments and debug output
 *
 * Revision 1.35  2001/04/21 16:58:06  slava
 * o updated the procedure of detecting number of CPUs on Linux. It
 *   turned out that different versions of Linux have different formats
 *   of /proc/cpuinfo file. Some have "processor" entry per each processor,
 *   some have "cpus detected" field which tells the total number number
 *   of detected cpus. There are other variants, I'm sure. The current
 *   algorithm is to check for "cpus detected" first, and then if it's
 *   missing, count the "processor" fields. This works on all Linux boxes
 *   that I have access to
 *
 * Revision 1.34  2001/03/17 07:42:42  slava
 * o added SOCKET_Connect() function
 *
 * Revision 1.33  2001/03/13 06:04:37  slava
 * o added PARSE_Byte and PARSE_UByte() functions
 *
 * Revision 1.32  2001/01/31 02:14:59  slava
 * o fixed compilation problems under Windoze
 *
 * Revision 1.31  2001/01/31 02:00:41  slava
 * o portability fixes. Now compiles on Solaris
 *
 * Revision 1.30  2001/01/14 07:16:35  slava
 * o fixed compilation warning on Linux (unused variable)
 *
 * Revision 1.29  2001/01/13 16:05:00  slava
 * o added FILE_FilePart() function
 * o FileSaveCB callback now gets the destination file name rather than
 *   the temporary one. The temporary file name can be queried from the
 *   File object, if needed.
 *
 * Revision 1.28  2001/01/06 04:57:05  slava
 * o added INET_ResolveAddr() function
 *
 * Revision 1.27  2001/01/03 09:18:23  slava
 * o new file I/O support for transparent access to plain or compressed
 *   files, sockets, etc.
 *
 * Revision 1.26  2000/12/31 02:38:02  oleg
 * o changed SYSTEM_CountCPU to work correctly on 2CPU linux box
 *
 * Revision 1.25  2000/12/29 07:03:43  slava
 * o invoke WIN32_DebugPrint() from DebugTrace() to make debug trace appear
 *   on the debug console in addition to being written to standard output
 *
 * Revision 1.24  2000/12/27 03:54:27  slava
 * o changed DebugAssertFormat() to call DebugAssert() after it has
 *   formatted the error message. This way all ASSERTs and ASSMSGs
 *   end up in DebugAssert()
 *
 * Revision 1.23  2000/12/23 17:01:39  slava
 * o added ASSMSG1...ASSMSG7 assertions macros that allow formatting. For
 *   example, you can write something like ASSMSG1("Invalid index %d",i)
 *   instead of just ASSERT(FALSE)
 *
 * Revision 1.22  2000/11/17 14:29:52  slava
 * o implemented FILE_List() for Linux
 * o fixed bug in Unix version of FILE_FindSeparator
 *
 * Revision 1.21  2000/11/17 05:28:49  slava
 * o moved TEMP_FILE_NAME_LEN to header file
 * o made FILE_MakeUnique() public
 * o a bunch of new file manipulation functions: FILE_Exist, FILE_NonExist,
 *   FILE_IsDir, FILE_MkDir, FILE_RmDir and FILE_List
 *
 * Revision 1.20  2000/11/05 07:07:49  slava
 * o reorganization of output functions (Verbose(), Output(), etc.)
 *
 * Revision 1.19  2000/11/01 13:25:05  slava
 * o replaced TRUE/FALSE with True/False
 *
 * Revision 1.18  2000/11/01 03:47:21  slava
 * o WIN32_DebugPrintVa didn't restore last error
 * o OutputDebugString can print no more than 1020 bytes
 *
 * Revision 1.17  2000/10/01 00:44:20  slava
 * o made CPU_count local static variable in SYSTEM_CountCPU()
 *
 * Revision 1.16  2000/10/01 00:34:09  slava
 * o in debug build, dump trace to debug console (Win32 specific)
 *
 * Revision 1.15  2000/09/08 04:12:37  slava
 * o added trace mask so that trace level can be configured at run time
 *
 * Revision 1.14  2000/09/03 00:41:03  slava
 * o some network-related changes - IPaddr is now a 32-bit integer, added
 *   byte order definitions for Win32
 *
 * Revision 1.13  2000/09/02 12:48:19  slava
 * o some formatting
 *
 * Revision 1.12  2000/09/02 04:53:11  slava
 * o fixed warnings produced by Microsoft VC++ 6.0
 *
 * Revision 1.11  2000/09/01 11:33:17  slava
 * o added SOCKET_Wait()
 * o if socket functions fail, print the error code
 *
 * Revision 1.10  2000/09/01 02:27:27  slava
 * o implemented SOCKET_GetLastError() for Win32
 *
 * Revision 1.9  2000/08/31 05:26:34  slava
 * o added SOCKET_GetLastError
 *
 * Revision 1.8  2000/08/28 00:51:37  slava
 * o fixed bug in SOCKET_Create - port number in sockaddr_in must be in
 *   network byte order
 *
 * Revision 1.7  2000/08/26 12:11:45  slava
 * o added SOCKET_Create() and SOCKET_Close() functions
 *
 * Revision 1.6  2000/08/25 12:03:10  slava
 * o added FILE_CanOpen() function
 * o another FILE_Save() fix - if target file does not exist, just rename
 *   temp file into target
 *
 * Revision 1.5  2000/08/25 11:20:27  slava
 * o reimplemented FILE_Save so that it works on Windoze. The problem was
 *   that Win32 implementation of rename() fails if target file exists. We
 *   have to rename the target into another temp file, rename first temp
 *   file into the target and then delete the second temp file. Oh, well..
 *   I guess this makes the code more portable
 *
 * Revision 1.4  2000/08/22 04:13:53  slava
 * o implemented detection of number of CPUs for Linux (via reading
 *   /proc/stat file)
 *
 * Revision 1.3  2000/08/19 11:47:35  slava
 * o <math.h> is now included from s_os.h
 *
 * Revision 1.2  2000/08/19 05:21:12  slava
 * o fixed Win32 compile error (_CrtDbgReport undefined) and a bunch of
 *   signed/unsigned mismatch warnings
 *
 * Revision 1.1  2000/08/19 04:48:59  slava
 * o initial checkin
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "gmake -C .."
 * End:
 */
