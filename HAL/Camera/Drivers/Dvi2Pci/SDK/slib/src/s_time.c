/*
 * $Id: s_time.c,v 1.13 2009/04/09 21:54:57 slava Exp $
 *
 * Copyright (C) 2001-2009 by Slava Monich
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

/* static data used by WinCE version of TIME_ToString */
#ifdef _WIN32_WCE

#include "s_thread.h"

#define ASCBUFSIZE 26
STATIC Char TIME_ToStringBuf[ASCBUFSIZE] = {0};
STATIC ThrKey TIME_ToStringThrKey = (ThrKey)0;

#endif /* _WIN32_WCE */

/* number of milliseconds between Jan 1st 1601 and Jan 1st 1970 */
#define NT_TIME_SHIFT __INT64_C(11644473600000)

/*==========================================================================*
 *              T I M E
 *==========================================================================*/

/**
 * Returns number of milliseconds between now and  
 * January 1st 1970, 00:00 GMT
 */
Time TIME_Now() 
{
    Time now;
#if defined(_NT_KERNEL)
    LARGE_INTEGER currentTime;
    KeQuerySystemTime(&currentTime);
    now = currentTime.QuadPart;
    now /= 10000;  /* convert from 100-ns ticks to milliseconds */
    now -= NT_TIME_SHIFT;   /* Jan 1st 1601 to Jan 1st 1970 */
#elif defined(_WIN32)
    SYSTEMTIME sysTime;
    FILETIME   fileTime;
    GetSystemTime(&sysTime);
    SystemTimeToFileTime(&sysTime, &fileTime);
    now = ((I64u)fileTime.dwHighDateTime << 32) 
         + (I64u)fileTime.dwLowDateTime;
    now /= 10000;  /* convert from 100-ns ticks to milliseconds */
    now -= NT_TIME_SHIFT;   /* Jan 1st 1601 to Jan 1st 1970 */
#else /* !_WIN32 && !_NT_KERNEL */
    struct timeval tv;
#  ifdef _LINUX_KERNEL
    do_gettimeofday(&tv);
#  else /* !_LINUX_KERNEL */
    struct timezone tz;
    gettimeofday(&tv, &tz);
#  endif /* !_LINUX_KERNEL */
    now = ((Time)tv.tv_sec)*1000 + tv.tv_usec/1000;
#endif /* !_WIN32 && !_NT_KERNEL */
    return now;
}

/**
 * Converts slib time into Unix time
 */
time_t TIME_ToUnix(Time t) 
{
    return (time_t)(t/((Time)1000));
}

/**
 * Destructor for strings allocated by WinCe version of TIME_ToString
 */
#ifdef _WIN32_WCE
static void TIME_ToStringThrCleanup(void * value)
{
    MEM_Free(value);
    THREAD_SetValue(TIME_ToStringThrKey,NULL);
}
#endif /* _WIN32_WCE */

/**
 * Converts time to string
 */
#ifndef __KERNEL__

Str TIME_ToString(Time t) 
{

#ifdef _WIN32_WCE

    /*  Day/month three character abbreviations strung together */
    static const Char dnames[] = TEXT("SunMonTueWedThuFriSat");
    static const Char mnames[] = TEXT("JanFebMarAprMayJunJulAugSepOctNovDec");

    /* if nothing else works, will use static buffer */
    Char * buf = TIME_ToStringBuf;

    /* convert to 100-ns ticks since Jan 1st 1601 */
    Time ticks = (t + NT_TIME_SHIFT)*10000;

    FILETIME fileTime;
    FILETIME localTime;
    SYSTEMTIME sysTime;

    /* 
     * NOTE: due to possibility of race conditions, we may end up allocating
     * multiple slots in thread local storage. However, since it would not 
     * break anything, only waste some thread local storage, and that the
     * probability of that it extremely low, this is a minor problem and
     * may be ignored. At this point I have more important things to worry 
     * about...
     */
    if (!TIME_ToStringThrKey) {
        TIME_ToStringThrKey = THREAD_CreateKey(TIME_ToStringThrCleanup);
    }

    /*
     * NOTE: Win32 version of THREAD_CanSetValue does more that just checking
     * thread local storage key for NULL.
     */
    if (THREAD_CanSetValue(TIME_ToStringThrKey)) {
        Char * thrBuf = MEM_NewArray(Char,ASCBUFSIZE);
        if (thrBuf) {
            if (THREAD_SetValue(TIME_ToStringThrKey, thrBuf)) {
                buf = thrBuf;
            } else {
                MEM_Free(thrBuf);
            }
        }
    }

    /* translate to system time */
    fileTime.dwLowDateTime = (DWORD)(ticks);
    fileTime.dwHighDateTime = (DWORD)(ticks >> 32);
    if (FileTimeToLocalFileTime(&fileTime, &localTime) &&
        FileTimeToSystemTime(&localTime, &sysTime)) {

        int day = sysTime.wDayOfWeek * 3;   /* index to correct day string */
        int mon = (sysTime.wMonth-1) * 3;   /* index to correct month string */
        int i;
        Char * p = buf;
        for (i=0; i<3; i++, p++) {
            p[0] = dnames[day + i];
            p[4] = mnames[mon + i];
        }

        *p = _T(' ');                   /* blank between day and month */
        p += 4;
        *p++ = _T(' ');

        /* day of the month (1-31) */
        *p++ = (Char)(_T('0') + sysTime.wDay / 10);
        *p++ = (Char)(_T('0') + sysTime.wDay % 10);
        *p++ = _T(' ');

        /* hours (0-23) */
        *p++ = (Char)(_T('0') + sysTime.wHour / 10);
        *p++ = (Char)(_T('0') + sysTime.wHour % 10);
        *p++ = _T(':');

        /* minutes (0-59) */
        *p++ = (Char)(_T('0') + sysTime.wMinute / 10);
        *p++ = (Char)(_T('0') + sysTime.wMinute % 10);
        *p++ = _T(':');

        /* seconds (0-59) */
        *p++ = (Char)(_T('0') + sysTime.wSecond / 10);
        *p++ = (Char)(_T('0') + sysTime.wSecond % 10);
        *p++ = _T(' ');

        /* year */
        Sprintf(p,TEXT("%d"),sysTime.wYear);
        
    } else {
        StrCpy(buf, TEXT("<unknown>"));
    }

    return buf;

#else  /* !_WIN32_WCE */

    time_t unixTime = TIME_ToUnix(t);
    Char * s = Ctime(&unixTime);
    ASSERT(s);
    if (s) {
        size_t len = StrLen(s);
        if (len > 0 && s[len-1] == '\n') s[len-1] = 0;
    }
    return s;

#endif /* !_WIN32_WCE */
}

#endif /* !__KERNEL__ */

/*
 * HISTORY:
 *
 * $Log: s_time.c,v $
 * Revision 1.13  2009/04/09 21:54:57  slava
 * o use size_t instead of int where it's appropriate
 *
 * Revision 1.12  2005/02/21 02:16:40  slava
 * o build errors
 *
 * Revision 1.11  2005/02/20 20:31:30  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.10  2003/12/04 04:48:55  slava
 * o replaced _LINUX_KERNEL_ with _LINUX_KERNEL for consistency with other
 *   platform defines such as _UNIX, _WIN32, _NT_KERNEL etc.
 *
 * Revision 1.9  2003/11/30 02:49:58  slava
 * o port to Linux kernel mode environment
 *
 * Revision 1.8  2003/03/18 16:52:44  slava
 * o added Unicode build configurations for Windoze
 *
 * Revision 1.7  2002/10/26 13:51:28  slava
 * o corrected a few typos in the comments
 *
 * Revision 1.6  2002/07/01 05:16:33  slava
 * o bug fix: WinCE version of TIME_ToString didn't translate time into
 *   local time zone
 *
 * Revision 1.5  2002/07/01 04:43:10  slava
 * o renamed TIME_Convert to TIME_ToUnix
 *
 * Revision 1.4  2002/07/01 02:33:39  slava
 * o implemented TIME_ToString for WinCE
 *
 * Revision 1.3  2001/10/08 05:17:52  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.2  2001/05/31 16:09:04  slava
 * o fixed TIME_Now() for NT kernel mode
 *
 * Revision 1.1  2001/05/30 09:10:48  slava
 * o moved time related functions into separate file s_time.c
 *
 * Local Variables:
 * c-basic-offset:4
 * indent-tabs-mode: nil
 * End:
 */
