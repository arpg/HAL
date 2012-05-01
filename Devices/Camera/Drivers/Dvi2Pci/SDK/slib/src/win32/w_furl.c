/*
 * $Id: w_furl.c,v 1.10 2009/10/08 14:32:11 slava Exp $
 *
 * Copyright (C) 2006-2009 by Slava Monich
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

#include "s_fio.h"
#include "s_util.h"
#include "s_mem.h"

/* This functionality required wininet.dll */
#include <wininet.h>

/*==========================================================================*
 *              W I N D O W S    U R L    I / O
 *==========================================================================*/

typedef struct _InetFile {
    File file;         /* shared File structure */
    HINTERNET hinet;   /* Internet session handle */
    HINTERNET hconn;   /* Internet connection handle */
    HINTERNET hreq;    /* Internet request handle */
    Bool eof;          /* True if we know that connection down */
} InetFile;

STATIC int  InetRead   P_((File * f, void * buf, int len));
STATIC int  InetWrite  P_((File * f, const void * buf, int len));
STATIC Bool InetEof    P_((File * f));
STATIC Bool InetFlush  P_((File * f));
STATIC void InetDetach P_((File * f));
STATIC void InetClose  P_((File * f));
STATIC void InetFree   P_((File * f));

/*
 * Table of I/O handlers
 */
STATIC const FileIO InetIO = {
    NULL        /* open     */,
    NULL        /* reopen   */,
    NULL        /* setparam */,
    InetRead    /* read     */,
    InetWrite   /* write    */,
    NULL        /* skip     */,
    InetFlush   /* flush    */,
    InetEof     /* eof      */,
    NULL        /* fd       */,
    NULL        /* target   */,
    InetDetach  /* detach   */,
    InetClose   /* close    */,
    InetFree    /* free     */,
    0           /* flags    */
};

/*
 * Utilities
 */
#if DEBUG
#  define WININET_ERROR(function) InetPrintError(TEXT(function))
STATIC void InetPrintError(Str function)
{
    Char msg[256];
    DWORD msglen = COUNT(msg);
    DWORD err = GetLastError();
    DWORD exterr = err;
    if (err == ERROR_INTERNET_EXTENDED_ERROR &&
        InternetGetLastResponseInfo(&exterr, msg, &msglen)) {
        TRACE3("SLIB: %s error %lu (%s)\n", function, err, msg);
    } else {
        TRACE2("SLIB: %s error %lu\n", function, err);
    }
}
#else /* !DEBUG */
#  define WININET_ERROR(function) NOTHING
#endif /* !DEBUG */

STATIC InetFile * InetFileCast(File * f)
{
    ASSERT(f);
    if (f) {
        ASSERT(f->io == &InetIO);
        if (f->io == &InetIO) {
            return CAST(f,InetFile,file);
        }
    }
    return NULL;
}

/*
 * I/O handlers
 */
STATIC int InetRead(File * f, void * buf, int len)
{
    InetFile * inet  = InetFileCast(f);
    if (inet) {
        DWORD bytesRead = 0;
        if (InternetReadFile(inet->hreq, buf, len, &bytesRead)) {
            if (!bytesRead) inet->eof = True;
            return bytesRead;
        } else {
            WININET_ERROR("InternetReadFile");
        }
        inet->eof = True;
    }
    return -1;
}

STATIC int InetWrite(File * f, const void * buf, int len)
{
    InetFile * inet  = InetFileCast(f);
    if (inet) {
        DWORD bytesWritten = 0;
        if (InternetWriteFile(inet->hreq, buf, len, &bytesWritten)) {
            return bytesWritten;
        } else {
            WININET_ERROR("InternetWriteFile");
        }
    }
    return -1;
}

STATIC Bool InetEof(File * f)
{
    InetFile * inet  = InetFileCast(f);
    return (inet ? inet->eof : True);
}

STATIC Bool InetFlush(File * f)
{
    return BoolValue(!InetEof(f));
}

STATIC void InetDetach(File * f)
{
    InetFile * inet  = InetFileCast(f);
    if (inet) {
        inet->hinet = NULL;
        inet->hconn = NULL;
        inet->hreq = NULL;
    }
}

STATIC void InetClose(File * f)
{
    InetFile * inet  = InetFileCast(f);
    if (inet) {
        if (inet->hreq) {
            InternetCloseHandle(inet->hreq);
            inet->hreq = NULL;
        }
        if (inet->hconn) {
            InternetCloseHandle(inet->hconn);
            inet->hconn = NULL;
        }
        if (inet->hinet) {
            InternetCloseHandle(inet->hinet);
            inet->hinet = NULL;
        }
    }
}

STATIC void InetFree(File * f)
{
    MEM_Free(InetFileCast(f));
}

/**
 * Opens request handle (and optionally, connection handle) for the
 * specified URL.
 */
STATIC Bool InetConnect(HINTERNET hinet, Str szURL, Str usr, Str pwd,
                        HINTERNET * hconn, HINTERNET * hreq)
{
    const DWORD flags = INTERNET_FLAG_NO_CACHE_WRITE | INTERNET_FLAG_RELOAD;
    URL_COMPONENTS url;

    if (!usr && !pwd) {
        /* Simple case */
        *hreq = InternetOpenUrl(hinet, szURL, NULL, 0, flags, 0);
        if (*hreq) {
            return True;
        } else {
            WININET_ERROR("InternetOpenUrl");
            return False;
        }
    }

    ZeroMemory(&url, sizeof(url));
    url.dwStructSize = sizeof(url);
    url.dwSchemeLength = 1;
    url.dwHostNameLength = 1;
    url.dwUserNameLength = 1;
    url.dwPasswordLength = 1;
    url.dwUrlPathLength = 1;
    if (InternetCrackUrl(szURL, 0, 0, &url)) {
        if (url.nScheme == INTERNET_SCHEME_HTTP ||
            url.nScheme == INTERNET_SCHEME_FTP) {
            StrBuf64 host;
            STRBUF_InitBufXXX(&host);
            if (STRBUF_CopyN(&host.sb,url.lpszHostName,url.dwHostNameLength)){
                
                /*
                 * Username and password passed directly to InternetConnect
                 * don't always work. They seem to work for FTP connections.
                 * but in HTTP case we need to create a connection handle
                 * and then InternetSetOption to associate username and
                 * password with it. So we do both, just in case.
                 */

                *hconn = InternetConnect(hinet, STRBUF_Text(&host.sb),
                    url.nPort, usr, pwd, url.nScheme, flags, 0);
                STRBUF_Destroy(&host.sb);

                if (*hconn) {

                        /* Set username*/
                    if ((!usr || InternetSetOption(*hconn,
                        INTERNET_OPTION_USERNAME, (void*)usr,
                        (DWORD)(sizeof(*usr)*(StrLen(usr)+1)))) &&
                        
                        /* and password */
                        (!pwd || InternetSetOption(*hconn,
                        INTERNET_OPTION_PASSWORD, (void*)pwd,
                        (DWORD)(sizeof(*pwd)*(StrLen(pwd)+1))))) {

                        /* Protocol-specific action */
                        if (url.nScheme == INTERNET_SCHEME_HTTP) {

                            /* HTTP */
                            static LPCTSTR types[] = {TEXT("*/*"),NULL};
                            *hreq = HttpOpenRequest(*hconn, NULL,
                                url.lpszUrlPath, NULL, NULL, types, flags, 0);
                            if (*hreq) {
                                if (HttpSendRequest(*hreq, NULL, 0, NULL, 0)) {
                                    return True;
                                }

                                WININET_ERROR("HttpSendRequest");
                                InternetCloseHandle(*hreq);
                                *hreq = NULL;

                            } else {
                                WININET_ERROR("HttpOpenRequest");
                            }

                        } else if (url.nScheme == INTERNET_SCHEME_FTP) {

                            /* FTP */
                            *hreq = FtpOpenFile(*hconn, url.lpszUrlPath,
                                GENERIC_READ, FTP_TRANSFER_TYPE_BINARY, 0);
                            if (*hreq) {
                                return True;
                            }

                            /* Try again without leading slash */
                            if (url.lpszUrlPath && *url.lpszUrlPath == '/') {
                                *hreq = FtpOpenFile(*hconn, url.lpszUrlPath+1,
                                    GENERIC_READ, FTP_TRANSFER_TYPE_BINARY, 0);
                                if (*hreq) {
                                    return True;
                                }
                            }

                            WININET_ERROR("FtpOpenFile");

                        } else {
                            ASSMSG1("Unsupported scheme %d", url.nScheme);
                        }
                    } else {
                        WININET_ERROR("InternetSetOption");
                    }

                    InternetCloseHandle(*hconn);
                    *hconn = NULL;
                } else {
                    WININET_ERROR("InternetConnect");
                }
            } else {
                STRBUF_Destroy(&host.sb);
            }
        } else {
            TRACE1("SLIB: unsupported authentication scheme %d",url.nScheme);
        }

        /* Do the best we can without authentication */
        *hreq = InternetOpenUrl(hinet, szURL, NULL, 0, flags, 0);
        if (*hreq) {
            return True;
        } else {
            WININET_ERROR("InternetOpenUrl");
        }

    } else {
        TRACE1("SLIB: can't parse URL %s", szURL);
        WININET_ERROR("InternetCrackUrl");
    }

    return False;
}

/**
 * Connects to the specified URL.
 */
File * FILE_AuthURL(Str url, Str username, Str password)
{
    HINTERNET hinet = InternetOpen(TEXT("SLIB ") SLIB_VERSION_TEXT,
        INTERNET_OPEN_TYPE_PRECONFIG, NULL, NULL, 0);
    if (hinet) {
        HINTERNET hconn = NULL;
        HINTERNET hreq = NULL;
        if (InetConnect(hinet, url, username, password, &hconn, &hreq)) {
            InetFile * inet = MEM_New(InetFile);
            if (inet) {
                if (FILE_Init(&inet->file, url, False, &InetIO)) {
                    inet->hinet = hinet;
                    inet->hconn = hconn;
                    inet->hreq = hreq;
                    inet->eof = False;
                    return &inet->file;
                }
                MEM_Free(inet);
            }
            if (hreq) InternetCloseHandle(hreq);
            if (hconn) InternetCloseHandle(hconn);
        }
        InternetCloseHandle(hinet);
    } else {
        WININET_ERROR("InternetOpen");
    }
    return NULL;
}

/**
 * Connects to the specified URL.
 */
File * FILE_OpenURL(Str url)
{
    return FILE_AuthURL(url, NULL, NULL);
}

/*
 * HISTORY:
 *
 * $Log: w_furl.c,v $
 * Revision 1.10  2009/10/08 14:32:11  slava
 * o added FILE_Fd() and FILE_TargetFd() functions
 *
 * Revision 1.9  2009/04/09 21:36:26  slava
 * o fixed a warning in 64-bit Windows build
 *
 * Revision 1.8  2009/02/10 12:30:26  slava
 * o implemented FILE_AuthURL for Windows
 *
 * Revision 1.7  2008/09/01 12:08:15  slava
 * o replaced FileIO::fileIO field with flags. Added FIO_BLOCKING_READ flag
 *   which marks I/O methods whose read callback can block before the end of
 *   stream is reached (i.e. socket I/O).
 *
 * Revision 1.6  2008/09/01 10:57:21  slava
 * o added 'skip' I/O callback which can be implemented by those I/O methods
 *   which can efficiently skip input data without actually reading and
 *   copying anything. Currently, such callback is only implemented for
 *   in-memory I/O, but it can also be done for file I/O using fseek.
 *
 * Revision 1.5  2008/05/10 20:13:17  slava
 * o force reload from the server; don't write to the cache
 *
 * Revision 1.4  2007/06/27 21:53:44  slava
 * o fixed Unicode debug build
 *
 * Revision 1.3  2007/05/20 22:09:40  slava
 * o fixed Unicode build
 *
 * Revision 1.2  2007/02/07 21:38:26  slava
 * o improved debug output
 *
 * Revision 1.1  2007/02/07 04:01:56  slava
 * o implemented FILE_OpenURL on Windows.
 *
 */
