/*
 * $Id: u_furl.c,v 1.7 2009/12/17 23:11:25 slava Exp $
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
#include "s_mem.h"
#include "s_util.h"
#include "s_event.h"
#include "s_thread.h"

/* This functionality requires curl */
#ifdef _HAVE_CURL

#include <curl/curl.h>
#include <curl/easy.h>

#ifdef __APPLE__
#  include <SystemConfiguration/SystemConfiguration.h>
#endif /* __APPLE__ */

/*==========================================================================*
 *              C U R L    U R L    I / O
 *==========================================================================*/

typedef struct _CurlFile {
    File file;         /* shared File structure */
    ThrID thr;         /* handle to the curl thread */
    Event readEvent;   /* set by the curl thread when receive completes */
    Event readReq;     /* set when receive has been requested */
    char * userPwd;    /* "name:password" to use when fetching */
    char * readBuf;    /* buffer to write received data */
    int readSize;      /* size of the receive buffer */
    int readLen;       /* number of bytes in the receive buffer */
    Bool eof;          /* set by read thread if there will no more reads */
} CurlFile;

STATIC int  CurlRead   P_((File * f, void * buf, int len));
STATIC int  CurlWrite  P_((File * f, const void * buf, int len));
STATIC Bool CurlEof    P_((File * f));
STATIC void CurlClose  P_((File * f));
STATIC void CurlFree   P_((File * f));

/*
 * Table of I/O handlers
 */
STATIC const FileIO CurlIO = {
    NULL        /* open     */,
    NULL        /* reopen   */,
    NULL        /* setparam */,
    CurlRead    /* read     */,
    CurlWrite   /* write    */,
    NULL        /* skip     */,
    NULL        /* flush    */,
    CurlEof     /* eof      */,
    NULL        /* fd       */,
    NULL        /* target   */,
    NULL        /* detach   */,
    CurlClose   /* close    */,
    CurlFree    /* free     */,
    0           /* flags    */
};

/*
 * CURL thread
 */
STATIC size_t CurlWriteFunc(void * data, size_t size, size_t nitems, void * f)
{
    CurlFile * cf = f;
    const char * src = data;
    int len = (size * nitems);
    int pos = 0;
    while (pos < len) {
        EVENT_Wait(&cf->readReq);
        if (cf->eof) {
            return 0;
        } else {
            int bytesToCopy = MIN(len - pos, cf->readSize - cf->readLen);
            ASSERT(bytesToCopy > 0);
            if (bytesToCopy > 0) {
                memcpy(cf->readBuf + cf->readLen, src + pos, bytesToCopy);
                cf->readLen += bytesToCopy;
                pos += bytesToCopy;
                if (cf->readLen == cf->readSize) {
                    EVENT_Reset(&cf->readReq);
                    EVENT_Set(&cf->readEvent);
                }
            } else {
                return 0;
            }
        }
    }
    return len;
}

#ifdef __APPLE__
/* Converts CFStringRef to a UTF-8 string */
static char* CFStringRef_ToUTF8(CFStringRef strRef)
{
    if (strRef) {
        CFStringEncoding enc = kCFStringEncodingUTF8;
        size_t strLen = CFStringGetLength(strRef);
        size_t bufSize = CFStringGetMaximumSizeForEncoding(strLen, enc);
        char *str = MEM_Alloc(bufSize);
        if (str) {
            str[0] = 0; 
            if (CFStringGetCString(strRef, str, bufSize, enc)) {
                str[bufSize-1] = 0;
                return str;
            }
            MEM_Free(str);
        }
    }
    return NULL;
}
#endif /* __APPLE__ */

STATIC void CurlThread(void * arg)
{
    CurlFile * cf = arg;
    CURL * curl = curl_easy_init();
    if (curl) {
        CURLcode result;

#ifdef __APPLE__
        /* Auto-detect proxy information on Mac OS X */
        int colon = STRING_IndexOf(cf->file.name, ':');
        if (colon > 0) {
            CFDictionaryRef proxyDict = SCDynamicStoreCopyProxies(NULL);
            if (proxyDict) {
                CFStringRef enabledKey = NULL, hostKey = NULL, portKey = NULL;

                /* Extract scheme from the URL */
                char scheme[16];
                int32_t num = MIN(colon, COUNT(scheme)-1);
                strncpy(scheme, cf->file.name, num);
                scheme[num] = 0;

                /* Each scheme has a key of its own */
                if (!StrCaseCmp(scheme, "http")) {
                    enabledKey = kSCPropNetProxiesHTTPEnable;
                    hostKey = kSCPropNetProxiesHTTPProxy;
                    portKey = kSCPropNetProxiesHTTPPort;
                } else if (!StrCaseCmp(scheme, "https")) {
                    enabledKey = kSCPropNetProxiesHTTPSEnable;
                    hostKey = kSCPropNetProxiesHTTPSProxy;
                    portKey = kSCPropNetProxiesHTTPSPort;
                } else if (!StrCaseCmp(scheme, "ftp")) {
                    enabledKey = kSCPropNetProxiesFTPEnable;
                    hostKey = kSCPropNetProxiesFTPProxy;
                    portKey = kSCPropNetProxiesFTPPort;
                } else if (!StrCaseCmp(scheme, "gopher")) {
                    enabledKey = kSCPropNetProxiesGopherEnable;
                    hostKey = kSCPropNetProxiesGopherProxy;
                    portKey = kSCPropNetProxiesGopherPort;
                }

                /* Is it a known scheme? */
                if (enabledKey) {

                    /* Is proxy enabled for this scheme? */
                    CFNumberRef numRef;
                    numRef = CFDictionaryGetValue(proxyDict, enabledKey);
                    if (numRef) {
                        num = 0;
                        CFNumberGetValue(numRef, kCFNumberSInt32Type, &num);
                        if (num) {

                            /* It is enabled. Get the host and port */
                            CFStringRef hostRef;
                            hostRef = CFDictionaryGetValue(proxyDict, hostKey);
                            if (hostRef) {
                                char *host = CFStringRef_ToUTF8(hostRef);
                                if (host) {
                                    curl_easy_setopt(curl,CURLOPT_PROXY,host);
                                    numRef = CFDictionaryGetValue(proxyDict,
                                        portKey);
                                    if (numRef) {
                                        num = 0;
                                        CFNumberGetValue(numRef,
                                            kCFNumberSInt32Type, &num);
                                        if (num) {
                                            curl_easy_setopt(curl,
                                                CURLOPT_PROXYPORT, num);
                                        }
                                    }
                                    MEM_Free(host);
                                }
                            }
                        }
                    }
                }

                CFRelease(proxyDict);
            }
        }
#endif /* __APPLE__ */

        curl_easy_setopt(curl, CURLOPT_URL, cf->file.name);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, "SLIB " SLIB_VERSION_TEXT);
        curl_easy_setopt(curl, CURLOPT_FILE, cf);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, CurlWriteFunc);
        curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 20);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
        if (cf->userPwd) curl_easy_setopt(curl, CURLOPT_USERPWD, cf->userPwd);
        result = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        /* synchronize with the read thread */
        EVENT_Wait(&cf->readReq);
        if (!cf->eof) {
            if (cf->readLen > 0) {
                /* flush buffered data */
                EVENT_Reset(&cf->readReq);
                EVENT_Set(&cf->readEvent);
                EVENT_Wait(&cf->readReq);
            }
            if (!cf->eof) {
                /* complete the final read */
                cf->readLen = ((result == CURLE_OK) ? 0 : (-1));
                EVENT_Reset(&cf->readReq);
                EVENT_Set(&cf->readEvent);
            }
        }
    }
}

/*
 * I/O handlers
 */
STATIC CurlFile * CurlFileCast(File * f)
{
    ASSERT(f);
    if (f) {
        ASSERT(f->io == &CurlIO);
        if (f->io == &CurlIO) {
            return CAST(f,CurlFile,file);
        }
    }
    return NULL;
}

STATIC int CurlRead(File * f, void * buf, int len)
{
    CurlFile * cf  = CurlFileCast(f);
    if (cf && !cf->eof) {
        ASSERT(!cf->readBuf);
        cf->readBuf = buf;
        cf->readSize = len;
        cf->readLen = 0;
        EVENT_Set(&cf->readReq);
        EVENT_Wait(&cf->readEvent);
        EVENT_Reset(&cf->readEvent);
        cf->readBuf = NULL;
        cf->readSize = 0;
        if (cf->readLen <= 0) {
            cf->eof = True;
        }
        return cf->readLen;
    }
    return -1;
}

STATIC int CurlWrite(File * f, const void * buf, int len)
{
    /* not supported */
    return -1;
}

STATIC Bool CurlEof(File * f)
{
    CurlFile * cf  = CurlFileCast(f);
    return (cf ? cf->eof : True);
}

STATIC void CurlClose(File * f)
{
    CurlFile * cf  = CurlFileCast(f);
    if (cf) {
        /* make sure that the curl thread is dead */
        cf->eof = True;
        EVENT_Set(&cf->readReq);
        THREAD_Join(cf->thr);
    }
}

STATIC void CurlFree(File * f)
{
    CurlFile * cf  = CurlFileCast(f);
    if (cf) {
        MEM_Free(cf->userPwd);
        MEM_Free(cf);
    }
}

/**
 * Connects to the specified URL.
 */
File * FILE_AuthURL(Str url, Str username, Str password)
{
    ASSERT(url);
    if (url) {
        CurlFile * cf = MEM_New(CurlFile);
        if (cf) {
            memset(cf, 0, sizeof(*cf));
            if (EVENT_Init(&cf->readEvent)) {
                if (EVENT_Init(&cf->readReq)) {
                    if (username || password) {
                        int len = 0;
                        if (username) len += strlen(username);
                        if (password) len += strlen(password);
                        cf->userPwd = MEM_NewArray(char, len+2);
                        if (cf->userPwd) {
                            cf->userPwd[0] = 0;
                            if (username) strcpy(cf->userPwd, username);
                            strcat(cf->userPwd, ":");
                            if (password) strcat(cf->userPwd, password);
                        }
                    }

                    if (!(username || password) || cf->userPwd) {
                        cf->readBuf = NULL;
                        cf->readSize = 0;
                        cf->readLen = 0;
                        cf->eof = False;
                        if (FILE_Init(&cf->file, url, False, &CurlIO)) {
                            cf->file.flags &= (~FILE_CAN_WRITE);
                            if (THREAD_Create(&cf->thr, CurlThread, cf)) {
                                return &cf->file;
                            }
                            FILE_Destroy(&cf->file);
                        }
                        MEM_Free(cf->userPwd);
                    }
                    EVENT_Destroy(&cf->readReq);
                }
                EVENT_Destroy(&cf->readEvent);
            }
            MEM_Free(cf);
        }
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

#endif /* _HAVE_CURL */

/*
 * HISTORY:
 *
 * $Log: u_furl.c,v $
 * Revision 1.7  2009/12/17 23:11:25  slava
 * o auto-detect proxy information on Mac OS X
 *
 * Revision 1.6  2009/11/30 09:00:42  slava
 * o immediately return -1 from CurlRead if EOF flag is set
 *
 * Revision 1.5  2009/10/08 14:32:11  slava
 * o added FILE_Fd() and FILE_TargetFd() functions
 *
 * Revision 1.4  2009/02/09 23:02:16  slava
 * o added FILE_AuthURL function
 *
 * Revision 1.3  2008/09/04 10:12:47  slava
 * o adapted to changes in FileIO structure
 *
 * Revision 1.2  2007/02/08 18:26:39  slava
 * o set CURLOPT_USERAGENT and more importantly CURLOPT_MAXREDIRS and
 *   CURLOPT_FOLLOWLOCATION options. libcurl doesn't follow redirects by
 *   default.
 *
 * Revision 1.1  2007/02/08 17:53:33  slava
 * o added Unix implementation of URL stream using libcurl. Not particularly
 *   efficient. Each FILE_OpenURL call creates its own thread.
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
