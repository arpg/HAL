/*
 * $Id: s_fsock.c,v 1.14 2009/10/08 14:32:11 slava Exp $
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
#include "s_fio.h"
#include "s_mem.h"

/*==========================================================================*
 *              P L A I N     S O C K E T    I O
 *==========================================================================*/

#undef SocketFile
typedef struct _SocketFile {
    File file;      /* shared File structure */
    Socket sock;    /* the network socket */
    Bool eof;       /* True if we know that connection down */
} SocketFile;

STATIC SocketFile * SocketFileCast(File * f)
{
    ASSERT(f);
    if (f) {
        ASSERT(f->io == &SocketIO);
        if (f->io == &SocketIO) {
            return CAST(f,SocketFile,file);
        }
    }
    return NULL;
}

/** NOTE: both IP address and port are in host byte order */
STATIC File * SocketOpen2(IPaddr addr, Port port)
{
    Socket sock = INVALID_SOCKET;
    if (SOCKET_GetTcp(0,&sock)) {
        if (SOCKET_Connect(sock, addr, port)) {
            SocketFile * s = MEM_New(SocketFile);
            if (s) {
                Bool ok;
                StrBuf32 nameBuf;
                STRBUF_InitBufXXX(&nameBuf);
                STRBUF_Format(&nameBuf.sb, TEXT(IPADDR_FORMAT)TEXT_(":%hu"),
                    HOST_IPADDR_FORMAT_ARG(addr),port);

                memset(s, 0, sizeof(*s));
                ok = FILE_Init(&s->file, nameBuf.sb.s, False, &SocketIO);
                STRBUF_Destroy(&nameBuf.sb);

                if (ok) {
                    s->sock = sock;
                    s->eof = False;
                    return &s->file;
                }
            }            
            shutdown(sock, SHUT_RDWR);
        }
        closesocket(sock);
    }
    return NULL;
}

STATIC File * SocketOpen(Str hostname, const char * port)
{
    /* 
     * parse the port number and resolve the hostname and then call the
     * basic handler which deal with numeric port and address.
     */
    IPaddr addr;
    int p = atoi(port);
    if (p > 0 && p <= USHRT_MAX && INET_ResolveAddr(hostname,&addr)) {
        return SocketOpen2(addr, (Port)p);
    }
    return NULL;
}

STATIC Bool SocketReopen(File * f, Str hostname, const char * port)
{
    SocketFile * s  = SocketFileCast(f);
    if (s) {
        File * f2;
        shutdown(s->sock, SHUT_RDWR);
        closesocket(s->sock);
        f2 = SocketOpen(hostname, port);
        if (f2) {
            File save = *f;
            SocketFile * s2 = SocketFileCast(f2);
            *s = *s2;
            *f = save;
            MEM_Free(s2);
            return True;
        }
    }
    return False;
}

STATIC int SocketRead(File * f, void * buf, int len)
{
    int nbytes = -1;
    SocketFile * s  = SocketFileCast(f);
    if (s) {
        char * ptr = (char*)buf;
        nbytes = recv(s->sock, ptr, len, 0);
        if (nbytes > 0) {
            while (nbytes < len) {
                int n = recv(s->sock, ptr + nbytes, len - nbytes, 0);
                if (n <= 0) break;
                nbytes += n;
            }
        } else if (nbytes < 0) {
            s->eof = True;
        }
    }
    return nbytes;
}

STATIC int SocketWrite(File * f, const void * buf, int len)
{
    int nbytes = -1;
    SocketFile * s  = SocketFileCast(f);
    if (s) {
        nbytes = send(s->sock, (char*)buf, len, 0);
        if (nbytes < 0) s->eof = True;
    }
    return nbytes;
}

STATIC Bool SocketEof(File * f)
{
    SocketFile * s  = SocketFileCast(f);
    return (s ? s->eof : True);
}

STATIC int SocketFd(File * f)
{
    SocketFile * s  = SocketFileCast(f);
    return (s ? s->sock : -1);
}

STATIC Bool SocketFlush(File * f)
{
    return BoolValue(!SocketEof(f));
}

STATIC void SocketDetach(File * f)
{
    SocketFile * s  = SocketFileCast(f);
    if (s) s->sock = INVALID_SOCKET;
}

STATIC void SocketClose(File * f)
{
    SocketFile * s  = SocketFileCast(f);
    if (s) {
        shutdown(s->sock, SHUT_RDWR);
        closesocket(s->sock);
        s->sock = INVALID_SOCKET;
    }
}

STATIC void SocketFree(File * f)
{
    SocketFile * s  = SocketFileCast(f);
    if (s) {
        ASSERT(!(f->flags & FILE_IS_OPEN));
        MEM_Free(s);
    }
}

/**
 * This is an exported function that associates a "socket file" with an
 * open socket. If this call fails, the caller is responsible for closing
 * the socket. If this cal succeeds, then the socket will be closed by
 * FILE_Close() 
 */
File * FILE_AttachToSocket(Socket sock)
{
    SocketFile * sf = MEM_New(SocketFile);
    if (sf) {
        memset(sf, 0, sizeof(*sf));
        sf->sock = sock;
        sf->eof = False;
        if (FILE_Init(&sf->file, TEXT("socket"), True, &SocketIO)) {
            return &sf->file;
        }
        MEM_Free(sf);
    }
    return NULL;
}

/**
 * Connects to the specified IP address 
 * NOTE: both IP address and port are in host byte order 
 */
File * FILE_Connect(IPaddr addr, Port port)
{
    return SocketOpen2(addr, port);
}

/*
 * A set of handlers that perform file I/O
 */
const FileIO SocketIO = {
    SocketOpen          /* open     */,
    SocketReopen        /* reopen   */,
    NULL                /* setparam */,
    SocketRead          /* read     */,
    SocketWrite         /* write    */,
    NULL                /* skip     */,
    SocketFlush         /* flush    */,
    SocketEof           /* eof      */,
    SocketFd            /* fd       */,
    NULL                /* target   */,
    SocketDetach        /* detach   */,
    SocketClose         /* close    */,
    SocketFree          /* free     */,
    0                   /* flags    */
};

/*
 * HISTORY:
 *
 * $Log: s_fsock.c,v $
 * Revision 1.14  2009/10/08 14:32:11  slava
 * o added FILE_Fd() and FILE_TargetFd() functions
 *
 * Revision 1.13  2008/09/03 09:37:02  slava
 * o removed static FIO_BLOCKING_READ flag and added FILE_CAN_BLOCK flag that
 *   can be set on individual streams. This is a lot more flexible. Added
 *   FILE_CanBlock function.
 *
 * Revision 1.12  2008/09/01 12:08:15  slava
 * o replaced FileIO::fileIO field with flags. Added FIO_BLOCKING_READ flag
 *   which marks I/O methods whose read callback can block before the end of
 *   stream is reached (i.e. socket I/O).
 *
 * Revision 1.11  2008/09/01 10:57:21  slava
 * o added 'skip' I/O callback which can be implemented by those I/O methods
 *   which can efficiently skip input data without actually reading and
 *   copying anything. Currently, such callback is only implemented for
 *   in-memory I/O, but it can also be done for file I/O using fseek.
 *
 * Revision 1.10  2008/08/29 10:37:50  slava
 * o updated SocketRead to deal with the situation when recv returns
 *   less than the requested amount of data. In that case, we will
 *   issue additional recv calls until we either fill the buffer or
 *   encounter an error.
 *
 * Revision 1.9  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.8  2004/04/08 01:44:18  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.7  2003/05/21 00:11:03  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.6  2003/01/28 07:26:53  slava
 * o fixed a bug in SocketOpen2. Now FILE_Connect actually works
 *
 * Revision 1.5  2003/01/08 17:15:43  slava
 * o removed ZIP_GetTarget and WRAP_GetTarget functions, replaced them
 *   with generic FILE_Target
 *
 * Revision 1.4  2002/12/30 21:42:21  slava
 * o added fileIO field to the FileIO structure. This field is set to
 *   True if the File stream performs file based I/O (as opposed to socket
 *   or in-memory I/O)
 *
 * Revision 1.3  2002/08/27 11:52:02  slava
 * o some reformatting
 *
 * Revision 1.2  2002/01/25 03:01:27  slava
 * o no more 'compress' parameter for FILE_AttachToSocket and FILE_Connect
 *
 * Revision 1.1  2001/12/23 21:17:00  slava
 * o moved each file I/O implemenetation into a separate file. This prevents
 *   unnecessary linkage with zlib and socket library and makes executables
 *   smaller. Most linkers are not very good in removing dead references
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
