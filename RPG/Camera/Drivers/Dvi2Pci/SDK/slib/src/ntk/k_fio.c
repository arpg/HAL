/*
 * $Id: k_fio.c,v 1.3 2010/12/10 12:32:54 slava Exp $
 *
 * Copyright (C) 2010 by Slava Monich
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

#ifndef INVALID_HANDLE_VALUE
#  define INVALID_HANDLE_VALUE ((HANDLE)(LONG_PTR)-1)
#endif

/*==========================================================================*
 *              N T    K E R N E L    F I L E    I O
 *
 * Obviously, all this only works at PASSIVE_LEVEL 
 *
 *==========================================================================*/

typedef struct _NTFile {
    File file;  /* shared File structure */
    HANDLE h;   /* file handle */
} NTFile;

STATIC NTFile * NTFileCast(File * f)
{
    ASSERT(f);
    if (f) {
        ASSERT(f->io == &PlainFileIO);
        if (f->io == &PlainFileIO) {
            return CAST(f,NTFile,file);
        }
    }
    return NULL;
}

STATIC File * NTFileOpen(Str path, const char * mode)
{
    NTFile * nf = NULL; /* assume failure */
    ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);
    if (path) {
        ACCESS_MASK AccessFlags = 0;
        ULONG ShareAccess = 0;
        UNICODE_STRING FileName;
        NTSTATUS Status = STATUS_SUCCESS;
        BOOLEAN FileNameAllocated = FALSE;
        ULONG CreateDisposition = FILE_OPEN;

        if (mode) {
            while (*mode) {
                const c = *mode++;
                switch (c) {
                case 'r':
                    AccessFlags |= GENERIC_READ;
                    ShareAccess |= FILE_SHARE_READ;
                    break;
                case 'w':
                    AccessFlags |= GENERIC_WRITE;
                    ShareAccess |= FILE_SHARE_WRITE;
                    CreateDisposition = FILE_CREATE;
                    break;
                default:
                    TRACE1("WARNING: Unsupported fopen flag '%c'\n",c);
                    break;
                }
            }
        } else {
            AccessFlags = GENERIC_READ;
        }

#ifdef _UNICODE
            RtlInitUnicodeString(&FileName, path);
#else  /* !_UNICODE */

        {
            ANSI_STRING Ansi;
            RtlInitAnsiString(&Ansi, path);
            Status = RtlAnsiStringToUnicodeString(&FileName,&Ansi,TRUE);
            FileNameAllocated = NT_SUCCESS(Status);
        }
#endif /* !_UNICODE */

        if (NT_SUCCESS(Status)) {
            HANDLE FileHandle;
            IO_STATUS_BLOCK IoStatus;
            OBJECT_ATTRIBUTES Attributes;

            InitializeObjectAttributes(&Attributes, &FileName,
                (OBJ_CASE_INSENSITIVE | OBJ_KERNEL_HANDLE),
                NULL, NULL);

            Status = ZwCreateFile(&FileHandle, AccessFlags, &Attributes,
                &IoStatus, NULL, FILE_ATTRIBUTE_NORMAL, ShareAccess,
                CreateDisposition, FILE_SYNCHRONOUS_IO_NONALERT, NULL, 0);

            if (NT_SUCCESS(Status)) {
                nf = MEM_New(NTFile);
                if (nf) {
                    memset(nf, 0, sizeof(*nf));
                    nf->h = FileHandle;
                } else {
                    ZwClose(FileHandle);
                }
            } else {
                TRACE2("ZwCreateFile(%wZ) error %08X\n",&FileName,Status);
            }
            if (FileNameAllocated) RtlFreeUnicodeString(&FileName);
        }
    }
    return (nf ? &nf->file : NULL);
}

STATIC int NTFileRead(File * f, void * buf, int len)
{
    NTFile * nf = NTFileCast(f);
    ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);
    if (nf) {
        IO_STATUS_BLOCK IoStatus;
        NTSTATUS Status = ZwReadFile(nf->h, NULL, NULL, NULL, &IoStatus,
            buf, len, NULL, NULL);
        if (NT_SUCCESS(Status)) {
            return (int)IoStatus.Information;
        } else if (Status == STATUS_END_OF_FILE) {
            TRACE1("ZwReadFile EOF info = %u\n",IoStatus.Information);
            ASSERT(!IoStatus.Information);
        } else {
            TRACE1("ZwReadFile error %08X\n",Status);
        }
    }
    return (-1);
}

STATIC int NTFileWrite(File * f, const void * buf, int len)
{
    NTFile * nf = NTFileCast(f);
    ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);
    if (nf) {
        IO_STATUS_BLOCK IoStatus;
        NTSTATUS Status = ZwWriteFile(nf->h, NULL, NULL, NULL, &IoStatus,
            (PVOID)buf, len, NULL, NULL);
        if (NT_SUCCESS(Status)) {
            return (int)IoStatus.Information;
        }
        TRACE1("ZwWriteFile error %08X\n",Status);
    }
    return (-1);
}

STATIC Bool NTFileEof(File * f)
{
    NTFile * nf = NTFileCast(f);
    ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);
    if (nf) {
        IO_STATUS_BLOCK IoStatus;
        FILE_STANDARD_INFORMATION  StandardInfo;
        NTSTATUS Status = ZwQueryInformationFile(nf->h, &IoStatus,
            &StandardInfo, sizeof(StandardInfo), FileStandardInformation);
        if (NT_SUCCESS(Status)) {
            FILE_POSITION_INFORMATION PositionInfo;
            Status = ZwQueryInformationFile(nf->h, &IoStatus,
                &PositionInfo, sizeof(PositionInfo), FilePositionInformation);
            if (NT_SUCCESS(Status)) {
                ASSERT(StandardInfo.EndOfFile.QuadPart >=
                    PositionInfo.CurrentByteOffset.QuadPart);
                return (StandardInfo.EndOfFile.QuadPart <=
                    PositionInfo.CurrentByteOffset.QuadPart);
            } else {
                TRACE1("FilePositionInformation error %08X\n",Status);
            }
        } else {
            TRACE1("FileStandardInformation error %08X\n",Status);
        }
    }
    return False;
}

STATIC int NTFileFd(File * f)
{
    NTFile * nf = NTFileCast(f);
    return (nf ? (int)(nf->h) : -1);
}

STATIC void NTFileDetach(File * f)
{
    NTFile * nf = NTFileCast(f);
    if (nf) nf->h = INVALID_HANDLE_VALUE;
}

STATIC void NTFileClose(File * f)
{
    NTFile * nf = NTFileCast(f);
    ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);
    if (nf) {
        if (nf->h != INVALID_HANDLE_VALUE) {
            ZwClose(nf->h);
            nf->h = INVALID_HANDLE_VALUE;
        }
    }
}

STATIC void NTFileFree(File * f)
{
    NTFile * nf = NTFileCast(f);
    if (nf) {
        ASSERT(!(f->flags & FILE_IS_OPEN));
        MEM_Free(nf);
    }
}

/*
 * A set of handlers that perform file I/O
 */
const FileIO PlainFileIO = {
    NTFileOpen          /* open     */,
    NULL                /* reopen   */,
    NULL                /* setparam */,
    NTFileRead          /* read     */,
    NTFileWrite         /* write    */,
    NULL                /* skip     */,
    NULL                /* flush    */,
    NTFileEof           /* eof      */,
    NTFileFd            /* fd       */,
    NULL                /* target   */,
    NTFileDetach        /* detach   */,
    NTFileClose         /* close    */,
    NTFileFree          /* free     */,
    FIO_FILE_BASED      /* flags    */
};

/*
 * HISTORY:
 *
 * $Log: k_fio.c,v $
 * Revision 1.3  2010/12/10 12:32:54  slava
 * o fixed erroneous ASSERT
 *
 * Revision 1.2  2010/12/09 12:31:12  slava
 * o removed unnecessary include
 *
 * Revision 1.1  2010/12/09 12:29:39  slava
 * o added file I/O implementation for NT kernel
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
