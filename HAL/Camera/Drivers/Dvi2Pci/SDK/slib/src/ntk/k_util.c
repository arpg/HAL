/*
 * $Id: k_util.c,v 1.2 2009/12/26 12:21:32 slava Exp $
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

#include "s_mem.h"

/****************************************************************************\
 * This file contains a few utilities that are specific to the Windows NT 
 * kernel mode environment.
 ****************************************************************************/

#ifndef _NT_KERNEL
#  error "_NT_KERNEL must be defined to compile this code"
#endif

#ifdef ALLOC_PRAGMA
#pragma alloc_text(PAGE,NT_CreateDeviceObject)
#pragma alloc_text(PAGE,NT_DeleteDeviceObject)
#endif

/*==========================================================================*
 *              S T R I N G S
 *==========================================================================*/

STATIC WCHAR nt_DevicePrefix[]     = L"\\Device\\";
STATIC WCHAR nt_DosDevicePrefix[]  = L"\\DosDevices\\";

/****************************************************************************\
 *
 * NT_AllocUnicodeString()
 *
 * DESCRIPTION:
 *
 *  Allocates an empty UNICODE string
 *
 * ARGUMENTS:
 *
 *  MaxLen    - max length of the string, in characters
 *
 * RETURN VALUE:
 *
 *  Pointer to the allocated string, NULL if the allocation failed
 *
\****************************************************************************/
PUNICODE_STRING NT_AllocUnicodeString (IN USHORT MaxLen)
{
    PUNICODE_STRING String = NULL;

    ASSERT(MaxLen);
    if (MaxLen) {
        USHORT MaxByteLen = MaxLen*sizeof(WCHAR);
        USHORT NumBytes = (USHORT)(sizeof (UNICODE_STRING) + MaxByteLen);
        PUCHAR BytePtr = MEM_NewArray(UCHAR,NumBytes);
        if (BytePtr) {
            String = (PUNICODE_STRING)BytePtr;
            BytePtr += sizeof (UNICODE_STRING);

            RtlZeroMemory(String, sizeof (UNICODE_STRING) + sizeof(WCHAR));
            String->MaximumLength = MaxByteLen;
            String->Buffer = (PWSTR)BytePtr;
        }
    }

    return (String);
}

/****************************************************************************\
 *
 * NT_DupUnicodeString()
 *
 * DESCRIPTION:
 *
 *  Allocates a copy of UNICODE string
 *
 * ARGUMENTS:
 *
 *  String - string to duplicate
 *  MaxLen - max length of the new string, in characters. If zero, allocates
             just enough to copy the contents of the source
 *
 * RETURN VALUE:
 *
 *  Pointer to the new string, NULL if memory allocation failed
 *
\****************************************************************************/
PUNICODE_STRING 
NT_DupUnicodeString(IN PUNICODE_STRING String, IN USHORT MaxLen)
{
    PUNICODE_STRING NewString = NULL;
    ASSERT(String);
    if (String) {
        USHORT ActualMaxLength = MAX(MaxLen, String->Length/sizeof(WCHAR));
        NewString = NT_AllocUnicodeString(ActualMaxLength);
        if (NewString) {
            RtlCopyUnicodeString( NewString, String);
        }
    }
    return (NewString);
}

/****************************************************************************\
 *
 * NT_ConcatUnicodeStrings()
 *
 * DESCRIPTION:
 *
 *  This routine concatenates two NULL terminated strings and returns 
 *  the result as an allocated UNICODE string. 
 *
 * ARGUMENTS:
 *
 *  s1, s2 - strings to concatenate
 *  MaxLength - max length of the UNICODE string. If 0, then max length of 
 *              the allocated string equals the length of the concatenated
 *              string.
 *
 * RETURN VALUE:
 *
 *  Pointer to the allocated string, NULL if the allocation failed
 *
\****************************************************************************/
PUNICODE_STRING 
NT_ConcatUnicodeStrings( IN PWSTR s1, IN PWSTR s2, IN USHORT MaxLength )
{
    PUNICODE_STRING String;
    size_t ActualMaxLength = (wcslen(s1) + wcslen(s2) + 1);
    ASSERT( (MaxLength > ActualMaxLength) || MaxLength == 0 );
    if ( MaxLength > ActualMaxLength ) ActualMaxLength = MaxLength;
    String = NT_AllocUnicodeString( (USHORT)ActualMaxLength );
    if ( String ) {
        /* RtlAppendUnicodeToString must not fail here if we have correctly 
         * calculated the required size. */
        NTSTATUS Status = RtlAppendUnicodeToString( String, s1 );
        ASSERT( Status == STATUS_SUCCESS );
        Status = RtlAppendUnicodeToString( String, s2 );
        ASSERT( Status == STATUS_SUCCESS );
    }
    return (String);
}

/* Convenient shortcuts */
PUNICODE_STRING NT_AllocDeviceName( IN PWSTR Name )
{
    return NT_ConcatUnicodeStrings(nt_DevicePrefix,Name,0);
}

PUNICODE_STRING NT_AllocSymLinkName( IN PWSTR Name )
{
    return NT_ConcatUnicodeStrings(nt_DosDevicePrefix,Name,0);
}

/****************************************************************************\
 *
 * NT_FreeUnicodeString()
 *
 * DESCRIPTION:
 *
 *  This routine deallocates the UNICODE string previously allocated by any
 *  of the above call. 
 *
 * ARGUMENTS:
 *
 *  s - the string to deallocate. NULL is ignored
 *
\****************************************************************************/
VOID NT_FreeUnicodeString( IN PUNICODE_STRING s )
{
    MEM_Free(s);
}

/****************************************************************************\
 *
 * NT_QueryObjectName()
 *
 * DESCRIPTION:
 *
 *  Queries the name of the object (such as a device or file object).
 *  The caller must deallocate the return value with MEM_Free. The
 *  returned string (if not NULL) is guaranteed to be NULL terminated.
 *
 * ARGUMENTS:
 *
 *  obj - the object whose name to query
 *
\****************************************************************************/
OBJECT_NAME_INFORMATION * NT_QueryObjectName(PVOID obj)
{
    ULONG len = 0;
    NTSTATUS status = ObQueryNameString(obj, NULL, 0, &len);
    ASSERT(status != STATUS_SUCCESS);

    /*  Unlike the rest of the system, ObQueryNameString returns
     *  STATUS_INFO_LENGTH_MISMATCH instead of STATUS_BUFFER_TOO_SMALL 
     *  when passed too small a buffer. We expect to get this error here.
     *  Anything else we can't handle.
     */
    if (status == STATUS_INFO_LENGTH_MISMATCH && len > 0) {
        OBJECT_NAME_INFORMATION * name = MEM_Alloc(len + sizeof(WCHAR));
        if (name) {
            /* NOTE: ObQueryNameString may return STATUS_SUCCESS and empty
             * string for unnamed objects */
            status = ObQueryNameString(obj, name, len, &len);
            if (NT_SUCCESS(status) && name->Name.MaximumLength > 0) {
                /* make sure it's NULL terminated */
                ULONG Last = name->Name.Length/sizeof(name->Name.Buffer[0]);
                name->Name.Buffer[Last] = 0; 
                return name;
            }
            MEM_Free(name);
        }
    }
    return NULL;
}

/*==========================================================================*
 *              D E V I C E    O B J E C T
 *==========================================================================*/

/****************************************************************************\
 *
 * NT_CreateDeviceObject
 *
 * DESCRIPTION:
 *
 *  This routine creates the device and associated symbolic name in Win32
 *  namespace. 
 *
 * ARGUMENTS:
 *
 *  DriverObject - the driver object
 *  Name         - the device name (without prefix) and also the symbolic name
 *  ExtSize      - the device extension size
 *  Exclusive    - TRUE if this is an exclusive device
 *  DeviceObject - receives the created device object
 *  Status       - (optional) receive the status of the operation
 *
 * RETURN VALUE:
 *
 *  TRUE if the operation succeeds
 *
\****************************************************************************/
BOOLEAN 
NT_CreateDeviceObject( 
    IN  PDRIVER_OBJECT   DriverObject, 
    IN  PWSTR            Name, 
    IN  ULONG            ExtSize,
    IN  BOOLEAN          Exclusive,
    IN  DEVICE_TYPE      DeviceType,
    OUT PDEVICE_OBJECT * DeviceObject,
    OUT NTSTATUS       * Status OPTIONAL)
{
    BOOLEAN Success = FALSE;
    PUNICODE_STRING DeviceNameString;
    PUNICODE_STRING SymLinkString;
    NTSTATUS TmpStatus;

    PAGED_CODE();

    if (!Status) Status = &TmpStatus;

    /* Allocate UNICODE strings for the device name and symbolic link name */
    DeviceNameString = NT_AllocDeviceName(Name);
    if (!DeviceNameString) {
        *Status = STATUS_INSUFFICIENT_RESOURCES;
        return (FALSE);
    }
    SymLinkString = NT_AllocSymLinkName(Name);
    if (!SymLinkString) {
        NT_FreeUnicodeString(DeviceNameString);
        *Status = STATUS_INSUFFICIENT_RESOURCES;
        return (FALSE);
    }

    /* Create the device used for communications with user mode code */
    TRACE1("Creating device %wZ\n", DeviceNameString );
    *Status = IoCreateDevice(DriverObject, ExtSize, DeviceNameString, 
                             DeviceType, 0, Exclusive, DeviceObject);
    if (NT_SUCCESS(*Status)) {
        /* Create a symbolic link that the Win32 apps can specify to 
         * gain access to this driver/device. */
        TRACE1("Creating symbolic link %wZ\n", SymLinkString );
        *Status = IoCreateSymbolicLink( SymLinkString, DeviceNameString);
        if (*Status == STATUS_OBJECT_NAME_COLLISION) {
            TRACE1("Symbolic link %wZ already exists, trying again\n",SymLinkString);
            IoDeleteSymbolicLink( SymLinkString );
            *Status = IoCreateSymbolicLink( SymLinkString, DeviceNameString);
        }

        if (NT_SUCCESS(*Status)) {
            Success = TRUE;
        } else {
            TRACE2("IoCreateSymbolicLink(%wZ) failed, status %08lX\n",
                SymLinkString,Status);
	        IoDeleteDevice( *DeviceObject );
            *DeviceObject = NULL;
	    }
    } else {
        /* If device with the same name already exists, we assume that
         * it's another copy of the same driver created it, and it should 
         * be OK. We may want to drop a message into the event log though... */
        TRACE2("IoCreateDevice(%wZ) failed, status %08lX\n",DeviceNameString,
            *Status);
        switch (*Status) {
        case STATUS_OBJECT_NAME_COLLISION:
        case STATUS_OBJECT_NAME_EXISTS:
            TRACE("Ignoring this error...\n");
            Success = TRUE;
            break;
        }
    }

    /* Deallocate the temporary strings and return status to the caller */
    NT_FreeUnicodeString(SymLinkString);
    NT_FreeUnicodeString(DeviceNameString);
    return (Success);
}

/****************************************************************************\
 *
 * NT_DeleteDeviceObject
 *
 * DESCRIPTION:
 *
 *  This routine deletes the device and associated symbolic name from Win32
 *  namespace. Normally used in conjuction with NT_CreateDeviceObject
 *
 * ARGUMENTS:
 *
 *  DeviceObject - the device object to delete
 *
 *
\****************************************************************************/
VOID NT_DeleteDeviceObject( IN PDEVICE_OBJECT DeviceObject )
{
    NTSTATUS Status;
    OBJECT_NAME_INFORMATION * NameInfo;

    PAGED_CODE();

    /* To delete symbolic link, get the device name from device object. */
    NameInfo = NT_QueryObjectName(DeviceObject);
    if (NameInfo) {
        ULONG PrefixLength = (ULONG)wcslen(nt_DevicePrefix);
        ULONG PrefixSize = PrefixLength * sizeof(WCHAR);
        ULONG Index = 0;

        PUNICODE_STRING SymLink;

        /* Check if the object name start with \Device\ (It usually does) */
        if (RtlCompareMemory(NameInfo->Name.Buffer,nt_DevicePrefix,
            PrefixSize) == PrefixSize) {
            Index = PrefixLength;
        }

        SymLink = NT_AllocSymLinkName( NameInfo->Name.Buffer + Index);
        if (SymLink) {
            TRACE1("Deleting symbolic link %wZ\n",SymLink);
            Status = IoDeleteSymbolicLink( SymLink );
            if (!NT_SUCCESS(Status)) {
                TRACE2("IoDeleteSymbolicLink(%wZ) failed, status %08lX\n",
                    SymLink,Status);
            }
            NT_FreeUnicodeString(SymLink);
        }        
        MEM_Free(NameInfo);
    }

    /* Finally, delete the actual device */
    IoDeleteDevice( DeviceObject );
}

/*
 * HISTORY:
 * $Log: k_util.c,v $
 * Revision 1.2  2009/12/26 12:21:32  slava
 * o working on 64-bit issues
 *
 * Revision 1.1  2008/11/05 12:06:29  slava
 * o moved s_ntk.c from src to src/ntk directory
 *
 */
