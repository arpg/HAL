/*
 * $Id: s_iop.h,v 1.4 2006/10/20 04:56:45 slava Exp $
 *
 * Copyright (C) 2000-2006 by Slava Monich
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

#ifndef _SLAVA_IO_PRIVATE_H_
#define _SLAVA_IO_PRIVATE_H_

#include "s_futil.h"
#include "s_itrp.h"

/**
 * Platform independent part of the directory iterator
 */
typedef struct _DirIterator {
    Iterator itr;           /* common part */
    DirEntry entry;         /* what we return from ITR_Next */
    StrBuf dirName;         /* the directory name */
    StrBuf fileName;        /* the last returned file name */
    Bool hasNext;           /* whether we have the next file */
} DirIterator;

/**
 * Platform independent directory iterator callbacks
 */
extern void DIR_ItrInit P_((DirIterator * di, const Itr * type));
extern void DIR_ItrDestroy P_((DirIterator * di));
extern Bool DIR_ItrHasNext P_((Iterator * itr));
extern Bool DIR_ItrRemove P_((Iterator * itr));

/**
 * Internal functions
 */
extern Bool FILE_CreateDir P_((Str dir));

#endif /* _SLAVA_IO_PRIVATE_H_ */

/*
 * HISTORY:
 *
 * $Log: s_iop.h,v $
 * Revision 1.4  2006/10/20 04:56:45  slava
 * o cleanup. moved file related utilities (most if not all of them implemented
 *   in s_futil.c) into a separate header file, s_futil.h. This may break
 *   compilation of the sources that include individual slib header files
 *   instead of including s_lib.h
 *
 * Revision 1.3  2005/05/31 06:19:42  slava
 * o fixed a typo
 *
 * Revision 1.2  2005/02/19 01:12:37  slava
 * o cleanup
 *
 * Revision 1.1  2005/02/19 00:37:41  slava
 * o separated platform specific code from platform independent code
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
