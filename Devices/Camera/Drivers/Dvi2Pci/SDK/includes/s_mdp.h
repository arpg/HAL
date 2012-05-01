/*
 * $Id: s_mdp.h,v 1.2 2009/04/09 21:17:17 slava Exp $
 *
 * Copyright (C) 2000-2009 by Slava Monich
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

#ifndef _SLAVA_MD_PRIVATE_H_
#define _SLAVA_MD_PRIVATE_H_

#include "s_md.h"

/* Message digest callbacks */
typedef void (*DigestInit)   P_((Digest * d));
typedef void (*DigestUpdate) P_((Digest * d, const void * data, size_t size));
typedef void (*DigestFinish) P_((Digest * d, void * out));
typedef void (*DigestFree)   P_((Digest * d));

/* message digest type definition */
typedef struct _DigestType {
    Str name;               /* digest name */
    int size;               /* size of the message digest */
    DigestInit init;        /* re-initializes the message digest */
    DigestUpdate update;    /* updates the digest with new data */
    DigestFinish finish;    /* returns the final digest */
    DigestFree free;        /* deallocates the digest */
} DigestType;

/* command part of the dogest objects */
struct _Digest {
    const DigestType* type; /* pointer to the digest type */
    int flags;              /* flags: */
#define MD_FINISHED 0x01    /* finished but not re-initialized yet */
#define MD_INITIAL  0x00    /* initial flags (none) */
};

#endif /* _SLAVA_MD_PRIVATE_H_ */

/*
 * HISTORY:
 *
 * $Log: s_mdp.h,v $
 * Revision 1.2  2009/04/09 21:17:17  slava
 * o user size_t instead of int where it's appropriate
 *
 * Revision 1.1  2004/07/29 17:09:37  slava
 * o message digest framework
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
